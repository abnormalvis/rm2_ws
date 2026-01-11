//
// Created by qiayuan on 23-4-13.
//

#include "rm2_ecat_ros/RmStandardSlaveManagerRos.h"
#include "rm2_ecat_ros/RosMsgConversions.h"

#include <any_node/Topic.hpp>
#include <memory>
#include <message_logger/log/log_messages.hpp>
#include <rclcpp/publisher.hpp>

namespace rm2_ecat {
namespace standard {
rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings RmStandardSlaveManagerRos::getReadingsMsg() {
  std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
  return readingsMsg_;
}

sensor_msgs::msg::JointState RmStandardSlaveManagerRos::getJointStateMsg() {
  std::lock_guard<std::recursive_mutex> jointStatesMsgLock(jointStatesMsgMutex_);
  return jointStatesMsg_;
}

std::vector<sensor_msgs::msg::Imu> RmStandardSlaveManagerRos::getImuMsgs() {
  std::lock_guard<std::recursive_mutex> readingsMsgLock(imuMsgsMutex_);
  return imuMsgs_;
}

void RmStandardSlaveManagerRos::updateProcessReadings() {
  ClearSlaveManagerRosBase<RmEcatStandardSlaveManager>::updateProcessReadings();
  const auto& readings = getReadings<rm2_ecat::standard::Reading>();
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
    readingsMsg_ = createRmSlaveReadingsMsg(getNamesOfSlaves(), readings);
    if (readingsPublisher_) {
      readingsPublisher_->publish(readingsMsg_);
    }
    readingsMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(jointStatesMsgMutex_);
    jointStatesMsg_ = createRmJointStateMsg(readings);
    if (jointStatesPublisher_) {
      jointStatesPublisher_->publish(jointStatesMsg_);
    }
    jointStatesMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(imuMsgsMutex_);
    imuMsgs_ = createImuMsgs(readings);
    for (size_t i = 0; i < imuPublishers_.size(); ++i) {
      imuPublishers_[i]->publish(imuMsgs_[i]);
    }
    imuMsgsUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(dbusDatasMutex_);
    dbusDatas = createDbusDatas(readings);
    for (size_t i = 0; i < dbusDatasPublishers_.size(); ++i) {
      dbusDatasPublishers_[i]->publish(dbusDatas[i]);
    }
    dbusDatasUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(gpioDatasMutex_);
    gpioDatas = createRmGpioDatas(readings);
    for (size_t i = 0; i < gpioDatasPublishers_.size(); ++i) {
      gpioDatasPublishers_[i]->publish(gpioDatas[i]);
    }
    gpioDatasUpdated_ = true;
  }
  size_t triggerTimePublisherIndex = 0;
  const auto slaves = getSlaves();
  for (const auto& slave : slaves) {
    const auto reading = slave->getReading();
    const auto buss = reading.getEnabledImuBuss();
    const auto statusWord = reading.getStatusword();
    for (const auto& bus : buss) {
      if (statusWord.isTriggered(bus)) {
        imuTriggerTimePublishers_[triggerTimePublisherIndex]->publish(createTimeReferenceMsg(reading.getStamp()));
      }
      triggerTimePublisherIndex++;
    }
  }
}

void RmStandardSlaveManagerRos::sendRos() {
  if (readingsMsgUpdated_) {
    readingsMsgUpdated_ = false;
    readingsPublisher_->sendRos();
  }
  if (jointStatesMsgUpdated_) {
    jointStatesMsgUpdated_ = false;
    jointStatesPublisher_->sendRos();
  }
  if (imuMsgsUpdated_) {
    imuMsgsUpdated_ = false;
    for (auto& imuPublisher : imuPublishers_) {
      imuPublisher->sendRos();
    }
  }
  if(dbusDatasUpdated_){
    dbusDatasUpdated_ = false;
    for (auto& dbusPublisher : dbusDatasPublishers_) {
      dbusPublisher->sendRos();
    }
  }
  if (gpioDatasUpdated_) {
    gpioDatasUpdated_ = false;
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->sendRos();
    }
  }
}

void RmStandardSlaveManagerRos::startupRosInterface() {
  shutdownPublishWorkerRequested_ = false;

  readingsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings>>(
      node_->create_publisher<rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings>("rm_readings", 10), 50, false);
  readingsMsgUpdated_ = false;

  jointStatesPublisher_ = std::make_shared<any_node::ThreadedPublisher<sensor_msgs::msg::JointState>>(
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10), 50, false);
  jointStatesMsgUpdated_ = false;

  const auto slaves = getSlaves();
  for (const auto& slave : slaves) {
    const auto buss = slave->getReading().getEnabledImuBuss();
    for (const auto& bus : buss) {
      imuPublishers_.push_back(std::make_shared<any_node::ThreadedPublisher<sensor_msgs::msg::Imu>>(
          node_->create_publisher<sensor_msgs::msg::Imu>(slave->getReading().getImuName(bus), 10), 50, false));
      imuTriggerTimePublishers_.push_back(
          node_->create_publisher<sensor_msgs::msg::TimeReference>(slave->getReading().getImuName(bus) + "/trigger_time", 10));
    }
  }
  imuMsgsUpdated_ = false;

  imusTriggerServer_ = node_->create_service<rm2_msgs::srv::EnableImuTrigger>("imu_trigger",std::bind(&RmStandardSlaveManagerRos::imuTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));

  for (const auto& slave : slaves) {
    if(slave->getReading().getEnabledDbus())
    {
      dbusDatasPublishers_.push_back(std::make_shared<any_node::ThreadedPublisher<rm2_msgs::msg::DbusData>>(
          node_->create_publisher<rm2_msgs::msg::DbusData>(slave->getReading().getDbusName(), 10), 50, false));
    }
  }

  for (const auto& slave : slaves) {
    if (!slave->getReading().getEnabledDigitalInputIds().empty()) {
      gpioDatasPublishers_.push_back(std::make_shared<any_node::ThreadedPublisher<rm2_msgs::msg::GpioData>>(
          node_->create_publisher<rm2_msgs::msg::GpioData>(slave->getName() + "_gpios", 10), 50, false));
    }
  }

  if (standalone_) {
    publishWorker_->start(20);
  }
}

void RmStandardSlaveManagerRos::shutdownRosInterface() {
  if (!shutdownPublishWorkerRequested_) {
    jointStatesPublisher_->shutdown();
    readingsPublisher_->shutdown();
    for (auto& imuPublisher : imuPublishers_) {
      imuPublisher->shutdown();
    }
    for (auto& dbusPublisher: dbusDatasPublishers_) {
      dbusPublisher->shutdown();
    }
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->shutdown();
    }
  }
}

void RmStandardSlaveManagerRos::imuTriggerCallback(const std::shared_ptr<rm2_msgs::srv::EnableImuTrigger::Request> req,
                        std::shared_ptr<rm2_msgs::srv::EnableImuTrigger::Response> res) {
  for (auto& slave : slaves_) {
    const auto reading = slave->getReading();
    const auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      if ((reading.getImuName(bus) == req->imu_name)) {
        slave->setImuTrigger(bus, req->enable_trigger);  // TODO: thread safety?
        res->is_success = true;
      }
    }
  }

  res->is_success = false;
}
}  // namespace standard
}  // namespace rm2_ecat
