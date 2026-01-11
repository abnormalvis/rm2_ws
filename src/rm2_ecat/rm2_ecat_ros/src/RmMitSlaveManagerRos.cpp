//
// Created by kook on 12/25/23.
//

#include "rm2_ecat_ros/RmMitSlaveManagerRos.h"
#include "rm2_ecat_ros/RosMsgConversions.h"

namespace rm2_ecat {
namespace mit {
rm2_ecat_msgs::msg::RmEcatMitSlaveReadings RmMitSlaveManagerRos::getReadingsMsg() {
  std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
  return readingsMsg_;
}

sensor_msgs::msg::JointState RmMitSlaveManagerRos::getJointStateMsg() {
  std::lock_guard<std::recursive_mutex> jointStatesMsgLock(jointStatesMsgMutex_);
  return jointStatesMsg_;
}

void RmMitSlaveManagerRos::updateProcessReadings() {
  ClearSlaveManagerRosBase<RmEcatMitManager>::updateProcessReadings();
  const auto& readings = getReadings<rm2_ecat::mit::Reading>();
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
    readingsMsg_ = createMitSlaveReadingsMsg(getNamesOfSlaves(), readings);
    if (readingsPublisher_) {
      readingsPublisher_->publish(readingsMsg_);
    }
    readingsMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(jointStatesMsgMutex_);
    jointStatesMsg_ = createMitJointStateMsg(readings);
    if (jointStatesPublisher_) {
      jointStatesPublisher_->publish(jointStatesMsg_);
    }
    jointStatesMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(gpioDatasMutex_);
    gpioDatas = createMitGpioDatas(readings);
    for (size_t i = 0; i < gpioDatasPublishers_.size(); ++i) {
      gpioDatasPublishers_[i]->publish(gpioDatas[i]);
    }
    gpioDatasUpdated_ = true;
  }
}

void RmMitSlaveManagerRos::sendRos() {
  if (readingsMsgUpdated_) {
    readingsMsgUpdated_ = false;
    readingsPublisher_->sendRos();
  }
  if (jointStatesMsgUpdated_) {
    jointStatesMsgUpdated_ = false;
    jointStatesPublisher_->sendRos();
  }
  if (gpioDatasUpdated_) {
    gpioDatasUpdated_ = false;
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->sendRos();
    }
  }
}

void RmMitSlaveManagerRos::startupRosInterface() {
  shutdownPublishWorkerRequested_ = false;
  readingsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm2_ecat_msgs::msg::RmEcatMitSlaveReadings>>(
      node_->create_publisher<rm2_ecat_msgs::msg::RmEcatMitSlaveReadings>("mit_readings", 10), 50, false);

  readingsMsgUpdated_ = false;

  jointStatesPublisher_ = std::make_shared<any_node::ThreadedPublisher<sensor_msgs::msg::JointState>>(
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10), 50, false);
  jointStatesMsgUpdated_ = false;

  const auto slaves = getSlaves();
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

void RmMitSlaveManagerRos::shutdownRosInterface() {
  if (!shutdownPublishWorkerRequested_) {
    jointStatesPublisher_->shutdown();
    readingsPublisher_->shutdown();
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->shutdown();
    }
  }
}
}  // namespace mit
}  // namespace rm2_ecat