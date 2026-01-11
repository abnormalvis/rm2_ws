//
// Created by qiayuan on 23-4-13.
//

#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <rm2_ecat_manager/RmEcatStandardSlaveManager.h>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rm2_ecat_msgs/msg/rm_ecat_standard_slave_readings.hpp>
#include <rm2_msgs/srv/enable_imu_trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sensor_msgs/msg/time_reference.hpp"
#include <any_node/ThreadedPublisher.hpp>

#include "rm2_ecat_ros/ClearSlaveManagerRosBase.h"
#include "rm2_msgs/srv/enable_imu_trigger.hpp"

namespace rm2_ecat {
namespace standard {
class RmStandardSlaveManagerRos : public cleardrive_ros::ClearSlaveManagerRosBase<rm2_ecat::standard::RmEcatStandardSlaveManager> {
 public:
  using ClearSlaveManagerRosBase<rm2_ecat::standard::RmEcatStandardSlaveManager>::ClearSlaveManagerRosBase;

  rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings getReadingsMsg();
  sensor_msgs::msg::JointState getJointStateMsg();
  std::vector<sensor_msgs::msg::Imu> getImuMsgs();

  void updateProcessReadings() override;
  void sendRos() override;

 protected:
  void startupRosInterface() override;
  void shutdownRosInterface() override;

  void imuTriggerCallback(const std::shared_ptr<rm2_msgs::srv::EnableImuTrigger::Request> req,
                        std::shared_ptr<rm2_msgs::srv::EnableImuTrigger::Response> res);

//   rclcpp::Subscription<typename MessageT> commandsSubscriber_;
  any_node::ThreadedPublisherPtr<rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings> readingsPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::msg::JointState> jointStatesPublisher_;
  std::vector<any_node::ThreadedPublisherPtr<sensor_msgs::msg::Imu>> imuPublishers_;
  std::vector<any_node::ThreadedPublisherPtr<rm2_msgs::msg::DbusData>> dbusDatasPublishers_;
  std::vector<any_node::ThreadedPublisherPtr<rm2_msgs::msg::GpioData>> gpioDatasPublishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr> imuTriggerTimePublishers_;
  rclcpp::Service<rm2_msgs::srv::EnableImuTrigger>::SharedPtr imusTriggerServer_;

  std::recursive_mutex readingsMsgMutex_;
  std::atomic<bool> readingsMsgUpdated_;
  rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings readingsMsg_;

  std::recursive_mutex jointStatesMsgMutex_;
  std::atomic<bool> jointStatesMsgUpdated_;
  sensor_msgs::msg::JointState jointStatesMsg_;

  std::recursive_mutex imuMsgsMutex_;
  std::atomic<bool> imuMsgsUpdated_;
  std::vector<sensor_msgs::msg::Imu> imuMsgs_;

  std::recursive_mutex dbusDatasMutex_;
  std::atomic<bool> dbusDatasUpdated_;
  std::vector<rm2_msgs::msg::DbusData> dbusDatas;

 std::recursive_mutex gpioDatasMutex_;
 std::atomic<bool> gpioDatasUpdated_;
 std::vector<rm2_msgs::msg::GpioData> gpioDatas;
};
}  // namespace standard
}  // namespace rm2_ecat
