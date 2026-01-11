//
// Created by kook on 12/25/23.
//

#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <rm2_ecat_manager/RmEcatMitManager.h>
#include <rm2_ecat_msgs/msg/rm_ecat_mit_slave_readings.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <any_node/ThreadedPublisher.hpp>

#include "rm2_ecat_ros/ClearSlaveManagerRosBase.h"

namespace rm2_ecat {
namespace mit {
class RmMitSlaveManagerRos : public cleardrive_ros::ClearSlaveManagerRosBase<rm2_ecat::mit::RmEcatMitManager> {
 public:
  using cleardrive_ros::ClearSlaveManagerRosBase<rm2_ecat::mit::RmEcatMitManager>::ClearSlaveManagerRosBase;

  rm2_ecat_msgs::msg::RmEcatMitSlaveReadings getReadingsMsg();
  sensor_msgs::msg::JointState getJointStateMsg();

  void updateProcessReadings() override;
  void sendRos() override;

 protected:
  void startupRosInterface() override;
  void shutdownRosInterface() override;

  // rclcpp::Subscription<typename MessageT> commandsSubscriber_;
  any_node::ThreadedPublisherPtr<rm2_ecat_msgs::msg::RmEcatMitSlaveReadings> readingsPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::msg::JointState> jointStatesPublisher_;
  std::vector<any_node::ThreadedPublisherPtr<rm2_msgs::msg::GpioData>> gpioDatasPublishers_;

  std::recursive_mutex readingsMsgMutex_;
  std::atomic<bool> readingsMsgUpdated_;
  rm2_ecat_msgs::msg::RmEcatMitSlaveReadings readingsMsg_;

  std::recursive_mutex jointStatesMsgMutex_;
  std::atomic<bool> jointStatesMsgUpdated_;
  sensor_msgs::msg::JointState jointStatesMsg_;


  std::recursive_mutex gpioDatasMutex_;
  std::atomic<bool> gpioDatasUpdated_;
  std::vector<rm2_msgs::msg::GpioData> gpioDatas;
};
}  // namespace mit
}  // namespace rm2_ecat
