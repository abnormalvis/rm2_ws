//
// Created by qiayuan on 23-4-16.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>
#include <joint_limits/joint_saturation_limiter.hpp>
#include <joint_limits/joint_soft_limiter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <joint_limits/joint_limits.hpp>
#include <hardware_interface/system_interface.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/simple_transmission_loader.hpp>

#include "rm2_ecat_ros/RmMitSlaveManagerRos.h"
#include "rm2_ecat_ros/RmStandardSlaveManagerRos.h"
#include "rm2_ecat_ros/types.h"
#include "rm2_msgs/msg/bus_state.hpp"
#include "rm2_msgs/msg/gpio_data.hpp"

namespace rm2_ecat {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;  

class RmEcatHardwareInterface final : public hardware_interface::SystemInterface {
 public:
  RmEcatHardwareInterface() = default;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override;
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool isRunning();

 protected:
  bool updateWorkerCb(const any_worker::WorkerEvent& /*event*/);
  bool publishWorkerCb(const any_worker::WorkerEvent& /*event*/);
  void handleSignal(int signum);

  std::shared_ptr<ecat_manager::EcatBusManager> busManager_;
  std::set<std::string> busNames_;
  std::map<std::string, bool> busIsOk_;
  int busDiagDecimationCount_ = 0;
  std::shared_ptr<rm2_ecat::standard::RmStandardSlaveManagerRos> rmStandardSlaveManager_;
  std::shared_ptr<rm2_ecat::mit::RmMitSlaveManagerRos> rmMitSlaveManager_;

  std::shared_ptr<any_worker::Worker> updateWorker_;
  std::shared_ptr<any_worker::Worker> publishWorker_;
  any_node::ThreadedPublisherPtr<rm2_msgs::msg::BusState> busStatesPublisher_;
  std::atomic<bool> busStatesMsgUpdated_{};
  any_node::ThreadedPublisherPtr<rm2_msgs::msg::GpioData> rmGpioOutputsPublisher_;
  std::atomic<bool> rmGpioOutputsMsgUpdated_{};
  any_node::ThreadedPublisherPtr<rm2_msgs::msg::GpioData> mitGpioOutputsPublisher_;
  std::atomic<bool> mitGpioOutputsMsgUpdated_{};

  bool loadUrdf(std::shared_ptr<rclcpp::Node> node);
  void setupActuators();
  bool setupTransmission();
  bool setupJointLimit();
  bool enforceLimit(const rclcpp::Duration &period);
  void setupImus();
  void setupGpios();

  std::string urdf_string_;
  std::shared_ptr<urdf::Model> urdf_model_;

  JointLimiters joint_limiters_;
  std::unique_ptr<transmission_interface::SimpleTransmissionLoader> transmission_loader_{};
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;
  std::unordered_map<std::string, TransmissionData> joint_transmission_interfaces_;
  std::unordered_map<std::string, TransmissionData> actuator_transmission_interfaces_;

  std::shared_ptr<rclcpp::Node> node_;

};

}  // namespace rm2_ecat
