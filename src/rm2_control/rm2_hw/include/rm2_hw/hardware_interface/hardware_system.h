//
// Created by ch on 2025/10/25.
//

#pragma once

#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <urdf/model.h>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_saturation_limiter.hpp"
#include "joint_limits/joint_soft_limiter.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rm2_hw/hardware_interface/can_bus.h"
#include "rm2_msgs/msg/actuator_state.hpp"

namespace rm2_hw
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RmHardwareSystem final : public hardware_interface::SystemInterface
{
public:
  RmHardwareSystem() = default;
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  int thread_priority_;
  bool is_actuator_specified_ = false;
  bool parseActCoeffs(const std::vector<std::string>& act_coeffs);
  bool parseActData(const std::vector<std::string>& act_datas);
  bool setupTransmission();
  void setCanBusThreadPriority(int thread_priority);

  std::unique_ptr<transmission_interface::SimpleTransmissionLoader> transmission_loader_{};
  void publishActuatorState(const rclcpp::Time& time);

  std::unordered_map<std::string, InterfaceData> joint_name2joint_interfaces_;
  std::unordered_map<std::string, InterfaceData> actuator_name2actuator_interfaces_;
  
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;
  std::vector<std::unique_ptr<CanBus>> can_buses_{};

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  // Actuator
  std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // // Imu
  // std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};
  // bool is_actuator_specified_ = false;

  std::shared_ptr<rclcpp::Node> node_, pub_node_;
  std::shared_ptr<rclcpp::Publisher<rm2_msgs::msg::ActuatorState>> actuator_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm2_msgs::msg::ActuatorState>> actuator_state_pub_rt_;
};
};  // namespace rm2_hw
