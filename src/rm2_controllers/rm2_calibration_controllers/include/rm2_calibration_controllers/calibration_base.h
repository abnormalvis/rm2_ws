#pragma once

#include <hardware_interface/hardware_component_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <control_msgs/srv/query_calibration_state.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace rm2_calibration_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using QueryCalibration = control_msgs::srv::QueryCalibrationState;

class CalibrationBase : public controller_interface::ControllerInterface
{
public:
  CalibrationBase() = default;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  bool isCalibrated(const std::shared_ptr<QueryCalibration::Request> req,
                    std::shared_ptr<QueryCalibration::Response> resp);

  template<typename T>
  std::unordered_map<std::string, size_t> buildInterfaceIndexMap(T&& interfaces) {
    std::unordered_map<std::string, size_t> interface_index_map;
    interface_index_map.reserve(interfaces.size());
    for (size_t i = 0; i < interfaces.size(); ++i) {
      interface_index_map[interfaces[i].get_name()] = i;
    }
    return interface_index_map;
  }
  rclcpp::Service<QueryCalibration>::SharedPtr is_calibrated_srv_;
  enum State
  {
    INITIALIZED,
    CALIBRATED
  };
  struct PidType
  {
    std::shared_ptr<control_toolbox::PidROS> pid_ptr;
    double command{0.0};
  };
  int state_{};
  double velocity_search_{};
  bool calibration_success_ = false;
  std::string actuator_name_;
  std::string joint_name_;
  std::unordered_map<std::string, size_t> command_interface_index_map_;
  std::unordered_map<std::string, size_t> state_interface_index_map_;
  PidType velocity_pid_;
  PidType position_pid_;
};

}  // namespace rm2_calibration_controllers
