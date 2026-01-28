#include "rm2_calibration_controllers/calibration_base.h"

namespace rm2_calibration_controllers
{
controller_interface::CallbackReturn CalibrationBase::on_init() {
  actuator_name_ = get_node()->declare_parameter<std::string>("actuator", "");
  joint_name_ = get_node()->declare_parameter<std::string>("joint", "");
  if (actuator_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No actuator given (namespace: %s)", get_node()->get_name());
    return CallbackReturn::ERROR;
  }
  if (joint_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint given (namespace: %s)", get_node()->get_name());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CalibrationBase::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  velocity_pid_.pid_ptr = std::make_shared<control_toolbox::PidROS>(get_node(), "velocity.pid", "~/" + joint_name_, false);
  velocity_pid_.pid_ptr->initialize_from_ros_parameters();
  velocity_search_ = get_node()->declare_parameter<double>("velocity_search", 0.0);
  if (velocity_search_ == 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Search velocity was not specified (namespace: %s)", get_node()->get_namespace());
    return CallbackReturn::ERROR;
  }
  is_calibrated_srv_ = get_node()->create_service<control_msgs::srv::QueryCalibrationState>(
      "is_calibrated", std::bind(&CalibrationBase::isCalibrated, this, std::placeholders::_1, std::placeholders::_2));
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CalibrationBase::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(actuator_name_ + "/act_extra/offset");
  config.names.push_back(actuator_name_ + "/act_extra/halted");
  config.names.push_back(actuator_name_ + "/act_extra/need_calibration");
  config.names.push_back(actuator_name_ + "/act_extra/calibrated");
  config.names.push_back(actuator_name_ + "/act_extra/calibration_reading");
  config.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  return config;

}

controller_interface::InterfaceConfiguration CalibrationBase::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(actuator_name_ + "/act_state/pos");
  config.names.push_back(actuator_name_ + "/act_state/vel");
  config.names.push_back(actuator_name_ + "/act_state/eff");
  config.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  return config;

}

controller_interface::CallbackReturn CalibrationBase::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  command_interface_index_map_ = buildInterfaceIndexMap(command_interfaces_);
  state_interface_index_map_ = buildInterfaceIndexMap(state_interfaces_);

  if (command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/calibrated")).get_optional<bool>().value()) {
    RCLCPP_INFO(get_node()->get_logger(), "Joint %s will be recalibrated, but was already calibrated at offset %f",
                 joint_name_.c_str(),
                 command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/offset")).get_optional<double>().value());
  }

  if (!command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/calibrated")).set_value(false)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set calibrated for actuator %s", actuator_name_.c_str());
    return CallbackReturn::ERROR;
  }
  state_ = INITIALIZED;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CalibrationBase::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  command_interface_index_map_.clear();
  state_interface_index_map_.clear();
  calibration_success_ = false;
  return CallbackReturn::SUCCESS;
}

bool CalibrationBase::isCalibrated(const std::shared_ptr<QueryCalibration::Request> /*req*/,
                                         std::shared_ptr<QueryCalibration::Response> resp) {
  RCLCPP_DEBUG(get_node()->get_logger(), "Is calibrated service %d", calibration_success_);
  resp->is_calibrated = calibration_success_;
  return true;
}
}  // namespace rm2_calibration_controllers
