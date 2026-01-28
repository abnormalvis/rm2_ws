#include "rm2_calibration_controllers/mechanical_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace rm2_calibration_controllers 
{
controller_interface::CallbackReturn MechanicalCalibrationController::on_init() {
  if (CalibrationBase::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  is_center_ = get_node()->declare_parameter<bool>("center", false);
  is_return_ = get_node()->declare_parameter<bool>("return", false);
  velocity_threshold_ = get_node()->declare_parameter<double>("velocity.vel_threshold", -1.0);
  if (velocity_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Velocity threshold occur error or not specified with value: %lf (namespace: %s)", velocity_threshold_, get_node()->get_namespace());
    return CallbackReturn::ERROR;
  }

  if (is_return_) {
    position_pid_.pid_ptr = std::make_shared<control_toolbox::PidROS>(get_node(), "position_return.pid", "~/" + joint_name_, false);
    position_pid_.pid_ptr->initialize_from_ros_parameters();
    target_position_ = get_node()->declare_parameter<double>("position_return.target_position", 0.0);
    position_threshold_ = get_node()->declare_parameter<double>("position_return.pos_threshold", -1.0);
    if (position_threshold_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Position threshold occur error not specified with value: %lf (namespace: %s)", position_threshold_, get_node()->get_namespace());
      return CallbackReturn::ERROR;
    }
    calibration_success_ = false;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MechanicalCalibrationController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  switch (state_) {
    case INITIALIZED:
    {
      velocity_pid_.command = velocity_search_;
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)).get_optional<double>().value()) < velocity_threshold_ 
          && !command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/halted")).get_optional<bool>().value())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_pid_.command = 0.0;
        if (!is_center_)
        {
          if (!command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/offset")).set_value(
                            -state_interfaces_.at(state_interface_index_map_.at(actuator_name_ + "/act_state/pos")).get_optional<double>().value() +
                            command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/offset")).get_optional<double>().value())) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set offset for actuator %s", actuator_name_.c_str());
            return controller_interface::return_type::ERROR;
          }
          if (!command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/calibrated")).set_value(true)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set calibrated for actuator %s", actuator_name_.c_str());
            return controller_interface::return_type::ERROR;
          
          }
          RCLCPP_INFO(get_node()->get_logger(), "Joint %s calibrated", joint_name_.c_str());
          state_ = CALIBRATED;
          if (is_return_)
          {
            position_pid_.command = target_position_;
          }
          else
          {
            if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(0.0)) {
              RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
              return controller_interface::return_type::ERROR;
            }
            calibration_success_ = true;
          }
        }
        else
        {
          positive_position_ = state_interfaces_.at(state_interface_index_map_.at(actuator_name_ + "/act_state/pos")).get_optional<double>().value();
          countdown_ = 100;
          velocity_pid_.command = -velocity_search_;
          state_ = MOVING_NEGATIVE;
        }
      }
      
      if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(
          velocity_pid_.pid_ptr->compute_command(
            velocity_pid_.command - state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)).get_optional<double>().value(),
            period))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
        return controller_interface::return_type::ERROR;
      }
      break;
    }
    case MOVING_NEGATIVE:
    {
      if (std::abs(state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)).get_optional<double>().value()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_pid_.command = 0.0;
        negative_position_ = state_interfaces_.at(state_interface_index_map_.at(actuator_name_ + "/act_state/pos")).get_optional<double>().value();
        if (!command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/offset")).set_value(
                          -(positive_position_ + negative_position_) / 2 +
                                command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/offset")).get_optional<double>().value())) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set offset for actuator %s", actuator_name_.c_str());
            return controller_interface::return_type::ERROR;
          }
        if (!command_interfaces_.at(command_interface_index_map_.at(actuator_name_ + "/act_extra/calibrated")).set_value(true)) {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to set calibrated for actuator %s", actuator_name_.c_str());
          return controller_interface::return_type::ERROR;
          
        }
        RCLCPP_INFO(get_node()->get_logger(), "Joint %s calibrated", joint_name_.c_str());
        state_ = CALIBRATED;
        if (is_return_)
          position_pid_.command = target_position_;
        else
        {
          if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(0.0)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
            return controller_interface::return_type::ERROR;
          }
          calibration_success_ = true;
        }
      }

      if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(
          velocity_pid_.pid_ptr->compute_command(
            velocity_pid_.command - state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)).get_optional<double>().value(),
            period))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
        return controller_interface::return_type::ERROR;
      }
      break;
    }
    case CALIBRATED:
    {
      if (is_return_)
      {
        if ((std::abs(state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_POSITION)).get_optional<double>().value() - 
            target_position_)) < position_threshold_) {
          calibration_success_ = true;              
        }
        if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(
            position_pid_.pid_ptr->compute_command(
              position_pid_.command - state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_POSITION)).get_optional<double>().value(),
              period))) {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
          return controller_interface::return_type::ERROR;
        }
      }
      else
        if (!command_interfaces_.at(command_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT)).set_value(
            velocity_pid_.pid_ptr->compute_command(
              velocity_pid_.command - state_interfaces_.at(state_interface_index_map_.at(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)).get_optional<double>().value(),
              period))) {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for joint %s", joint_name_.c_str());
          return controller_interface::return_type::ERROR;
        }
      break;
    }
  }
  return controller_interface::return_type::OK;
}
} // namespace rm2_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm2_calibration_controllers::MechanicalCalibrationController, controller_interface::ControllerInterface)