#include "rm2_gimbal_controllers/joint_controller.h"

#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace rm2_gimbal_controllers {
namespace {
double pickIntegralLimit(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                         const std::string &max_name,
                         const std::string &clamp_name) {
  const double max_value = node->get_parameter(max_name).as_double();
  const double clamp_value = node->get_parameter(clamp_name).as_double();
  if (std::abs(max_value) < 1e-12 && std::abs(clamp_value) > 1e-12) {
    return clamp_value;
  }
  return max_value;
}
} // namespace

bool JointController::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string &joint_prefix) {
  joint_name_ = node->get_parameter(joint_prefix + ".joint").as_string();
  vel_p_gain_ = node->get_parameter(joint_prefix + ".pid.p").as_double();
  vel_i_gain_ = node->get_parameter(joint_prefix + ".pid.i").as_double();
  vel_d_gain_ = node->get_parameter(joint_prefix + ".pid.d").as_double();
  vel_i_max_ = pickIntegralLimit(node, joint_prefix + ".pid.i_max",
                                 joint_prefix + ".pid.i_clamp_max");
  vel_i_min_ = pickIntegralLimit(node, joint_prefix + ".pid.i_min",
                                 joint_prefix + ".pid.i_clamp_min");

  pos_p_gain_ = node->get_parameter(joint_prefix + ".pid_pos.p").as_double();
  pos_i_gain_ = node->get_parameter(joint_prefix + ".pid_pos.i").as_double();
  pos_d_gain_ = node->get_parameter(joint_prefix + ".pid_pos.d").as_double();
  pos_i_max_ = pickIntegralLimit(node, joint_prefix + ".pid_pos.i_max",
                                 joint_prefix + ".pid_pos.i_clamp_max");
  pos_i_min_ = pickIntegralLimit(node, joint_prefix + ".pid_pos.i_min",
                                 joint_prefix + ".pid_pos.i_clamp_min");

  if (node->has_parameter(joint_prefix + ".limits.max_position")) {
    joint_limits_.has_position_limits = true;
    joint_limits_.max_position =
        node->get_parameter(joint_prefix + ".limits.max_position").as_double();
    joint_limits_.min_position =
        node->get_parameter(joint_prefix + ".limits.min_position").as_double();
  }

  if (node->has_parameter(joint_prefix + ".limits.max_velocity")) {
    joint_limits_.has_velocity_limits = true;
    joint_limits_.max_velocity =
        node->get_parameter(joint_prefix + ".limits.max_velocity").as_double();
  }

  if (node->has_parameter(joint_prefix + ".limits.max_effort")) {
    joint_limits_.has_effort_limits = true;
    joint_limits_.max_effort =
        node->get_parameter(joint_prefix + ".limits.max_effort").as_double();
  }

  parent_link_name_ =
      node->get_parameter(joint_prefix + ".parent_link").as_string();
  return true;
}

void JointController::assignStateInterfaces(
    std::vector<hardware_interface::LoanedStateInterface> &interfaces) {
  for (auto &interface : interfaces) {
    if (interface.get_prefix_name() == joint_name_) {
      if (interface.get_interface_name() == "position") {
        position_state_ = &interface;
      } else if (interface.get_interface_name() == "velocity") {
        velocity_state_ = &interface;
      }
    }
  }
}

void JointController::assignCommandInterface(
    std::vector<hardware_interface::LoanedCommandInterface> &interfaces) {
  for (auto &interface : interfaces) {
    if (interface.get_prefix_name() == joint_name_ &&
        interface.get_interface_name() == "effort") {
      effort_command_ = &interface;
      break;
    }
  }
}

void JointController::releaseInterfaces() {
  reset();
  position_state_ = nullptr;
  velocity_state_ = nullptr;
  effort_command_ = nullptr;
}

void JointController::reset() {
  position_error_ = 0.0;
  position_error_last_ = 0.0;
  velocity_error_ = 0.0;
  velocity_error_last_ = 0.0;
  pos_integral_error_ = 0.0;
  vel_integral_error_ = 0.0;
  outer_loop_velocity_reference_ = 0.0;
  command_.effort_cmd_ = 0.0;
}

void JointController::setCommand(double position_cmd, double velocity_cmd) {
  if (joint_limits_.has_position_limits) {
    position_cmd = std::clamp(position_cmd, joint_limits_.min_position,
                              joint_limits_.max_position);
  }

  if (joint_limits_.has_velocity_limits) {
    velocity_cmd = std::clamp(velocity_cmd, -joint_limits_.max_velocity,
                              joint_limits_.max_velocity);
  }

  command_.position_cmd_ = position_cmd;
  command_.velocity_cmd_ = velocity_cmd;
}

void JointController::update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration &period) {
  if (!position_state_ || !velocity_state_) {
    return;
  }

  const double dt = period.seconds();
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return;
  }

  auto pos_opt = position_state_->get_optional();
  if (!pos_opt) {
    return;
  }
  const double current_position = *pos_opt;
  position_error_ = angles::shortest_angular_distance(current_position,
                                                      command_.position_cmd_);

  auto vel_opt = velocity_state_->get_optional();
  if (!vel_opt) {
    return;
  }
  const double current_velocity = *vel_opt;

  pos_integral_error_ += position_error_ * dt;
  pos_integral_error_ = std::clamp(pos_integral_error_, pos_i_min_, pos_i_max_);
  const double pos_derivative = (position_error_ - position_error_last_) / dt;
  position_error_last_ = position_error_;

  outer_loop_velocity_reference_ =
      pos_p_gain_ * position_error_ + pos_i_gain_ * pos_integral_error_ +
      pos_d_gain_ * pos_derivative;
  double velocity_des = command_.velocity_cmd_ + outer_loop_velocity_reference_;
  if (joint_limits_.has_velocity_limits) {
    velocity_des = std::clamp(velocity_des, -joint_limits_.max_velocity,
                              joint_limits_.max_velocity);
  }

  velocity_error_ = velocity_des - current_velocity;
  vel_integral_error_ += velocity_error_ * dt;
  vel_integral_error_ = std::clamp(vel_integral_error_, vel_i_min_, vel_i_max_);
  const double vel_derivative = (velocity_error_ - velocity_error_last_) / dt;
  velocity_error_last_ = velocity_error_;

  command_.effort_cmd_ = vel_p_gain_ * velocity_error_ +
                         vel_i_gain_ * vel_integral_error_ +
                         vel_d_gain_ * vel_derivative;

  if (joint_limits_.has_effort_limits) {
    command_.effort_cmd_ = std::clamp(command_.effort_cmd_,
                                      -joint_limits_.max_effort,
                                      joint_limits_.max_effort);
  }

  if (effort_command_ != nullptr) {
    const auto ret = effort_command_->set_value(command_.effort_cmd_);
    if (!ret) {
      static rclcpp::Clock clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("rm2_gimbal_controllers"), clock, 2000,
          "Failed to set effort for joint %s: effort=%.3f, pos=%.3f, vel=%.3f",
          joint_name_.c_str(), command_.effort_cmd_,
          pos_opt.value_or(std::numeric_limits<double>::quiet_NaN()),
          vel_opt.value_or(std::numeric_limits<double>::quiet_NaN()));
    }
  }
}

double JointController::getPosition() const {
  if (!position_state_) {
    return 0.0;
  }
  const auto value = position_state_->get_optional();
  return value.value_or(0.0);
}

double JointController::getVelocity() const {
  if (!velocity_state_) {
    return 0.0;
  }
  const auto value = velocity_state_->get_optional();
  return value.value_or(0.0);
}

double JointController::getPositionError() const { return position_error_; }

double JointController::getInnerLoopVelocityError() const {
  return velocity_error_;
}

double JointController::getOuterLoopVelocityReference() const {
  return outer_loop_velocity_reference_;
}

double JointController::getCommand() const { return command_.effort_cmd_; }

double JointController::getPositionCommand() const {
  return command_.position_cmd_;
}

double JointController::getVelocityCommand() const {
  return command_.velocity_cmd_;
}

void JointController::setEffort(double effort) {
  command_.effort_cmd_ = effort;
  if (effort_command_ != nullptr) {
    (void)effort_command_->set_value(command_.effort_cmd_);
  }
}

} // namespace rm2_gimbal_controllers