#pragma once

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <limits>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

namespace rm2_gimbal_controllers {

struct JointLimits {
  double max_position = std::numeric_limits<double>::max();
  double min_position = -std::numeric_limits<double>::max();
  double max_velocity = std::numeric_limits<double>::max();
  double max_effort = std::numeric_limits<double>::max();
  bool has_position_limits = false;
  bool has_velocity_limits = false;
  bool has_effort_limits = false;
};

class JointController {
public:
  JointController() = default;

  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
            const std::string &joint_prefix);

  void assignCommandInterface(
      std::vector<hardware_interface::LoanedCommandInterface> &interfaces);

  void assignStateInterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &interfaces);

  void releaseInterfaces();
  void reset();

  void setCommand(double position_cmd, double velocity_cmd);
  void update(const rclcpp::Time &time, const rclcpp::Duration &period);

  double getPositionError() const;
  double getInnerLoopVelocityError() const;
  double getOuterLoopVelocityReference() const;
  double getPosition() const;
  double getVelocity() const;
  double getCommand() const;
  void setEffort(double effort);

  std::string joint_name_;
  std::string parent_link_name_;
  JointLimits joint_limits_;

private:
  double pos_p_gain_ = 0.0;
  double pos_i_gain_ = 0.0;
  double pos_d_gain_ = 0.0;
  double pos_i_max_ = 0.0;
  double pos_i_min_ = 0.0;

  double vel_p_gain_ = 0.0;
  double vel_i_gain_ = 0.0;
  double vel_d_gain_ = 0.0;
  double vel_i_max_ = 0.0;
  double vel_i_min_ = 0.0;

  double position_error_ = 0.0;
  double position_error_last_ = 0.0;
  double velocity_error_ = 0.0;
  double velocity_error_last_ = 0.0;
  double pos_integral_error_ = 0.0;
  double vel_integral_error_ = 0.0;
  double outer_loop_velocity_reference_ = 0.0;

  struct Command {
    double position_cmd_ = 0.0;
    double velocity_cmd_ = 0.0;
    double effort_cmd_ = 0.0;
  };

  Command command_{};
  hardware_interface::LoanedStateInterface *position_state_ = nullptr;
  hardware_interface::LoanedStateInterface *velocity_state_ = nullptr;
  hardware_interface::LoanedCommandInterface *effort_command_ = nullptr;
};

} // namespace rm2_gimbal_controllers