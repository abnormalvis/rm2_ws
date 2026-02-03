#include "rm2_gimbal_controllers/gimbal_base.h"
#include "rm2_gimbal_controllers/bullet_solver.h"
#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <optional>
#include <pal_statistics/pal_statistics.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm2_gimbal_controllers {
bool JointController::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string &joint_prefix) {
  // 获取关节名称
  joint_name_ = node->get_parameter(joint_prefix + ".joint").as_string();
  // 获取PID参数
  p_gain_ = node->get_parameter(joint_prefix + ".pid.p").as_double();
  i_gain_ = node->get_parameter(joint_prefix + ".pid.i").as_double();
  d_gain_ = node->get_parameter(joint_prefix + ".pid.d").as_double();
  i_max_ = node->get_parameter(joint_prefix + ".pid.i_max").as_double();
  i_min_ = node->get_parameter(joint_prefix + ".pid.i_min").as_double();

  // 获取关节限位
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

  // 获取父连杆名称（用于TF）
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
  position_state_ = nullptr;
  velocity_state_ = nullptr;
  effort_command_ = nullptr;
}

void JointController::setCommand(double position_cmd, double velocity_cmd) {
  // 根据配置的关节限位进行饱和处理
  if (joint_limits_.has_position_limits) {
    position_cmd = std::clamp(position_cmd, joint_limits_.min_position,
                              joint_limits_.max_position);
  }

  if (joint_limits_.has_velocity_limits) {
    // 若仅提供最大速度，则按对称范围 [-max_velocity, max_velocity] 限制
    velocity_cmd = std::clamp(velocity_cmd, -joint_limits_.max_velocity,
                              joint_limits_.max_velocity);
  }

  // 设置命令
  command_.position_cmd_ = position_cmd;
  command_.velocity_cmd_ = velocity_cmd;
}

void JointController::update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration &period) {
  // 计算位置误差
  if (!position_state_ || !velocity_state_) {
    return;
  }
  // get_value() 已经被弃用，改用 get_optional()
  auto pos_opt = position_state_->get_optional();
  if (!pos_opt) {
    return;
  }
  double current_position = *pos_opt;
  position_error_ = angles::shortest_angular_distance(current_position,
                                                      command_.position_cmd_);

  // 计算速度误差
  auto vel_opt = velocity_state_->get_optional();
  if (!vel_opt) {
    return;
  }
  double current_velocity = *vel_opt;
  velocity_error_ = command_.velocity_cmd_ - current_velocity;

  // 计算积分项
  integral_error_ += position_error_ * period.seconds();
  integral_error_ = std::clamp(integral_error_, i_min_, i_max_);

  // 计算微分项
  double derivative =
      (position_error_ - position_error_last_) / period.seconds();
  position_error_last_ = position_error_;

  // PID控制律
  command_.effort_cmd_ = p_gain_ * position_error_ + i_gain_ * integral_error_ +
                         d_gain_ * derivative;

  // 限制输出力矩
  if (joint_limits_.has_effort_limits) {
    command_.effort_cmd_ =
        std::clamp(command_.effort_cmd_, -joint_limits_.max_effort,
                   joint_limits_.max_effort);
  }

  // 设置命令
  if (effort_command_ != nullptr) {
    auto ret = effort_command_->set_value(command_.effort_cmd_);
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
  auto v = position_state_->get_optional();
  return v.value_or(0.0);
}

double JointController::getVelocity() const {
  if (!velocity_state_) {
    return 0.0;
  }
  auto v = velocity_state_->get_optional();
  return v.value_or(0.0);
}

double JointController::getCommand() const { return command_.effort_cmd_; }

void JointController::setEffort(double effort) {
  command_.effort_cmd_ = effort;
  if (effort_command_ != nullptr) {
    (void)effort_command_->set_value(command_.effort_cmd_);
  }
}

// ChassisVel 实现
ChassisVel::FilteredVelocity::FilteredVelocity(double cutoff_frequency)
    : cutoff_frequency_(cutoff_frequency) {}

void ChassisVel::FilteredVelocity::setCutoffFrequency(
    double cutoff_frequency) noexcept {
  cutoff_frequency_.store(cutoff_frequency);
}

void ChassisVel::FilteredVelocity::update(const double velocity[3],
                                          double period) {
  raw_value_[0] = velocity[0];
  raw_value_[1] = velocity[1];
  raw_value_[2] = velocity[2];

  // 对齐 ROS1(reference)：若周期过大，认为数据不连续，直接重置滤波状态
  if (period > 0.1) {
    filtered_value_ = raw_value_;
    return;
  }

  // 一阶低通：y[n] = y[n-1] + alpha * (x[n] - y[n-1])
  // 若 cutoff_frequency_ 使用 Hz，则离散化可写为：
  //   alpha = dt * (2*pi*fc) / (1 + dt * (2*pi*fc))
  const double cutoff_frequency = cutoff_frequency_.load();
  if (!(period > 0.0) || !(cutoff_frequency > 0.0)) {
    filtered_value_ = raw_value_;
    return;
  }

  constexpr double two_pi = 6.2831853071795864769;
  const double omega_c = two_pi * cutoff_frequency;
  double alpha = (period * omega_c) / (1.0 + period * omega_c);

  // 数值保护：避免极端参数导致 alpha 越界
  if (alpha < 0.0) {
    alpha = 0.0;
  } else if (alpha > 1.0) {
    alpha = 1.0;
  }

  filtered_value_[0] += alpha * (raw_value_[0] - filtered_value_[0]);
  filtered_value_[1] += alpha * (raw_value_[1] - filtered_value_[1]);
  filtered_value_[2] += alpha * (raw_value_[2] - filtered_value_[2]);
}

ChassisVel::ChassisVel(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
  node_ = std::move(node);
  cutoff_frequency_.store(
      node_->get_parameter("chassis_vel.cutoff_frequency").as_double());
  linear_ = std::make_shared<FilteredVelocity>(cutoff_frequency_.load());
  angular_ = std::make_shared<FilteredVelocity>(cutoff_frequency_.load());

  // 注册参数动态更新回调（rqt_reconfigure/ros2 param set 都会触发）
  param_cb_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&ChassisVel::paramCallback, this, std::placeholders::_1));
}

void ChassisVel::setCutoffFrequency(double cutoff_frequency) {
  cutoff_frequency_.store(cutoff_frequency);
  if (linear_) {
    linear_->setCutoffFrequency(cutoff_frequency);
  }
  if (angular_) {
    angular_->setCutoffFrequency(cutoff_frequency);
  }
}

rcl_interfaces::msg::SetParametersResult
ChassisVel::paramCallback(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    if (param.get_name() != "chassis_vel.cutoff_frequency") {
      continue;
    }

    double new_cutoff = 0.0;
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      new_cutoff = param.as_double();
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      new_cutoff = static_cast<double>(param.as_int());
    } else {
      result.successful = false;
      result.reason = "chassis_vel.cutoff_frequency must be a number";
      return result;
    }

    if (!(new_cutoff > 0.0)) {
      result.successful = false;
      result.reason = "chassis_vel.cutoff_frequency must be > 0";
      return result;
    }

    setCutoffFrequency(new_cutoff);
  }

  return result;
}

void ChassisVel::update(double linear_vel[3], double angular_vel[3],
                        double period) {
  linear_->update(linear_vel, period);
  angular_->update(angular_vel, period);
}

// GimbalControllerBase 实现
void GimbalControllerBase::setDes(const rclcpp::Time &time, const double yaw,
                                  const double pitch) {
  tf2::Quaternion q_odom2gimbal_des;
  q_odom2gimbal_des.setRPY(0.0, pitch, yaw);

  tf2::Quaternion q_odom2base;
  tf2::fromMsg(odom2base_.transform.rotation, q_odom2base);

  tf2::Quaternion q_base2gimbal_des = q_odom2base.inverse() * q_odom2gimbal_des;

  double roll_b2g = 0.0;
  double pitch_b2g = 0.0;
  double yaw_b2g = 0.0;
  tf2::Matrix3x3(q_base2gimbal_des).getRPY(roll_b2g, pitch_b2g, yaw_b2g);

  (void)setDesIntoLimit(yaw_b2g, yaw_controller_.joint_limits_);
  (void)setDesIntoLimit(pitch_b2g, pitch_controller_.joint_limits_);

  tf2::Quaternion q_base2gimbal_limited;
  q_base2gimbal_limited.setRPY(roll_b2g, pitch_b2g, yaw_b2g);

  // 写回 TF（odom->gimbal_des）
  odom2gimbal_des_.transform.rotation =
      tf2::toMsg(q_odom2base * q_base2gimbal_limited);
  odom2gimbal_des_.header.stamp = time;
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(odom2gimbal_des_);
  }

  // 更新关节 position 指令（base2gimbal 的 yaw/pitch 就是两关节的目标角）
  yaw_controller_.setCommand(yaw_b2g, yaw_vel_des_);
  pitch_controller_.setCommand(pitch_b2g, pitch_vel_des_);
}

bool GimbalControllerBase::setDesIntoLimit(double &des,
                                           const JointLimits &joint_limits) {
  if (!joint_limits.has_position_limits) {
    return true;
  }

  const double upper_limit = joint_limits.max_position;
  const double lower_limit = joint_limits.min_position;

  if ((des <= upper_limit && des >= lower_limit) ||
      (angles::two_pi_complement(des) <= upper_limit &&
       angles::two_pi_complement(des) >= lower_limit)) {
    return true;
  }

  des = (std::abs(angles::shortest_angular_distance(des, upper_limit)) <
         std::abs(angles::shortest_angular_distance(des, lower_limit)))
            ? upper_limit
            : lower_limit;
  return false;
}

void GimbalControllerBase::movejoint(const rclcpp::Time &time,
                                     const rclcpp::Duration &period) {
  yaw_controller_.update(time, period);
  pitch_controller_.update(time, period);

  double angular_vel_y = 0.0;
  double angular_vel_z = 0.0;
  if (has_imu_ && imu_node_.imu_angular_velocity_y_ &&
      imu_node_.imu_angular_velocity_z_) {
    angular_vel_y =
        imu_node_.imu_angular_velocity_y_->get_optional().value_or(0.0);
    angular_vel_z =
        imu_node_.imu_angular_velocity_z_->get_optional().value_or(0.0);
  } else {
    angular_vel_y = pitch_controller_.getVelocity();
    angular_vel_z = yaw_controller_.getVelocity();
  }

  const double chassis_ang_z = chassis_vel_ ? chassis_vel_->angular_->z() : 0.0;
  const double comp = updateCompensation(chassis_ang_z);

  const double pitch_ff = feedForwardPitch(time);

  const double pitch_extra = config_.pitch_k_v * pitch_vel_des_ +
                             pitch_controller_.getVelocity() - angular_vel_y +
                             pitch_ff;
  const double yaw_extra = -comp * chassis_ang_z +
                           config_.yaw_k_v * yaw_vel_des_ +
                           yaw_controller_.getVelocity() - angular_vel_z;

  pitch_controller_.setEffort(pitch_controller_.getCommand() + pitch_extra);
  yaw_controller_.setEffort(yaw_controller_.getCommand() + yaw_extra);
}

void GimbalControllerBase::feedforwardCompensation(
    const rclcpp::Time & /*time*/) {
  if (!enable_gravity_compensation_)
    return;
  // 重力补偿实现
}

void GimbalControllerBase::updateChassisVelocity() {
  if (!chassis_vel_)
    return;

  if (!has_odom2base_last_) {
    odom2base_last_ = odom2base_;
    has_odom2base_last_ = true;
    return;
  }

  const double tf_period = (rclcpp::Time(odom2base_.header.stamp) -
                            rclcpp::Time(odom2base_last_.header.stamp))
                               .seconds();
  if (!(tf_period > 0.0) || !std::isfinite(tf_period)) {
    odom2base_last_ = odom2base_;
    return;
  }

  const double linear_x = (odom2base_.transform.translation.x -
                           odom2base_last_.transform.translation.x) /
                          tf_period;
  const double linear_y = (odom2base_.transform.translation.y -
                           odom2base_last_.transform.translation.y) /
                          tf_period;
  const double linear_z = (odom2base_.transform.translation.z -
                           odom2base_last_.transform.translation.z) /
                          tf_period;

  double last_r = 0.0, last_p = 0.0, last_y = 0.0;
  double r = 0.0, p = 0.0, y = 0.0;
  {
    tf2::Quaternion q;
    tf2::fromMsg(odom2base_.transform.rotation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);
  }
  {
    tf2::Quaternion q;
    tf2::fromMsg(odom2base_last_.transform.rotation, q);
    tf2::Matrix3x3(q).getRPY(last_r, last_p, last_y);
  }

  const double angular_x =
      angles::shortest_angular_distance(last_r, r) / tf_period;
  const double angular_y =
      angles::shortest_angular_distance(last_p, p) / tf_period;
  const double angular_z =
      angles::shortest_angular_distance(last_y, y) / tf_period;

  double linear_vel[3]{linear_x, linear_y, linear_z};
  double angular_vel[3]{angular_x, angular_y, angular_z};
  chassis_vel_->update(linear_vel, angular_vel, tf_period);
  odom2base_last_ = odom2base_;
}

double GimbalControllerBase::updateCompensation(double chassis_vel_angular_z) {
  chassis_compensation_ =
      config_.chassis_compensation_a *
          std::sin(config_.chassis_compensation_b * chassis_vel_angular_z +
                   config_.chassis_compensation_c) +
      config_.chassis_compensation_d;
  return chassis_compensation_;
}

double GimbalControllerBase::feedForwardPitch(const rclcpp::Time &time) {
  if (!enable_gravity_compensation_) {
    return 0.0;
  }

  try {
    // reference.c: gravity 向量在 base_link，下变换到 pitch_link
    geometry_msgs::msg::Vector3Stamped gravity_base;
    gravity_base.header.stamp = time;
    gravity_base.header.frame_id = "base_link";
    gravity_base.vector.x = 0.0;
    gravity_base.vector.y = 0.0;
    gravity_base.vector.z = -gravity_;

    const auto tf = tf_buffer_->lookupTransform(
        odom2pitch_.child_frame_id, "base_link", tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped gravity_pitch;
    tf2::doTransform(gravity_base, gravity_pitch, tf);

    // mass_origin = (x,0,z)，计算 -mass_origin x gravity 的 y 分量
    const double mx = mass_position_.x;
    const double mz = mass_position_.z;

    const double gx = gravity_pitch.vector.x;
    // const double gy = gravity_pitch.vector.y;  // Unused
    const double gz = gravity_pitch.vector.z;

    const double cy = (mz * gx) - (mx * gz);
    return -cy;
  } catch (const tf2::TransformException &) {
    return 0.0;
  }
}

void GimbalControllerBase::commandCallback(
    rm2_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void GimbalControllerBase::trackCallback(
    rm2_msgs::msg::TrackData::ConstSharedPtr msg) {
  if (msg->id == 0) {
    return;
  }
  track_rt_buffer_.writeFromNonRT(*msg);
}

void GimbalControllerBase::rate(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to RATE mode");

    if (start_) {
      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      tf2::Quaternion q;
      tf2::fromMsg(odom2pitch_.transform.rotation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw_des_ = yaw;
      pitch_des_ = -pitch;
      start_ = false;
      yaw_vel_des_ = 0.0;
      pitch_vel_des_ = 0.0;
      setDes(time, yaw_des_, pitch_des_);
      return;
    }
  }

  yaw_vel_des_ = cmd_gimbal_.rate_yaw;
  pitch_vel_des_ = cmd_gimbal_.rate_pitch;

  yaw_des_ += yaw_vel_des_ * period.seconds();
  pitch_des_ += pitch_vel_des_ * period.seconds();

  setDes(time, yaw_des_, pitch_des_);
}

void GimbalControllerBase::track(const rclcpp::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to TRACK mode");
  }

  // 当前姿态（用于误差发布）
  double roll_real = 0.0, pitch_real = 0.0, yaw_real = 0.0;
  {
    tf2::Quaternion q_real;
    tf2::fromMsg(odom2pitch_.transform.rotation, q_real);
    tf2::Matrix3x3(q_real).getRPY(roll_real, pitch_real, yaw_real);
  }
  const double yaw_compute = yaw_real;
  const double pitch_compute = -pitch_real;

  geometry_msgs::msg::Point target_pos = data_track_.position;
  geometry_msgs::msg::Vector3 target_vel{};
  if (data_track_.id != 12) {
    target_vel = data_track_.velocity;
  }

  // 目标坐标系变换到 odom（若提供了 frame_id）
  try {
    if (!data_track_.header.frame_id.empty()) {
      const auto tf = tf_buffer_->lookupTransform(
          "odom", data_track_.header.frame_id,
          tf2::TimePoint(std::chrono::nanoseconds(
              rclcpp::Time(data_track_.header.stamp).nanoseconds())));
      tf2::doTransform(target_pos, target_pos, tf);
      tf2::doTransform(target_vel, target_vel, tf);
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "Track TF failed: %s", ex.what());
  }

  const double dt = (time - rclcpp::Time(data_track_.header.stamp)).seconds();

  // 时间补偿（对齐 reference.c）
  target_pos.x += target_vel.x * dt - odom2pitch_.transform.translation.x;
  target_pos.y += target_vel.y * dt - odom2pitch_.transform.translation.y;
  target_pos.z += target_vel.z * dt - odom2pitch_.transform.translation.z;

  // 底盘线速度补偿
  if (chassis_vel_) {
    target_vel.x -= chassis_vel_->linear_->x();
    target_vel.y -= chassis_vel_->linear_->y();
    target_vel.z -= chassis_vel_->linear_->z();
  }

  bool solve_success = false;
  if (bullet_solver_) {
    solve_success =
        bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed);
  }

  // publish error + bullet model
  if (publish_rate_ > 0.0 &&
      (last_publish_time_ +
       rclcpp::Duration::from_seconds(1.0 / publish_rate_)) < time) {
    if (error_pub_) {
      rm2_msgs::msg::GimbalDesError msg;
      msg.stamp = time;
      const double err = (bullet_solver_)
                             ? bullet_solver_->getGimbalError(
                                   target_pos, target_vel, yaw_compute,
                                   pitch_compute, cmd_gimbal_.bullet_speed)
                             : 0.0;
      msg.error = solve_success ? err : 1.0;
      error_pub_->publish(msg);
    }
    if (bullet_solver_) {
      bullet_solver_->bulletModelPub(odom2pitch_, time);
    }
    last_publish_time_ = time;
  }

  if (solve_success) {
    yaw_vel_des_ = 0.0;
    pitch_vel_des_ = 0.0;
    yaw_des_ = bullet_solver_->getYaw();
    pitch_des_ = -bullet_solver_->getPitch();
    setDes(time, yaw_des_, pitch_des_);
  } else {
    odom2gimbal_des_.header.stamp = time;
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(odom2gimbal_des_);
    }
  }
}

void GimbalControllerBase::direct(const rclcpp::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to DIRECT mode");
  }

  geometry_msgs::msg::Point aim_point_odom = cmd_gimbal_.target_pos.point;
  try {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty()) {
      const auto tf = tf_buffer_->lookupTransform(
          "odom", cmd_gimbal_.target_pos.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(aim_point_odom, aim_point_odom, tf);
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "Direct TF failed: %s", ex.what());
  }

  const double dx = aim_point_odom.x - odom2pitch_.transform.translation.x;
  const double dy = aim_point_odom.y - odom2pitch_.transform.translation.y;
  const double dz = aim_point_odom.z - odom2pitch_.transform.translation.z;

  yaw_vel_des_ = 0.0;
  pitch_vel_des_ = 0.0;

  yaw_des_ = std::atan2(dy, dx);
  const double horizontal_dist = std::sqrt(dx * dx + dy * dy);
  pitch_des_ = -std::atan2(dz, horizontal_dist);

  setDes(time, yaw_des_, pitch_des_);
}

void GimbalControllerBase::traj(const rclcpp::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to TRAJ mode");
  }

  double roll = 0.0;
  double pitch = cmd_gimbal_.traj_pitch;
  double yaw = cmd_gimbal_.traj_yaw;

  try {
    if (!cmd_gimbal_.traj_frame_id.empty()) {
      const auto odom2traj = tf_buffer_->lookupTransform(
          "odom", cmd_gimbal_.traj_frame_id, tf2::TimePointZero);
      tf2::Quaternion q;
      tf2::fromMsg(odom2traj.transform.rotation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      pitch += cmd_gimbal_.traj_pitch;
      yaw += cmd_gimbal_.traj_yaw;
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "Traj TF failed: %s", ex.what());
  }

  yaw_vel_des_ = 0.0;
  pitch_vel_des_ = 0.0;
  yaw_des_ = yaw;
  pitch_des_ = pitch;
  setDes(time, yaw_des_, pitch_des_);
}

void GimbalControllerBase::assignIMUinterfaces(
    std::vector<hardware_interface::LoanedStateInterface> &interfaces) {
  for (auto &interface : interfaces) {
    if (interface.get_prefix_name() == "imu") {
      if (interface.get_interface_name() == "angular_velocity.x") {
        imu_node_.imu_angular_velocity_x_ = &interface;
      } else if (interface.get_interface_name() == "angular_velocity.y") {
        imu_node_.imu_angular_velocity_y_ = &interface;
      } else if (interface.get_interface_name() == "angular_velocity.z") {
        imu_node_.imu_angular_velocity_z_ = &interface;
      } else if (interface.get_interface_name() == "orientation.x") {
        imu_node_.imu_orientation_x_ = &interface;
      } else if (interface.get_interface_name() == "orientation.y") {
        imu_node_.imu_orientation_y_ = &interface;
      } else if (interface.get_interface_name() == "orientation.z") {
        imu_node_.imu_orientation_z_ = &interface;
      } else if (interface.get_interface_name() == "orientation.w") {
        imu_node_.imu_orientation_w_ = &interface;
      }
    }
  }
}

controller_interface::CallbackReturn GimbalControllerBase::on_init() {
  try {
    auto_declare<std::string>("yaw.joint", "");
    auto_declare<double>("yaw.pid.p", 0.0);
    auto_declare<double>("yaw.pid.i", 0.0);
    auto_declare<double>("yaw.pid.d", 0.0);
    auto_declare<double>("yaw.pid.i_max", 0.0);
    auto_declare<double>("yaw.pid.i_min", 0.0);
    auto_declare<double>("yaw.limits.max_position", 0.0);
    auto_declare<double>("yaw.limits.min_position", 0.0);
    auto_declare<double>("yaw.limits.max_velocity", 0.0);
    auto_declare<double>("yaw.limits.max_effort", 0.0);
    auto_declare<std::string>("yaw.parent_link", "");

    auto_declare<std::string>("pitch.joint", "");
    auto_declare<double>("pitch.pid.p", 0.0);
    auto_declare<double>("pitch.pid.i", 0.0);
    auto_declare<double>("pitch.pid.d", 0.0);
    auto_declare<double>("pitch.pid.i_max", 0.0);
    auto_declare<double>("pitch.pid.i_min", 0.0);
    auto_declare<double>("pitch.limits.max_position", 0.0);
    auto_declare<double>("pitch.limits.min_position", 0.0);
    auto_declare<double>("pitch.limits.max_velocity", 0.0);
    auto_declare<double>("pitch.limits.max_effort", 0.0);
    auto_declare<std::string>("pitch.parent_link", "");

    auto_declare<bool>("feedforward.enable", false);
    auto_declare<double>("feedforward.mass_origin.x", 0.0);
    auto_declare<double>("feedforward.mass_origin.z", 0.0);
    auto_declare<double>("feedforward.gravity", 9.8);

    auto_declare<double>("yaw.resistance_compensation.resistance", 0.0);
    auto_declare<double>("yaw.resistance_compensation.error_tolerance", 0.0);
    auto_declare<double>("yaw.k_chassis_vel", 0.0);

    auto_declare<double>("publish_rate", 100.0);
    auto_declare<std::string>("imu_name", "");
    auto_declare<std::string>("imu_frame_id", "");

    auto_declare<double>("chassis_vel.cutoff_frequency", 20.0);
    auto_declare<double>("bullet_solver.resistance_coff", 0.0);
    auto_declare<double>("bullet_solver.resistance_coff_qd_10", 0.0);
    auto_declare<double>("bullet_solver.resistance_coff_qd_15", 0.0);
    auto_declare<double>("bullet_solver.resistance_coff_qd_16", 0.0);
    auto_declare<double>("bullet_solver.resistance_coff_qd_18", 0.0);
    auto_declare<double>("bullet_solver.resistance_coff_qd_30", 0.0);
    auto_declare<double>("bullet_solver.g", 9.8);
    auto_declare<int>("bullet_solver.max_iter", 20);

    auto_declare<double>("yaw_k_v", 0.0);
    auto_declare<double>("pitch_k_v", 0.0);
    auto_declare<double>("chassis_compensation.a", 0.0);
    auto_declare<double>("chassis_compensation.b", 0.0);
    auto_declare<double>("chassis_compensation.c", 0.0);
    auto_declare<double>("chassis_compensation.d", 0.0);
    auto_declare<std::vector<double>>("accel_pitch", std::vector<double>{});
    auto_declare<std::vector<double>>("accel_yaw", std::vector<double>{});

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalControllerBase::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::string yaw_joint = get_node()->get_parameter("yaw.joint").as_string();
  std::string pitch_joint =
      get_node()->get_parameter("pitch.joint").as_string();

  config.names.push_back(yaw_joint + "/effort");
  config.names.push_back(pitch_joint + "/effort");

  return config;
}

controller_interface::InterfaceConfiguration
GimbalControllerBase::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::string yaw_joint = get_node()->get_parameter("yaw.joint").as_string();
  std::string pitch_joint =
      get_node()->get_parameter("pitch.joint").as_string();

  config.names.push_back(yaw_joint + "/position");
  config.names.push_back(yaw_joint + "/velocity");
  config.names.push_back(pitch_joint + "/position");
  config.names.push_back(pitch_joint + "/velocity");

  std::string imu_name = get_node()->get_parameter("imu_name").as_string();
  if (!imu_name.empty()) {
    config.names.push_back(imu_name + "/angular_velocity.x");
    config.names.push_back(imu_name + "/angular_velocity.y");
    config.names.push_back(imu_name + "/angular_velocity.z");
    config.names.push_back(imu_name + "/orientation.x");
    config.names.push_back(imu_name + "/orientation.y");
    config.names.push_back(imu_name + "/orientation.z");
    config.names.push_back(imu_name + "/orientation.w");
  }

  return config;
}

controller_interface::CallbackReturn GimbalControllerBase::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  bool enable_feedforward =
      get_node()->get_parameter("feedforward.enable").as_bool();

  if (enable_feedforward) {
    enable_gravity_compensation_ = true;
    mass_position_.x =
        get_node()->get_parameter("feedforward.mass_origin.x").as_double();
    mass_position_.z =
        get_node()->get_parameter("feedforward.mass_origin.z").as_double();
    gravity_ = get_node()->get_parameter("feedforward.gravity").as_double();
  } else {
    enable_gravity_compensation_ = false;
  }

  yaw_compensation_ =
      get_node()
          ->get_parameter("yaw.resistance_compensation.resistance")
          .as_double();
  yaw_error_tolerance_ =
      get_node()
          ->get_parameter("yaw.resistance_compensation.error_tolerance")
          .as_double();
  k_chassis_vel_ = get_node()->get_parameter("yaw.k_chassis_vel").as_double();

  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

  has_imu_ = !get_node()->get_parameter("imu_name").as_string().empty();

  if (!has_imu_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "IMU not configured, some features may be limited");
  }

  chassis_vel_ = std::make_shared<ChassisVel>(get_node());
  bullet_solver_ = std::make_shared<BulletSolver>(get_node());

  // 控制器参数动态更新回调
  if (!param_cb_handle_) {
    param_cb_handle_ = get_node()->add_on_set_parameters_callback(std::bind(
        &GimbalControllerBase::paramCallback, this, std::placeholders::_1));
  }

  if (!yaw_controller_.init(get_node(), "yaw") ||
      !pitch_controller_.init(get_node(), "pitch")) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to initialize joint controllers");
    return controller_interface::CallbackReturn::ERROR;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

  cmd_gimbal_sub_ = get_node()->create_subscription<rm2_msgs::msg::GimbalCmd>(
      "~/command", rclcpp::QoS(10),
      std::bind(&GimbalControllerBase::commandCallback, this,
                std::placeholders::_1));

  data_track_sub_ = get_node()->create_subscription<rm2_msgs::msg::TrackData>(
      "/track", rclcpp::QoS(10),
      std::bind(&GimbalControllerBase::trackCallback, this,
                std::placeholders::_1));

  error_pub_ = get_node()->create_publisher<rm2_msgs::msg::GimbalDesError>(
      "~/error", rclcpp::QoS(10));

  gimbal_frame_id_ = "gimbal_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.0;

  odom2gimbal_traject_des_.header.frame_id = "odom";
  odom2gimbal_traject_des_.child_frame_id = "gimbal_traject_des";
  odom2gimbal_traject_des_.transform.rotation.w = 1.0;

  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = "pitch_link";

  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = "base_link";

  cmd_gimbal_.mode = RATE;
  cmd_gimbal_.rate_yaw = 0.0;
  cmd_gimbal_.rate_pitch = 0.0;
  cmd_gimbal_.bullet_speed = 15.0;
  cmd_rt_buffer_.writeFromNonRT(cmd_gimbal_);

  data_track_.id = 0;
  track_rt_buffer_.writeFromNonRT(data_track_);

  // 初始化动态配置（对齐 reference.c 的 config_rt_buffer_）
  config_.yaw_k_v = get_node()->get_parameter("yaw_k_v").as_double();
  config_.pitch_k_v = get_node()->get_parameter("pitch_k_v").as_double();
  config_.chassis_compensation_a =
      get_node()->get_parameter("chassis_compensation.a").as_double();
  config_.chassis_compensation_b =
      get_node()->get_parameter("chassis_compensation.b").as_double();
  config_.chassis_compensation_c =
      get_node()->get_parameter("chassis_compensation.c").as_double();
  config_.chassis_compensation_d =
      get_node()->get_parameter("chassis_compensation.d").as_double();
  config_.accel_pitch =
      get_node()->get_parameter("accel_pitch").as_double_array();
  config_.accel_yaw = get_node()->get_parameter("accel_yaw").as_double_array();
  config_rt_buffer_.writeFromNonRT(config_);

  yaw_des_ = 0.0;
  pitch_des_ = 0.0;
  start_ = true;
  has_odom2base_last_ = false;

  RCLCPP_INFO(get_node()->get_logger(),
              "Gimbal controller configured successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult GimbalControllerBase::paramCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  GimbalConfig cfg = config_;
  bool cfg_changed = false;

  for (const auto &param : parameters) {
    const auto &name = param.get_name();

    if (name == "publish_rate") {
      double v = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        v = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        v = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "publish_rate must be a number";
        return result;
      }
      if (!(v > 0.0)) {
        result.successful = false;
        result.reason = "publish_rate must be > 0";
        return result;
      }
      publish_rate_ = v;
    } else if (name == "yaw_k_v") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "yaw_k_v must be a number";
        return result;
      }
      cfg.yaw_k_v =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
      cfg_changed = true;
    } else if (name == "pitch_k_v") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "pitch_k_v must be a number";
        return result;
      }
      cfg.pitch_k_v =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
      cfg_changed = true;
    } else if (name == "chassis_compensation.a" ||
               name == "chassis_compensation.b" ||
               name == "chassis_compensation.c" ||
               name == "chassis_compensation.d") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "chassis_compensation.* must be a number";
        return result;
      }
      const double v =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
      if (name == "chassis_compensation.a") {
        cfg.chassis_compensation_a = v;
      } else if (name == "chassis_compensation.b") {
        cfg.chassis_compensation_b = v;
      } else if (name == "chassis_compensation.c") {
        cfg.chassis_compensation_c = v;
      } else {
        cfg.chassis_compensation_d = v;
      }
      cfg_changed = true;
    } else if (name == "accel_pitch") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        result.successful = false;
        result.reason = "accel_pitch must be double array";
        return result;
      }
      cfg.accel_pitch = param.as_double_array();
      cfg_changed = true;
    } else if (name == "accel_yaw") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        result.successful = false;
        result.reason = "accel_yaw must be double array";
        return result;
      }
      cfg.accel_yaw = param.as_double_array();
      cfg_changed = true;
    } else if (name == "yaw.k_chassis_vel") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "yaw.k_chassis_vel must be a number";
        return result;
      }
      k_chassis_vel_ =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
    } else if (name == "yaw.resistance_compensation.resistance") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason =
            "yaw.resistance_compensation.resistance must be a number";
        return result;
      }
      yaw_compensation_ =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
    } else if (name == "yaw.resistance_compensation.error_tolerance") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason =
            "yaw.resistance_compensation.error_tolerance must be a number";
        return result;
      }
      yaw_error_tolerance_ =
          (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
              ? param.as_double()
              : static_cast<double>(param.as_int());
    }
  }

  if (cfg_changed) {
    config_ = cfg;
    config_rt_buffer_.writeFromNonRT(cfg);
  }

  return result;
}

controller_interface::CallbackReturn GimbalControllerBase::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  yaw_controller_.assignCommandInterface(command_interfaces_);
  pitch_controller_.assignCommandInterface(command_interfaces_);

  yaw_controller_.assignStateInterfaces(state_interfaces_);
  pitch_controller_.assignStateInterfaces(state_interfaces_);

  if (has_imu_) {
    assignIMUinterfaces(state_interfaces_);
  }

  state_ = RATE;
  state_changed_ = true;

  last_publish_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "Gimbal controller activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalControllerBase::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  yaw_controller_.releaseInterfaces();
  pitch_controller_.releaseInterfaces();

  RCLCPP_INFO(get_node()->get_logger(), "Gimbal controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GimbalControllerBase::update(const rclcpp::Time &time,
                             const rclcpp::Duration &period) {
  // 从实时缓冲区读取最新的云台命令
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  // 从非RT缓冲区读取跟踪目标数据
  data_track_ = *track_rt_buffer_.readFromNonRT();
  // 动态配置（RT-safe）
  config_ = *config_rt_buffer_.readFromRT();

  try {
    odom2pitch_ = tf_buffer_->lookupTransform(
        "odom", odom2pitch_.child_frame_id, tf2::TimePointZero);
    odom2base_ = tf_buffer_->lookupTransform("odom", odom2base_.child_frame_id,
                                             tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "Could not get transform: %s", ex.what());
    return controller_interface::return_type::OK;
  }

  // 更新带盘速度
  updateChassisVelocity();

  // 更新控制状态
  if (state_ != static_cast<ControlMode>(cmd_gimbal_.mode)) {
    state_ = static_cast<ControlMode>(cmd_gimbal_.mode);
    state_changed_ = true;
  }

  // 状态机执行
  switch (state_) {
  case RATE:
    rate(time, period); // 根据角速度控制云台旋转
    break;
  case TRACK:
    track(time); // 跟踪运动目标
    break;
  case DIRECT:
    direct(time); // 直接指向空间某点
    break;
  case TRAJ:
    traj(time);
    break;
  default:
    RCLCPP_WARN(get_node()->get_logger(), "Unknown control mode: %d", state_);
    break;
  }

  // 移动关节
  movejoint(time, period);

  return controller_interface::return_type::OK;
}

} // namespace rm2_gimbal_controllers
