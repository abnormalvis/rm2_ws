#include "rm2_gimbal_controllers/gimbal_base.h"
#include "rm2_gimbal_controllers/bullet_solver.h"
#include <algorithm>
#include <angles/angles.h>
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

void ChassisVel::FilteredVelocity::update(const double velocity[3],
                                         double period) {
  raw_value_[0] = velocity[0];
  raw_value_[1] = velocity[1];
  raw_value_[2] = velocity[2];

  // 一阶低通：y[n] = y[n-1] + alpha * (x[n] - y[n-1])
  // 若 cutoff_frequency_ 使用 Hz，则离散化可写为：
  //   alpha = dt * (2*pi*fc) / (1 + dt * (2*pi*fc))
  if (!(period > 0.0) || !(cutoff_frequency_ > 0.0)) {
    filtered_value_ = raw_value_;
    return;
  }

  constexpr double two_pi = 6.2831853071795864769;
  const double omega_c = two_pi * cutoff_frequency_;
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
  cutoff_frequency_ =
      node->get_parameter("chassis_vel.cutoff_frequency").as_double();
  linear_ = std::make_shared<FilteredVelocity>(cutoff_frequency_);
  angular_ = std::make_shared<FilteredVelocity>(cutoff_frequency_);
}

void ChassisVel::update(double linear_vel[3], double angular_vel[3],
                        double period) {
  linear_->update(linear_vel, period);
  angular_->update(angular_vel, period);
}


// GimbalController 实现
void GimbalController::setDes(const rclcpp::Time &time, const double yaw,
                              const double pitch) {
  tf2::Quaternion q_des;
  q_des.setRPY(0.0, pitch, yaw);
  odom2gimbal_des_.transform.rotation = tf2::toMsg(q_des);
  odom2gimbal_des_.header.stamp = time;
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(odom2gimbal_des_);
  }
}

void GimbalController::setDesIntoLimit(double &des_yaw, double &des_pitch,
                                       double base2gimbal_current_des,
                                       const JointLimits &joint_limits) {
  if (!joint_limits.has_position_limits) {
    return;
  }

  double upper_limit = joint_limits.max_position;
  double lower_limit = joint_limits.min_position;

  if ((base2gimbal_current_des <= upper_limit &&
       base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit)) {
    // 在限位内
  } else {
    // 超出限位，限制到边界
    des_yaw = std::clamp(des_yaw, lower_limit, upper_limit);
    des_pitch = std::clamp(des_pitch, lower_limit, upper_limit);
  }
}

void GimbalController::movejoint(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {
  yaw_controller_.update(time, period);
  pitch_controller_.update(time, period);
}

void GimbalController::feedforwardCompensation(const rclcpp::Time & /*time*/) {
  if (!enable_gravity_compensation_)
    return;
  // 重力补偿实现
}

void GimbalController::updateChassisVelocity() {
  if (!chassis_vel_)
    return;
  double linear_vel[3] = {0.0, 0.0, 0.0};
  double angular_vel[3] = {0.0, 0.0, 0.0};
  // 从底盘获取速度数据
  chassis_vel_->update(linear_vel, angular_vel, 0.001);
}

void GimbalController::commandCallback(
    rm2_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void GimbalController::trackCallback(
    rm2_msgs::msg::TrackData::ConstSharedPtr msg) {
  track_rt_buffer_.writeFromNonRT(*msg);
}

void GimbalController::rate(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to RATE mode");
  }

  // 从命令中获取速率
  double yaw_rate = cmd_gimbal_.rate_yaw;
  double pitch_rate = cmd_gimbal_.rate_pitch;

  // 积分得到期望位置
  static double yaw_des = yaw_controller_.getPosition();
  static double pitch_des = pitch_controller_.getPosition();

  yaw_des += yaw_rate * period.seconds();
  pitch_des += pitch_rate * period.seconds();

  setDes(time, yaw_des, pitch_des);
}

void GimbalController::track(const rclcpp::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to TRACK mode");
  }

  // 获取当前姿态
  double roll_real, pitch_real, yaw_real;
  tf2::Quaternion q_real;
  tf2::fromMsg(odom2pitch_.transform.rotation, q_real);
  tf2::Matrix3x3(q_real).getRPY(roll_real, pitch_real, yaw_real);

  // 获取目标（使用 TrackData 的 position 和 velocity 字段）
  geometry_msgs::msg::Point target_pos = data_track_.position;
  geometry_msgs::msg::Vector3 target_vel = data_track_.velocity;

  // 计算相对位置
  target_pos.x -= odom2pitch_.transform.translation.x;
  target_pos.y -= odom2pitch_.transform.translation.y;
  target_pos.z -= odom2pitch_.transform.translation.z;

  // 补偿底盘运动
  if (chassis_vel_) {
    target_vel.x -= chassis_vel_->linear_->x();
    target_vel.y -= chassis_vel_->linear_->y();
    target_vel.z -= chassis_vel_->linear_->z();
  }

  // 弹道求解
  if (bullet_solver_ &&
      bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed)) {
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  }
}

void GimbalController::direct(const rclcpp::Time &time) {
  if (state_changed_) {
    state_changed_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Switched to DIRECT mode");
  }

  geometry_msgs::msg::Point aim_point = cmd_gimbal_.target_pos.point;

  double dx = aim_point.x - odom2pitch_.transform.translation.x;
  double dy = aim_point.y - odom2pitch_.transform.translation.y;
  double dz = aim_point.z - odom2pitch_.transform.translation.z;

  double yaw = std::atan2(dy, dx);
  double horizontal_dist = std::sqrt(dx * dx + dy * dy);
  double pitch = -std::atan2(dz, horizontal_dist);

  setDes(time, yaw, pitch);
}

void GimbalController::assignIMUinterfaces(
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

controller_interface::CallbackReturn GimbalController::on_init() {
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

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalController::command_interface_configuration() const {
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
GimbalController::state_interface_configuration() const {
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

controller_interface::CallbackReturn GimbalController::on_configure(
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
      std::bind(&GimbalController::commandCallback, this,
                std::placeholders::_1));

  data_track_sub_ = get_node()->create_subscription<rm2_msgs::msg::TrackData>(
      "/track", rclcpp::QoS(10),
      std::bind(&GimbalController::trackCallback, this, std::placeholders::_1));

  error_pub_ = get_node()->create_publisher<rm2_msgs::msg::GimbalDesError>(
      "~/error", rclcpp::QoS(10));

  gimbal_frame_id_ = "gimbal_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.0;

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

  RCLCPP_INFO(get_node()->get_logger(),
              "Gimbal controller configured successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_activate(
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

controller_interface::CallbackReturn GimbalController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  yaw_controller_.releaseInterfaces();
  pitch_controller_.releaseInterfaces();

  RCLCPP_INFO(get_node()->get_logger(), "Gimbal controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GimbalController::update(const rclcpp::Time &time,
                         const rclcpp::Duration &period) {
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();

  try {
    odom2pitch_ =
        tf_buffer_->lookupTransform("odom", "pitch_link", tf2::TimePointZero);
    odom2base_ =
        tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "Could not get transform: %s", ex.what());
    return controller_interface::return_type::OK;
  }

  updateChassisVelocity();

  if (state_ != static_cast<ControlMode>(cmd_gimbal_.mode)) {
    state_ = static_cast<ControlMode>(cmd_gimbal_.mode);
    state_changed_ = true;
  }

  switch (state_) {
  case RATE:
    rate(time, period);
    break;
  case TRACK:
    track(time);
    break;
  case DIRECT:
    direct(time);
    break;
  default:
    RCLCPP_WARN(get_node()->get_logger(), "Unknown control mode: %d", state_);
    break;
  }

  movejoint(time, period);

  return controller_interface::return_type::OK;
}

} // namespace rm2_gimbal_controllers
