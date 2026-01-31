#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <control_msgs/srv/query_calibration_state.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <ctime>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rm2_msgs/msg/gimbal_cmd.hpp>
#include <rm2_msgs/msg/track_data.h>
#include <rm2_msgs/msg/track_data.hpp>
#include <sstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
namespace rm2_gimbal_controllers {

// 控制模式枚举
enum ControlMode { RATE = 0, TRACK, DIRECT };

// 关节限位结构体
struct JointLimits {
  double max_position = 1e16;
  double min_position = -1e16;
  double max_velocity = 1e16;
  double max_effort = 1e16;
  bool has_position_limits = false;
  bool has_velocity_limits = false;
  bool has_effort_limits = false;
};

// 质心位置结构体
struct MassPos {
  double x = 0.0;
  double z = 0.0;
};

// 关节控制器类
class JointController {
public:
};

// 主控制器类
class GimbalController : public controller_interface::ControllerInterface {
public:
  GimbalController() = default;
  ~GimbalController() override = default;
  // 生命周期回调函数
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // 配置命令和状态接口
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

protected:
  // pid参数
  double p_gain_ = 0.0;
  double i_gain_ = 0.0;
  double d_gain_ = 0.0;
  double i_max_ = 0.0;
  double i_min_ = 0.0;

  // pid状态
  double position_error_ = 0.0;
  double position_error_last_ = 0.0;
  double velocity_error_ = 0.0;
  double integral_error_ = 0.0;

  // 命令和状态
  double position_cmd_ = {0.0};
  double velocity_cmd_ = {0.0};
  double effort_cmd_ = {0.0};

  // 关节控制器对象
  JointController yaw_controller_;
  JointController pitch_controller_;

  // 硬件接口指针接口
  std::shared_ptr<hardware_interface::LoanedStateInterface> position_state_;
  std::shared_ptr<hardware_interface::LoanedStateInterface> velocity_state_;
  std::shared_ptr<hardware_interface::LoanedCommandInterface> effort_commander_;

  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 坐标变换
  geometry_msgs::msg::TransformStamped odom2gimbal_des_;
  geometry_msgs::msg::TransformStamped odom2pitch_;
  geometry_msgs::msg::TransformStamped odom2base_;
  geometry_msgs::msg::TransformStamped odom2base_last_;

  std::string gimbal_frame_id_;
private:
  // 初始化方法
  void assignIMUinterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &interfaces);

  // 控制模式方法
  void rate(const rclcpp::Time &time, const rclcpp::Duration &period);
  void track(const rclcpp::Time &time);
  void direct(const rclcpp::Time &time);

  void setDes(const rclcpp::Time &time, const double yaw, const double pitch);
  // 限位检查
  void setDesIntoLimit(double &des_yaw, double &des_pitch,
                       double base2gimbal_current_des,
                       const JointLimits &joint_limits);

  // 关节运动控制
  void movejoint(const rclcpp::Time &time, const rclcpp::Duration &period);

  // 前馈补偿
  void feedforwardCompensation(const rclcpp::Time &time);

  // 更新底盘速度
  void updateChassisVelocity();

  // 控制指令回调函数
  void commandCallback(const std::shared_ptr<rm2_msgs::msg::GimbalCmd> &msg);
  void trackCallback(const std::shared_ptr<rm2_msgs::msg::TrackData> &msg);
};

} // namespace rm2_gimbal_controllers
