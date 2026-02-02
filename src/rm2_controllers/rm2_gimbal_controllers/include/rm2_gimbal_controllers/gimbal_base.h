#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rm2_gimbal_controllers/bullet_solver.h"
#include "rm2_msgs/msg/gimbal_cmd.hpp"
#include <atomic>
#include <array>
#include <control_msgs/srv/query_calibration_state.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ctime>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <limits>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <rm2_msgs/msg/gimbal_des_error.hpp>
#include <rm2_msgs/msg/track_data.hpp>
#include <sstream>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <vector>
namespace rm2_gimbal_controllers {

//using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 控制模式枚举（与 rm2_msgs/msg/GimbalCmd.msg 保持一致）
enum ControlMode { RATE = 0, TRACK = 1, DIRECT = 2, TRAJ = 3 };

struct GimbalConfig
{
  double yaw_k_v = 0.0;
  double pitch_k_v = 0.0;
  double chassis_compensation_a = 0.0;
  double chassis_compensation_b = 0.0;
  double chassis_compensation_c = 0.0;
  double chassis_compensation_d = 0.0;
  std::vector<double> accel_pitch{};
  std::vector<double> accel_yaw{};
};

// 关节限位结构体
struct JointLimits {
  double max_position = std::numeric_limits<double>::max();
  double min_position = -std::numeric_limits<double>::max();
  double max_velocity = std::numeric_limits<double>::max();
  double max_effort = std::numeric_limits<double>::max();
  bool has_position_limits = false;
  bool has_velocity_limits = false;
  bool has_effort_limits = false;
};

// IMU类
class IMUNode {
public:
  IMUNode()
      : imu_angular_velocity_x_(nullptr), imu_angular_velocity_y_(nullptr),
        imu_angular_velocity_z_(nullptr), imu_orientation_x_(nullptr),
        imu_orientation_y_(nullptr), imu_orientation_z_(nullptr),
        imu_orientation_w_(nullptr) {}

  // 确保在析构时清理资源
  ~IMUNode() {
    // 如果LoanedStateInterface需要手动释放
    imu_angular_velocity_x_ = nullptr;
    imu_angular_velocity_y_ = nullptr;
    imu_angular_velocity_z_ = nullptr;
    imu_orientation_x_ = nullptr;
    imu_orientation_y_ = nullptr;
    imu_orientation_z_ = nullptr;
    imu_orientation_w_ = nullptr;
  }

  hardware_interface::LoanedStateInterface *imu_angular_velocity_x_{};
  hardware_interface::LoanedStateInterface *imu_angular_velocity_y_{};
  hardware_interface::LoanedStateInterface *imu_angular_velocity_z_{};
  hardware_interface::LoanedStateInterface *imu_orientation_x_{};
  hardware_interface::LoanedStateInterface *imu_orientation_y_{};
  hardware_interface::LoanedStateInterface *imu_orientation_z_{};
  hardware_interface::LoanedStateInterface *imu_orientation_w_{};

private:
  std::string imu_name_;
  std::string imu_frame_id_;
};

// 质心位置结构体
struct MassPos {
  double x = 0.0;
  double z = 0.0;
};

// 底盘跟随类
class ChassisVel {
public:
  explicit ChassisVel(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

  void update(double linear_vel[3], double angular_vel[3], double period);
  void setCutoffFrequency(double cutoff_frequency);
  double cutoffFrequency() const noexcept { return cutoff_frequency_.load(); }

  class FilteredVelocity {
  public:
    explicit FilteredVelocity(double cutoff_frequency);
    void setCutoffFrequency(double cutoff_frequency) noexcept;
    void update(const double velocity[3], double period);
    double x() const noexcept { return filtered_value_[0]; }
    double y() const noexcept { return filtered_value_[1]; }
    double z() const noexcept { return filtered_value_[2]; }

  private:
    std::atomic<double> cutoff_frequency_{0.0};
    std::array<double, 3> filtered_value_{0.0, 0.0, 0.0};
    std::array<double, 3> raw_value_{0.0, 0.0, 0.0};
  };

  std::shared_ptr<FilteredVelocity> linear_;
  std::shared_ptr<FilteredVelocity> angular_;

private:
  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter> &parameters);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::atomic<double> cutoff_frequency_{20.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;
};

// 关节控制器类
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

  void setCommand(double position_cmd, double velocity_cmd);
  void update(const rclcpp::Time &time, const rclcpp::Duration &period);

  double getPosition() const;
  double getVelocity() const;
  double getCommand() const;
  void setEffort(double effort);

  std::string joint_name_;
  std::string parent_link_name_;
  JointLimits joint_limits_;

private:
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
  struct command {
    double position_cmd_ = 0.0;
    double velocity_cmd_ = 0.0;
    double effort_cmd_ = 0.0;
  };

  // 命令实例
  command command_{};

  // 硬件接口指针
  hardware_interface::LoanedStateInterface *position_state_ = nullptr;
  hardware_interface::LoanedStateInterface *velocity_state_ = nullptr;
  hardware_interface::LoanedCommandInterface *effort_command_ = nullptr;
};

// 主控制器类
class GimbalControllerBase : public controller_interface::ControllerInterface {
public:
  GimbalControllerBase() = default;
  ~GimbalControllerBase() override = default;
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
  geometry_msgs::msg::TransformStamped odom2gimbal_traject_des_;
  geometry_msgs::msg::TransformStamped odom2pitch_;
  geometry_msgs::msg::TransformStamped odom2base_;
  geometry_msgs::msg::TransformStamped odom2base_last_;
  bool has_odom2base_last_ = false;

  std::string gimbal_frame_id_;

  // IMU相关
  bool has_imu_ = true;
  IMUNode imu_node_;

  // 前馈参数
  MassPos mass_position_;
  double gravity_ = 0.0;
  bool enable_gravity_compensation_ = false;

  // 摩擦补偿
  double yaw_compensation_ = 0.0;
  double yaw_error_tolerance_ = 0.0;

  // 底盘速度补偿
  double k_chassis_vel_ = 0.0;
  std::shared_ptr<ChassisVel> chassis_vel_;

  // 弹道求解器
  std::shared_ptr<BulletSolver> bullet_solver_;

  // 状态机
  ControlMode state_ = RATE;
  bool state_changed_ = true;

  // 订阅器
  rclcpp::Subscription<rm2_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
  rclcpp::Subscription<rm2_msgs::msg::TrackData>::SharedPtr data_track_sub_;

  // 发布器
  rclcpp::Publisher<rm2_msgs::msg::GimbalDesError>::SharedPtr error_pub_;

  // 实时缓冲区
  realtime_tools::RealtimeBuffer<rm2_msgs::msg::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm2_msgs::msg::TrackData> track_rt_buffer_;

  // 命令和跟踪数据
  rm2_msgs::msg::GimbalCmd cmd_gimbal_;
  rm2_msgs::msg::TrackData data_track_;

  // 发布频率
  double publish_rate_ = 100.0;
  rclcpp::Time last_publish_time_;

  // 动态配置（对齐 ROS1 的 dynamic_reconfigure + RT buffer）
  GimbalConfig config_{};
  realtime_tools::RealtimeBuffer<GimbalConfig> config_rt_buffer_;

  // 底盘补偿
  double chassis_compensation_ = 0.0;

  // 速度指令（用于 k_v 前馈）
  double yaw_vel_des_ = 0.0;
  double pitch_vel_des_ = 0.0;

  // RATE 模式启动对齐
  bool start_ = true;

  double yaw_des_ = 0.0;
  double pitch_des_ = 0.0;

  // 动态调参

private:
  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;

  // 初始化方法
  void assignIMUinterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &interfaces);

  // 控制模式方法
  void rate(const rclcpp::Time &time, const rclcpp::Duration &period);
  void track(const rclcpp::Time &time);
  void direct(const rclcpp::Time &time);
  void traj(const rclcpp::Time &time);

  void setDes(const rclcpp::Time &time, const double yaw, const double pitch);
  // 限位检查
  bool setDesIntoLimit(double &des, const JointLimits &joint_limits);

  // 关节运动控制
  void movejoint(const rclcpp::Time &time, const rclcpp::Duration &period);

  // 前馈补偿
  void feedforwardCompensation(const rclcpp::Time &time);

  // 更新底盘速度
  void updateChassisVelocity();

  double updateCompensation(double chassis_vel_angular_z);
  double feedForwardPitch(const rclcpp::Time &time);

  // 控制指令回调函数
  void commandCallback(rm2_msgs::msg::GimbalCmd::ConstSharedPtr msg);
  void trackCallback(rm2_msgs::msg::TrackData::ConstSharedPtr msg);
};

} // namespace rm2_gimbal_controllers
