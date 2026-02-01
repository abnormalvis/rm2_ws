
#include "rm2_gimbal_controllers/bullet_solver.h"

#include <angles/angles.h>
#include <cmath>
#include <limits>

namespace rm2_gimbal_controllers {

static double getDoubleParamOr(const rclcpp_lifecycle::LifecycleNode &node,
                               const std::string &name, double default_value) {
  if (!node.has_parameter(name)) {
    return default_value;
  }
  return node.get_parameter(name).as_double();
}

static int getIntParamOr(const rclcpp_lifecycle::LifecycleNode &node,
                         const std::string &name, int default_value) {
  if (!node.has_parameter(name)) {
    return default_value;
  }
  return node.get_parameter(name).as_int();
}

BulletSolver::BulletSolver(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
    : node_(std::move(node)) {
  resistance_coff_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff", 0.0);

  // 支持按弹速选择不同阻力系数
  resistance_coff_qd_10_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff_qd_10", 0.0);
  resistance_coff_qd_15_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff_qd_15", 0.0);
  resistance_coff_qd_16_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff_qd_16", 0.0);
  resistance_coff_qd_18_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff_qd_18", 0.0);
  resistance_coff_qd_30_ =
      getDoubleParamOr(*node_, "bullet_solver.resistance_coff_qd_30", 0.0);

  gravity_ = getDoubleParamOr(*node_, "bullet_solver.g", 9.8);
  max_iter_ = getIntParamOr(*node_, "bullet_solver.max_iter", 20);

  trajectory_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      "~/trajectory", 10);
}

double BulletSolver::resistanceCoefficientForSpeed(
    double bullet_speed) const noexcept {
  auto near = [](double a, double b) { return std::abs(a - b) < 0.5; };

  if (near(bullet_speed, 10.0) && resistance_coff_qd_10_ != 0.0)
    return resistance_coff_qd_10_;
  if (near(bullet_speed, 15.0) && resistance_coff_qd_15_ != 0.0)
    return resistance_coff_qd_15_;
  if (near(bullet_speed, 16.0) && resistance_coff_qd_16_ != 0.0)
    return resistance_coff_qd_16_;
  if (near(bullet_speed, 18.0) && resistance_coff_qd_18_ != 0.0)
    return resistance_coff_qd_18_;
  if (near(bullet_speed, 30.0) && resistance_coff_qd_30_ != 0.0)
    return resistance_coff_qd_30_;

  return resistance_coff_;
}

bool BulletSolver::solve(const geometry_msgs::msg::Point &target_pos,
                         const geometry_msgs::msg::Vector3 &target_vel,
                         double bullet_speed) {
  if (bullet_speed <= 0.0) {
    yaw_ = 0.0;
    pitch_ = 0.0;
    fly_time_ = 0.0;
    bullet_speed_ = 0.0;
    return false;
  }

  bullet_speed_ = bullet_speed;

  // 初始化飞行时间：用水平距离/弹速作为初值
  double x0 =
      std::sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);
  double fly_time = x0 / bullet_speed;

  const double k = resistanceCoefficientForSpeed(bullet_speed);

  for (int i = 0; i < max_iter_; ++i) {
    // 迭代中考虑目标运动补偿（更稳定）
    const double compensated_x = target_pos.x + target_vel.x * fly_time;
    const double compensated_y = target_pos.y + target_vel.y * fly_time;
    const double compensated_z = target_pos.z + target_vel.z * fly_time;

    const double x = std::sqrt(compensated_x * compensated_x +
                               compensated_y * compensated_y);
    const double y = compensated_z;

    // 若 k=0，退化为无阻力的近似：t = x / v
    if (k == 0.0) {
      const double t_new = x / bullet_speed;
      if (std::abs(t_new - fly_time) < 1e-4) {
        fly_time = t_new;
        break;
      }
      fly_time = t_new;
      continue;
    }

    // 估计初始仰角方向对应的竖直速度分量
    const double aim_angle = std::atan2(y, x);
    const double v_vertical = bullet_speed * std::sin(aim_angle);

    const double denom = (k * v_vertical + gravity_);
    if (denom == 0.0) {
      break;
    }

    const double t_new = (std::exp(k * fly_time) - 1.0) / denom;
    if (!std::isfinite(t_new) || t_new <= 0.0) {
      break;
    }

    if (std::abs(t_new - fly_time) < 1e-3) {
      fly_time = t_new;
      break;
    }
    fly_time = t_new;
  }

  fly_time_ = fly_time;

  const double compensated_x = target_pos.x + target_vel.x * fly_time;
  const double compensated_y = target_pos.y + target_vel.y * fly_time;
  const double compensated_z = target_pos.z + target_vel.z * fly_time;

  yaw_ = std::atan2(compensated_y, compensated_x);
  const double horizontal_dist =
      std::sqrt(compensated_x * compensated_x + compensated_y * compensated_y);

  // 这里保持原有实现的符号约定，避免影响上层控制逻辑
  pitch_ = std::atan2(compensated_z, horizontal_dist);
  pitch_ += std::atan2(0.5 * gravity_ * fly_time * fly_time, horizontal_dist);

  return true;
}

double
BulletSolver::getGimbalError(const geometry_msgs::msg::Point &target_pos,
                             const geometry_msgs::msg::Vector3 &target_vel,
                             double current_yaw, double current_pitch,
                             double bullet_speed) {
  if (!solve(target_pos, target_vel, bullet_speed)) {
    return 0.0;
  }
  const double yaw_error = angles::shortest_angular_distance(current_yaw, yaw_);
  const double pitch_error =
      angles::shortest_angular_distance(current_pitch, pitch_);
  return std::sqrt(yaw_error * yaw_error + pitch_error * pitch_error);
}

void BulletSolver::bulletModelPub(
    const geometry_msgs::msg::TransformStamped &odom2pitch,
    const rclcpp::Time &time) {
  if (!trajectory_pub_) {
    return;
  }
  if (fly_time_ <= 0.0 || bullet_speed_ <= 0.0 || !std::isfinite(fly_time_)) {
    return;
  }

  geometry_msgs::msg::PointStamped trajectory_point;
  trajectory_point.header.stamp = time;
  trajectory_point.header.frame_id = "odom";

  const double x =
      odom2pitch.transform.translation.x +
      std::cos(yaw_) * std::cos(pitch_) * fly_time_ * bullet_speed_;
  const double y =
      odom2pitch.transform.translation.y +
      std::sin(yaw_) * std::cos(pitch_) * fly_time_ * bullet_speed_;
  const double z = odom2pitch.transform.translation.z +
                   std::sin(pitch_) * fly_time_ * bullet_speed_ -
                   0.5 * gravity_ * fly_time_ * fly_time_;

  trajectory_point.point.x = x;
  trajectory_point.point.y = y;
  trajectory_point.point.z = z;
  trajectory_pub_->publish(trajectory_point);
}

} // namespace rm2_gimbal_controllers
