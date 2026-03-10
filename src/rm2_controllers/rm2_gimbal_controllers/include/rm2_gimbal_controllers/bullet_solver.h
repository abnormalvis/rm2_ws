
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

namespace rm2_gimbal_controllers {

class BulletSolver {
public:
	explicit BulletSolver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

	bool solve(const geometry_msgs::msg::Point &target_pos,
						 const geometry_msgs::msg::Vector3 &target_vel,
						 double bullet_speed);

	double getYaw() const noexcept { return yaw_; }
	double getPitch() const noexcept { return pitch_; }

	double getGimbalError(const geometry_msgs::msg::Point &target_pos,
												const geometry_msgs::msg::Vector3 &target_vel,
												double current_yaw, double current_pitch,
												double bullet_speed);

	void bulletModelPub(const geometry_msgs::msg::TransformStamped &odom2pitch,
											const rclcpp::Time &time);

private:
	double resistanceCoefficientForSpeed(double bullet_speed) const noexcept;

	double yaw_ = 0.0;
	double pitch_ = 0.0;
	double fly_time_ = 0.0;
	double bullet_speed_ = 0.0;

	// 弹道模型参数
	double resistance_coff_ = 0.0;
	double resistance_coff_qd_10_ = 0.0;
	double resistance_coff_qd_15_ = 0.0;
	double resistance_coff_qd_16_ = 0.0;
	double resistance_coff_qd_18_ = 0.0;
	double resistance_coff_qd_30_ = 0.0;

	double gravity_ = 9.8;
	int max_iter_ = 20;

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr trajectory_pub_;
	std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

} // namespace rm2_gimbal_controllers

