//
// Created by idris on 2026/2/4.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace rm2_orientation_controller
{

class OrientationController : public controller_interface::ControllerInterface
{
public:
    OrientationController() = default;
    ~OrientationController() override = default;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    struct OrientationSample
    {
        rclcpp::Time stamp{};
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
        bool valid = false;
    };

    void assignImuInterfaces(std::vector<hardware_interface::LoanedStateInterface> & interfaces);

    bool getTransform(
        geometry_msgs::msg::TransformStamped & source2target,
        double x,
        double y,
        double z,
        double w);

    void imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string frame_source_;
    std::string frame_target_;
    std::string imu_name_;
    std::string imu_frame_id_;
    std::string imu_topic_;

    hardware_interface::LoanedStateInterface * imu_orientation_x_{};
    hardware_interface::LoanedStateInterface * imu_orientation_y_{};
    hardware_interface::LoanedStateInterface * imu_orientation_z_{};
    hardware_interface::LoanedStateInterface * imu_orientation_w_{};

    realtime_tools::RealtimeBuffer<OrientationSample> imu_rt_buffer_;
    rclcpp::Time last_imu_update_time_;

    geometry_msgs::msg::TransformStamped source2target_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    bool receive_imu_msg_ = false;
};

}  // namespace rm2_orientation_controller