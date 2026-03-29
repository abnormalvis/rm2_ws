#include "rm2_gimbal_controllers/gimbal_controller.h"
#include "rm2_gimbal_controllers/bullet_solver.h"
#include "rm2_gimbal_controllers/pid_debug_publisher.h"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rm2_gimbal_controllers {

controller_interface::CallbackReturn GimbalController::on_init() {
    try {
        param_listener_ =
                std::make_shared<gimbal_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(),
                                 "Exception during GimbalController::on_init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    // 重力前馈补偿
    if (params_.feedforward.enable) {
        enable_gravity_compensation_ = true;
        mass_position_.x = params_.feedforward.mass_origin.x;
        mass_position_.z = params_.feedforward.mass_origin.z;
        gravity_ = params_.feedforward.gravity;
    } else {
        enable_gravity_compensation_ = false;
    }

    // 摩擦/阻力补偿
    yaw_compensation_ = params_.yaw.resistance_compensation.resistance;
    yaw_error_tolerance_ = params_.yaw.resistance_compensation.error_tolerance;
    k_chassis_vel_ = params_.yaw.k_chassis_vel;

    publish_rate_ = params_.publish_rate;
    has_imu_ = !params_.imu_name.empty();

    if (!has_imu_) {
        RCLCPP_WARN(get_node()->get_logger(),
                                "IMU not configured, some features may be limited");
    }

    // 子系统初始化（这些内部仍调用 get_parameter()，参数已由 ParamListener 声明）
    chassis_vel_ = std::make_shared<ChassisVel>(get_node());
    bullet_solver_ = std::make_shared<BulletSolver>(get_node());

    // 动态参数回调
    if (!param_cb_handle_) {
        param_cb_handle_ = get_node()->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &parameters) {
                    return this->paramCallback(parameters);
                });
    }

    // 关节控制器初始化（内部使用 PidROS，参数已由 ParamListener 声明）
    if (!yaw_controller_.init(get_node(), "yaw") ||
            !pitch_controller_.init(get_node(), "pitch")) {
        RCLCPP_ERROR(get_node()->get_logger(),
                                 "Failed to initialize joint controllers");
        return controller_interface::CallbackReturn::ERROR;
    }

    // TF 系统
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

    // 话题订阅与发布
    cmd_gimbal_sub_ = get_node()->create_subscription<rm2_msgs::msg::GimbalCmd>(
            "~/command", rclcpp::QoS(10),
            [this](rm2_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
                this->commandCallback(msg);
            });

    target_angle_sub_ =
            get_node()->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                    "~/command/target_angle", rclcpp::QoS(10),
                    [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                        this->targetAngleCallback(msg);
                    });

    data_track_sub_ = get_node()->create_subscription<rm2_msgs::msg::TrackData>(
            "/track", rclcpp::QoS(10),
            [this](rm2_msgs::msg::TrackData::ConstSharedPtr msg) {
                this->trackCallback(msg);
            });

    error_pub_ = get_node()->create_publisher<rm2_msgs::msg::GimbalDesError>(
            "~/error", rclcpp::QoS(10));
    pid_debug_publisher_ = std::make_shared<PIDDebugPublisher>(get_node());
    pid_debug_publisher_->setPublishRate(publish_rate_);

    // TF 帧 ID 初始化
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

    // 实时缓冲区初始值
    cmd_gimbal_.mode = RATE;
    cmd_gimbal_.rate_yaw = 0.0;
    cmd_gimbal_.rate_pitch = 0.0;
    cmd_gimbal_.bullet_speed = 15.0;
    cmd_rt_buffer_.writeFromNonRT(cmd_gimbal_);

    data_track_.id = 0;
    track_rt_buffer_.writeFromNonRT(data_track_);

    target_angle_cmd_.header.frame_id = "base_link";
    target_angle_cmd_.vector.x = 0.0;
    target_angle_cmd_.vector.y = 0.0;
    target_angle_cmd_.vector.z = 0.0;
    target_angle_rt_buffer_.writeFromNonRT(target_angle_cmd_);

    // 动态可调参数（同步到实时缓冲区）
    config_.yaw_k_v = params_.yaw_k_v;
    config_.pitch_k_v = params_.pitch_k_v;
    config_.chassis_compensation_a = params_.chassis_compensation.a;
    config_.chassis_compensation_b = params_.chassis_compensation.b;
    config_.chassis_compensation_c = params_.chassis_compensation.c;
    config_.chassis_compensation_d = params_.chassis_compensation.d;
    config_.accel_pitch = params_.accel_pitch;
    config_.accel_yaw = params_.accel_yaw;
    config_rt_buffer_.writeFromNonRT(config_);

    yaw_des_ = 0.0;
    pitch_des_ = 0.0;
    start_ = true;
    has_odom2base_last_ = false;

    RCLCPP_INFO(get_node()->get_logger(),
                            "GimbalController configured successfully");
    return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace rm2_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm2_gimbal_controllers::GimbalController,
                                             controller_interface::ControllerInterface)