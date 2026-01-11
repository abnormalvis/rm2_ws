//
// Created by ch on 2025/9/25.
//

#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <type_traits>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rm2_msgs/msg/chassis_cmd.hpp>
#include <rm2_msgs/msg/gimbal_cmd.hpp>
#include <rm2_msgs/msg/shoot_cmd.hpp>
#include <rm2_msgs/msg/shoot_beforehand_cmd.hpp>
#include <rm2_msgs/msg/gimbal_des_error.hpp>
#include <rm2_msgs/msg/state_cmd.hpp>
#include <rm2_msgs/msg/track_data.hpp>
#include <rm2_msgs/msg/game_robot_hp.hpp>
#include <rm2_msgs/msg/shoot_data.hpp>
#include <rm2_msgs/msg/leg_cmd.hpp>
#include <rm2_msgs/srv/status_change.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rm2_msgs/msg/multi_dof_cmd.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/joint_controller_state.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "rm2_common/decision/heat_limit.h"
#include "rm2_common/decision/power_limit.h"
#include "rm2_common/linear_interpolation.h"
#include "rm2_common/filters/filters.h"
#include "rm2_msgs/srv/status_change.hpp"

namespace rm2_common
{
template <class MsgType>
class CommandSenderBase
{
public:
  explicit CommandSenderBase(rclcpp::Node::SharedPtr node)
  {
    if (!node->get_parameter("topic", topic_))
      RCLCPP_ERROR(node->get_logger(), "Topic name no defined (namespace: %s)", node->get_namespace());
    queue_size_ = node->declare_parameter("queue_size", 1);
    pub_ = node->create_publisher<MsgType>(topic_, rclcpp::QoS(queue_size_));
  }
  void setMode(int mode)
  {
    if (!std::is_same<MsgType, geometry_msgs::msg::Twist>::value && !std::is_same<MsgType, std_msgs::msg::Float64>::value)
      msg_.mode = mode;
  }
  virtual void sendCommand(const rclcpp::Time& time)
  {
    pub_->publish(msg_);
  }
  virtual void updateGameRobotStatus(const rm2_msgs::msg::GameRobotStatus data)
  {
  }
  virtual void updateGameStatus(const rm2_msgs::msg::GameStatus data)
  {
  }
  virtual void updateCapacityData(const rm2_msgs::msg::PowerManagementSampleAndStatusData data)
  {
  }
  virtual void updatePowerHeatData(const rm2_msgs::msg::PowerHeatData data)
  {
  }
  virtual void setZero() = 0;
  MsgType* getMsg()
  {
    return &msg_;
  }

protected:
  std::string topic_;
  uint32_t queue_size_;
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  MsgType msg_;
};

template <class MsgType>
class TimeStampCommandSenderBase : public CommandSenderBase<MsgType>
{
public:
  explicit TimeStampCommandSenderBase(rclcpp::Node::SharedPtr node) : CommandSenderBase<MsgType>(node)
  {
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

template <class MsgType>
class HeaderStampCommandSenderBase : public CommandSenderBase<MsgType>
{
public:
  explicit HeaderStampCommandSenderBase(rclcpp::Node::SharedPtr node) : CommandSenderBase<MsgType>(node)
  {
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.header.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

class Vel2DCommandSender : public CommandSenderBase<geometry_msgs::msg::Twist>
{
public:
  explicit Vel2DCommandSender(rclcpp::Node::SharedPtr node) 
  : CommandSenderBase<geometry_msgs::msg::Twist>(node), max_linear_x_(node), max_linear_y_(node), max_angular_z_(node)
  {
    std::vector<double> input_vector, output_vector;
    if (!node->get_parameter("max_linear_x_input", input_vector) || !node->get_parameter("max_linear_x_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Max X linear velocity defined wrong (namespace: %s)", node->get_namespace());
    else
      max_linear_x_.init(input_vector, output_vector);
    if (!node->get_parameter("max_linear_y_input", input_vector) || !node->get_parameter("max_linear_y_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Max Y linear velocity defined wrong (namespace: %s)", node->get_namespace());
    else
      max_linear_y_.init(input_vector, output_vector);
    if (!node->get_parameter("max_angular_z_input", input_vector) || !node->get_parameter("max_angular_z_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Max Z angular velocity defined wrong (namespace: %s)", node->get_namespace());
    else
      max_angular_z_.init(input_vector, output_vector);

    std::string topic;
    node->get_parameter("power_limit_topic", topic);
    target_vel_yaw_threshold_ = node->declare_parameter("target_vel_yaw_threshold", 3.);
    chassis_power_limit_subscriber_ = 
      node->create_subscription<rm2_msgs::msg::ChassisCmd>(
      topic,
      rclcpp::QoS(1),
      [this](const rm2_msgs::msg::ChassisCmd::ConstSharedPtr& msg) {
        this->chassisCmdCallback(msg);});
  }

  void updateTrackData(const rm2_msgs::msg::TrackData& data)
  {
    track_data_ = data;
  }
  void setLinearXVel(double scale)
  {
    msg_.linear.x = scale * max_linear_x_.output(power_limit_);
  };
  void setLinearYVel(double scale)
  {
    msg_.linear.y = scale * max_linear_y_.output(power_limit_);
  };
  void setAngularZVel(double scale)
  {
    if (track_data_.v_yaw > target_vel_yaw_threshold_)
      vel_direction_ = -1.;
    if (track_data_.v_yaw < -target_vel_yaw_threshold_)
      vel_direction_ = 1.;
    msg_.angular.z = scale * max_angular_z_.output(power_limit_) * vel_direction_;
  };
  void setAngularZVel(double scale, double limit)
  {
    if (track_data_.v_yaw > target_vel_yaw_threshold_)
      vel_direction_ = -1.;
    if (track_data_.v_yaw < -target_vel_yaw_threshold_)
      vel_direction_ = 1.;
    double angular_z = max_angular_z_.output(power_limit_) > limit ? limit : max_angular_z_.output(power_limit_);
    msg_.angular.z = scale * angular_z * vel_direction_;
  };
  void set2DVel(double scale_x, double scale_y, double scale_z)
  {
    setLinearXVel(scale_x);
    setLinearYVel(scale_y);
    setAngularZVel(scale_z);
  }
  void setZero() override
  {
    msg_.linear.x = 0.;
    msg_.linear.y = 0.;
    msg_.angular.z = 0.;
  }

protected:
  void chassisCmdCallback(const rm2_msgs::msg::ChassisCmd::ConstSharedPtr& msg)
  {
    power_limit_ = msg->power_limit;
  }

  LinearInterp max_linear_x_, max_linear_y_, max_angular_z_;
  double power_limit_ = 0.;
  double target_vel_yaw_threshold_{};
  double vel_direction_ = 1.;
  rclcpp::Subscription<rm2_msgs::msg::ChassisCmd>::SharedPtr chassis_power_limit_subscriber_;
  rm2_msgs::msg::TrackData track_data_;
};

class ChassisCommandSender : public TimeStampCommandSenderBase<rm2_msgs::msg::ChassisCmd>
{
public:
  explicit ChassisCommandSender(rclcpp::Node::SharedPtr node) 
  : TimeStampCommandSenderBase<rm2_msgs::msg::ChassisCmd>(node), accel_x_(node), accel_y_(node), accel_z_(node)
  {
    power_limit_ = new PowerLimit(node);
    std::vector<double> input_vector, output_vector;
    if (!node->get_parameter("accel_x_input", input_vector) || !node->get_parameter("accel_x_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Accel X no defined (namespace: %s)", node->get_namespace());
    else
      accel_x_.init(input_vector, output_vector);
    if (!node->get_parameter("accel_y_input", input_vector) || !node->get_parameter("accel_y_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Accel Y no defined (namespace: %s)", node->get_namespace());
    else
      accel_y_.init(input_vector, output_vector);
    if (!node->get_parameter("accel_z_input", input_vector) || !node->get_parameter("accel_z_output", output_vector))
      RCLCPP_ERROR(node->get_logger(),"Accel Z no defined (namespace: %s)", node->get_namespace());
    else
      accel_z_.init(input_vector, output_vector);

  }

  void updateSafetyPower(int safety_power)
  {
    power_limit_->updateSafetyPower(safety_power);
  }
  void updateGameRobotStatus(const rm2_msgs::msg::GameRobotStatus data) override
  {
    power_limit_->setGameRobotData(data);
  }
  void updatePowerHeatData(const rm2_msgs::msg::PowerHeatData data) override
  {
    power_limit_->setChassisPowerBuffer(data);
  }
  void updateCapacityData(const rm2_msgs::msg::PowerManagementSampleAndStatusData data) override
  {
    power_limit_->setCapacityData(data);
  }
  void updateRefereeStatus(bool status)
  {
    power_limit_->setRefereeStatus(status);
  }
  void setFollowVelDes(double follow_vel_des)
  {
    msg_.follow_vel_des = follow_vel_des;
  }
  void setWirelessState(bool state)
  {
    msg_.wireless_state = state;
  }
  void sendChassisCommand(const rclcpp::Time& time, bool is_gyro)
  {
    power_limit_->setLimitPower(msg_, is_gyro);
    msg_.accel.linear.x = accel_x_.output(msg_.power_limit);
    msg_.accel.linear.y = accel_y_.output(msg_.power_limit);
    msg_.accel.angular.z = accel_z_.output(msg_.power_limit);
    TimeStampCommandSenderBase<rm2_msgs::msg::ChassisCmd>::sendCommand(time);
  }
  void setZero() override{};
  PowerLimit* power_limit_;

private:
  LinearInterp accel_x_, accel_y_, accel_z_;
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm2_msgs::msg::GimbalCmd>
{
public:
  explicit GimbalCommandSender(rclcpp::Node::SharedPtr node) 
  : TimeStampCommandSenderBase<rm2_msgs::msg::GimbalCmd>(node)
  {
    if (!node->get_parameter("max_yaw_vel", max_yaw_vel_))
      RCLCPP_ERROR(node->get_logger(), "Max yaw velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_pitch_vel", max_pitch_vel_))
      RCLCPP_ERROR(node->get_logger(), "Max pitch velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("time_constant_rc", time_constant_rc_))
      RCLCPP_ERROR(node->get_logger(), "Time constant rc no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("time_constant_pc", time_constant_pc_))
      RCLCPP_ERROR(node->get_logger(), "Time constant pc no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("track_timeout", track_timeout_))
      RCLCPP_ERROR(node->get_logger(), "Track timeout no defined (namespace: %s)", node->get_namespace());
    node->declare_parameter("eject_sensitivity",1.);
    node->get_parameter("eject_sensitivity", eject_sensitivity_);
  }
  ~GimbalCommandSender() = default;
  void setRate(double scale_yaw, double scale_pitch)
  {
    if (std::abs(scale_yaw) > 1)
      scale_yaw = scale_yaw > 0 ? 1 : -1;
    if (std::abs(scale_pitch) > 1)
      scale_pitch = scale_pitch > 0 ? 1 : -1;
    double time_constant{};
    if (use_rc_)
      time_constant = time_constant_rc_;
    else
      time_constant = time_constant_pc_;
    msg_.rate_yaw = msg_.rate_yaw + (scale_yaw * max_yaw_vel_ - msg_.rate_yaw) * (0.001 / (time_constant + 0.001));
    msg_.rate_pitch =
        msg_.rate_pitch + (scale_pitch * max_pitch_vel_ - msg_.rate_pitch) * (0.001 / (time_constant + 0.001));
    if (eject_flag_)
    {
      msg_.rate_yaw *= eject_sensitivity_;
      msg_.rate_pitch *= eject_sensitivity_;
    }
  }
  void setGimbalTraj(double traj_yaw, double traj_pitch)
  {
    msg_.traj_yaw = traj_yaw;
    msg_.traj_pitch = traj_pitch;
  }
  void setGimbalTrajFrameId(const std::string& traj_frame_id)
  {
    msg_.traj_frame_id = traj_frame_id;
  }
  void setZero() override
  {
    msg_.rate_yaw = 0.;
    msg_.rate_pitch = 0.;
  }
  void setBulletSpeed(double bullet_speed)
  {
    msg_.bullet_speed = bullet_speed;
  }
  void setEject(bool flag)
  {
    eject_flag_ = flag;
  }
  void setUseRc(bool flag)
  {
    use_rc_ = flag;
  }
  bool getEject() const
  {
    return eject_flag_;
  }
  void setPoint(geometry_msgs::msg::PointStamped point)
  {
    msg_.target_pos = point;
  }

private:
  double max_yaw_vel_{}, max_pitch_vel_{}, track_timeout_{}, eject_sensitivity_ = 1.;
  double time_constant_rc_{}, time_constant_pc_{};
  bool eject_flag_{}, use_rc_{};
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm2_msgs::msg::ShootCmd>
{
public:
  explicit ShooterCommandSender(rclcpp::Node::SharedPtr node) : TimeStampCommandSenderBase<rm2_msgs::msg::ShootCmd>(node)
  {
    auto limit_node = node->create_sub_node("heat_limit");
    heat_limit_ = new HeatLimit(limit_node);
    speed_10_ = node->declare_parameter<double>("speed_10m_per_speed", 10.);
    speed_15_ = node->declare_parameter<double>("speed_15m_per_speed", 15.);
    speed_16_ = node->declare_parameter<double>("speed_16m_per_speed", 16.);
    speed_18_ = node->declare_parameter<double>("speed_18m_per_speed", 18.);
    speed_30_ = node->declare_parameter<double>("speed_30m_per_speed", 30.);

    if (!node->get_parameter("wheel_speed_10", wheel_speed_10_))
      RCLCPP_ERROR(node->get_logger(), "Parameter 'wheel_speed_10' not defined");
    if (!node->get_parameter("wheel_speed_15", wheel_speed_15_))
      RCLCPP_ERROR(node->get_logger(), "Parameter 'wheel_speed_15' not defined");
    if (!node->get_parameter("wheel_speed_16", wheel_speed_16_))
      RCLCPP_ERROR(node->get_logger(), "Parameter 'wheel_speed_16' not defined");
    if (!node->get_parameter("wheel_speed_18", wheel_speed_18_))
      RCLCPP_ERROR(node->get_logger(), "Parameter 'wheel_speed_18' not defined");
    if (!node->get_parameter("wheel_speed_30", wheel_speed_30_))
      RCLCPP_ERROR(node->get_logger(), "Parameter 'wheel_speed_30' not defined");

    wheel_speed_offset_front_ = node->declare_parameter<double>("wheel_speed_offset_front", 0.0);
    wheel_speed_offset_back_ = node->declare_parameter<double>("wheel_speed_offset_back", 0.0);
    speed_oscillation_ = node->declare_parameter<double>("speed_oscillation", 1.0);
    extra_wheel_speed_once_ = node->declare_parameter<double>("extra_wheel_speed_once", 0.);
    deploy_wheel_speed_ = node->declare_parameter<double>("deploy_wheel_speed", 410.0);

    if (!node->has_parameter("auto_wheel_speed")) {
      auto_wheel_speed_ = node->declare_parameter<bool>("auto_wheel_speed", false);
      RCLCPP_INFO(node->get_logger(), 
                    "auto_wheel_speed not defined (namespace: %s), set to false.", 
                    node->get_namespace());
    }
    if (!node->has_parameter("target_acceleration_tolerance")) {
      target_acceleration_tolerance_ = node->declare_parameter<double>("target_acceleration_tolerance", 0.);
      RCLCPP_INFO(node->get_logger(), 
                    "target_acceleration_tolerance no defined(namespace: %s), set to zero.", 
                    node->get_namespace());
    }
    if (!node->get_parameter("track_armor_error_tolerance", track_armor_error_tolerance_))
      RCLCPP_ERROR(node->get_logger(), "track armor error tolerance no defined (namespace: %s)", node->get_namespace());
    untrack_armor_error_tolerance_ = node->declare_parameter<double>("untrack_armor_error_tolerance", track_armor_error_tolerance_);
    track_buff_error_tolerance_ = node->declare_parameter<double>("track_buff_error_tolerance", track_armor_error_tolerance_);
    if (!node->has_parameter("max_track_target_vel")) {
      max_track_target_vel_ = node->declare_parameter<double>("max_track_target_vel", 9.0);
      RCLCPP_INFO(node->get_logger(), 
                    "max track target vel no defined (namespace: %s)", 
                    node->get_namespace());
    }
  }
  ~ShooterCommandSender()
  {
    delete heat_limit_;
  }

  void updateGameRobotStatus(const rm2_msgs::msg::GameRobotStatus data) override
  {
    heat_limit_->setStatusOfShooter(data);
  }
  void updatePowerHeatData(const rm2_msgs::msg::PowerHeatData data) override
  {
    heat_limit_->setCoolingHeatOfShooter(data);
  }
  void updateRefereeStatus(bool status)
  {
    heat_limit_->setRefereeStatus(status);
  }
  void updateGimbalDesError(const rm2_msgs::msg::GimbalDesError& error)
  {
    gimbal_des_error_ = error;
  }
  void updateShootBeforehandCmd(const rm2_msgs::msg::ShootBeforehandCmd& data)
  {
    shoot_beforehand_cmd_ = data;
  }
  void updateTrackData(const rm2_msgs::msg::TrackData& data)
  {
    track_data_ = data;
  }
  void updateSuggestFireData(const std_msgs::msg::Bool& data)
  {
    suggest_fire_ = data;
  }
  void updateShootData(const rm2_msgs::msg::ShootData& data)
  {
    shoot_data_ = data;
    if (auto_wheel_speed_)
    {
      if (last_bullet_speed_ == 0.0)
        last_bullet_speed_ = speed_des_;
      if (shoot_data_.bullet_speed != last_bullet_speed_)
      {
        if (last_bullet_speed_ - speed_des_ >= speed_oscillation_ || shoot_data_.bullet_speed > speed_limit_)
        {
          total_extra_wheel_speed_ -= 5.0;
        }
        else if (speed_des_ - last_bullet_speed_ > speed_oscillation_)
        {
          total_extra_wheel_speed_ += 5.0;
        }
      }
      if (shoot_data_.bullet_speed != 0.0)
        last_bullet_speed_ = shoot_data_.bullet_speed;
    }
  }
  void checkError(const rclcpp::Time& time)
  {
    if (msg_.mode == rm2_msgs::msg::ShootCmd::PUSH && time - shoot_beforehand_cmd_.stamp < rclcpp::Duration::from_seconds(0.1))
    {
      if (shoot_beforehand_cmd_.cmd == rm2_msgs::msg::ShootBeforehandCmd::ALLOW_SHOOT)
        return;
      if (shoot_beforehand_cmd_.cmd == rm2_msgs::msg::ShootBeforehandCmd::BAN_SHOOT)
      {
        setMode(rm2_msgs::msg::ShootCmd::READY);
        return;
      }
    }
    double gimbal_error_tolerance;
    if (track_data_.id == 12)
      gimbal_error_tolerance = track_buff_error_tolerance_;
    else if (std::abs(track_data_.v_yaw) < max_track_target_vel_)
      gimbal_error_tolerance = track_armor_error_tolerance_;
    else
      gimbal_error_tolerance = untrack_armor_error_tolerance_;
    if (((gimbal_des_error_.error > gimbal_error_tolerance && time - gimbal_des_error_.stamp < rclcpp::Duration::from_seconds(0.1)) ||
         (track_data_.accel > target_acceleration_tolerance_)) ||
        (!suggest_fire_.data && armor_type_ == rm2_msgs::srv::StatusChange_Request::ARMOR_OUTPOST_BASE))
      if (msg_.mode == rm2_msgs::msg::ShootCmd::PUSH)
        setMode(rm2_msgs::msg::ShootCmd::READY);
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    msg_.wheel_speed = getWheelSpeedDes();
    msg_.wheels_speed_offset_front = getFrontWheelSpeedOffset();
    msg_.wheels_speed_offset_back = getBackWheelSpeedOffset();
    msg_.hz = heat_limit_->getShootFrequency();
    TimeStampCommandSenderBase<rm2_msgs::msg::ShootCmd>::sendCommand(time);
  }
  double getSpeed()
  {
    setSpeedDesAndWheelSpeedDes();
    return speed_des_;
  }
  double getWheelSpeedDes()
  {
    setSpeedDesAndWheelSpeedDes();
    if (hero_flag_)
    {
      if (deploy_flag_)
        return deploy_wheel_speed_;
      return wheel_speed_des_;
    }
    return wheel_speed_des_ + total_extra_wheel_speed_;
  }
  void setDeployState(bool flag)
  {
    deploy_flag_ = flag;
  }
  void setHeroState(bool flag)
  {
    hero_flag_ = flag;
  }
  bool getDeployState()
  {
    return deploy_flag_;
  }
  void setSpeedDesAndWheelSpeedDes()
  {
    switch (heat_limit_->getSpeedLimit())
    {
      case rm2_msgs::msg::ShootCmd::SPEED_10M_PER_SECOND:
      {
        speed_limit_ = 10.0;
        speed_des_ = speed_10_;
        wheel_speed_des_ = wheel_speed_10_;
        break;
      }
      case rm2_msgs::msg::ShootCmd::SPEED_15M_PER_SECOND:
      {
        speed_limit_ = 15.0;
        speed_des_ = speed_15_;
        wheel_speed_des_ = wheel_speed_15_;
        break;
      }
      case rm2_msgs::msg::ShootCmd::SPEED_16M_PER_SECOND:
      {
        speed_limit_ = 16.0;
        speed_des_ = speed_16_;
        wheel_speed_des_ = wheel_speed_16_;
        break;
      }
      case rm2_msgs::msg::ShootCmd::SPEED_18M_PER_SECOND:
      {
        speed_limit_ = 18.0;
        speed_des_ = speed_18_;
        wheel_speed_des_ = wheel_speed_18_;
        break;
      }
      case rm2_msgs::msg::ShootCmd::SPEED_30M_PER_SECOND:
      {
        speed_limit_ = 30.0;
        speed_des_ = speed_30_;
        wheel_speed_des_ = wheel_speed_30_;
        break;
      }
    }
  }
  double getFrontWheelSpeedOffset()
  {
    wheels_speed_offset_front_ = wheel_speed_offset_front_;
    return wheels_speed_offset_front_;
  }
  double getBackWheelSpeedOffset()
  {
    wheels_speed_offset_back_ = wheel_speed_offset_back_;
    return wheels_speed_offset_back_;
  }
  void dropSpeed()
  {
    if (hero_flag_)
      wheel_speed_offset_front_ -= extra_wheel_speed_once_;
    else
      total_extra_wheel_speed_ -= extra_wheel_speed_once_;
  }
  void raiseSpeed()
  {
    if (hero_flag_)
      wheel_speed_offset_front_ += extra_wheel_speed_once_;
    else
      total_extra_wheel_speed_ += extra_wheel_speed_once_;
  }
  void setArmorType(uint8_t armor_type)
  {
    armor_type_ = armor_type;
  }
  void setShootFrequency(uint8_t mode)
  {
    heat_limit_->setShootFrequency(mode);
  }
  uint8_t getShootFrequency()
  {
    return heat_limit_->getShootFrequencyMode();
  }
  void setZero() override{};
  HeatLimit* heat_limit_{};

private:
  double speed_10_{}, speed_15_{}, speed_16_{}, speed_18_{}, speed_30_{}, speed_des_{}, speed_limit_{};
  double wheel_speed_10_{}, wheel_speed_15_{}, wheel_speed_16_{}, wheel_speed_18_{}, wheel_speed_30_{},
      wheel_speed_des_{}, last_bullet_speed_{}, speed_oscillation_{};
  double wheel_speed_offset_front_{}, wheel_speed_offset_back_{};
  double wheels_speed_offset_front_{}, wheels_speed_offset_back_{};
  double track_armor_error_tolerance_{};
  double track_buff_error_tolerance_{};
  double untrack_armor_error_tolerance_{};
  double max_track_target_vel_{};
  double target_acceleration_tolerance_{};
  double extra_wheel_speed_once_{};
  double total_extra_wheel_speed_{};
  double deploy_wheel_speed_{};
  bool auto_wheel_speed_ = false;
  bool hero_flag_{};
  bool deploy_flag_ = false;
  rm2_msgs::msg::TrackData track_data_;
  rm2_msgs::msg::GimbalDesError gimbal_des_error_;
  rm2_msgs::msg::ShootBeforehandCmd shoot_beforehand_cmd_;
  rm2_msgs::msg::ShootData shoot_data_;
  std_msgs::msg::Bool suggest_fire_;
  uint8_t armor_type_{};
};

class UseLioCommandSender : public CommandSenderBase<std_msgs::msg::Bool>
{
public:
  explicit UseLioCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<std_msgs::msg::Bool>(node)
  {
  }

  void setUseLio(bool flag)
  {
    msg_.data = flag;
  }
  bool getUseLio() const
  {
    return msg_.data;
  }
  void setZero() override{};
};

class BalanceCommandSender : public CommandSenderBase<std_msgs::msg::UInt8>
{
public:
  explicit BalanceCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<std_msgs::msg::UInt8>(node)
  {
  }

  void setBalanceMode(const int mode)
  {
    msg_.data = mode;
  }
  int getBalanceMode()
  {
    return msg_.data;
  }
  void setZero() override{};
};

class LegCommandSender : public CommandSenderBase<rm2_msgs::msg::LegCmd>
{
public:
  explicit LegCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<rm2_msgs::msg::LegCmd>(node)
  {
  }

  void setJump(bool jump)
  {
    msg_.jump = jump;
  }
  void setLgeLength(double length)
  {
    msg_.leg_length = length;
  }
  bool getJump()
  {
    return msg_.jump;
  }
  double getLgeLength()
  {
    return msg_.leg_length;
  }
  void setZero() override{};
};

class Vel3DCommandSender : public HeaderStampCommandSenderBase<geometry_msgs::msg::TwistStamped>
{
public:
  explicit Vel3DCommandSender(rclcpp::Node::SharedPtr node) 
  : HeaderStampCommandSenderBase(node)
  {
    if (!node->get_parameter("max_linear_x", max_linear_x_))
        RCLCPP_ERROR(node->get_logger(), "Max X linear velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_linear_y", max_linear_y_))
        RCLCPP_ERROR(node->get_logger(), "Max Y linear velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_linear_z", max_linear_z_))
        RCLCPP_ERROR(node->get_logger(), "Max Z linear velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_angular_x", max_angular_x_))
        RCLCPP_ERROR(node->get_logger(), "Max X angular velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_angular_y", max_angular_y_))
        RCLCPP_ERROR(node->get_logger(), "Max Y angular velocity no defined (namespace: %s)", node->get_namespace());
    if (!node->get_parameter("max_angular_z", max_angular_z_))
        RCLCPP_ERROR(node->get_logger(), "Max Z angular velocity no defined (namespace: %s)", node->get_namespace());
  }
  void setLinearVel(double scale_x, double scale_y, double scale_z)
  {
    msg_.twist.linear.x = max_linear_x_ * scale_x;
    msg_.twist.linear.y = max_linear_y_ * scale_y;
    msg_.twist.linear.z = max_linear_z_ * scale_z;
  }
  void setAngularVel(double scale_x, double scale_y, double scale_z)
  {
    msg_.twist.angular.x = max_angular_x_ * scale_x;
    msg_.twist.angular.y = max_angular_y_ * scale_y;
    msg_.twist.angular.z = max_angular_z_ * scale_z;
  }
  void setZero() override
  {
    msg_.twist.linear.x = 0.;
    msg_.twist.linear.y = 0.;
    msg_.twist.linear.z = 0.;
    msg_.twist.angular.x = 0.;
    msg_.twist.angular.y = 0.;
    msg_.twist.angular.z = 0.;
  }

private:
  double max_linear_x_{}, max_linear_y_{}, max_linear_z_{}, max_angular_x_{}, max_angular_y_{}, max_angular_z_{};
};

class JointPositionBinaryCommandSender : public CommandSenderBase<std_msgs::msg::Float64>
{
public:
  explicit JointPositionBinaryCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<std_msgs::msg::Float64>(node)
  {
    if (!node->get_parameter("on_pos", on_pos_) || !node->get_parameter("off_pos", off_pos_))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter on_pos or off_pos no defined (namespace: %s)", node->get_namespace());      
      return;
    }
  }
  void on()
  {
    msg_.data = on_pos_;
    state = true;
  }
  void off()
  {
    msg_.data = off_pos_;
    state = false;
  }
  void changePosition(double scale)
  {
    current_position_ = msg_.data;
    change_position_ = current_position_ + scale * per_change_position_;
    msg_.data = change_position_;
  }
  bool getState() const
  {
    return state;
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<std_msgs::msg::Float64>::sendCommand(time);
  }
  void setZero() override{};

private:
  bool state{};
  double on_pos_{}, off_pos_{}, current_position_{}, change_position_{}, per_change_position_{ 0.05 };
};

class CardCommandSender : public CommandSenderBase<std_msgs::msg::Float64>
{
public:
  explicit CardCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<std_msgs::msg::Float64>(node)
  {
    if (!node->get_parameter("long_pos", long_pos_) || !node->get_parameter("short_pos", short_pos_) || !node->get_parameter("off_pos", off_pos_))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter long_pos or short_pos or off_pos no defined (namespace: %s)", node->get_namespace());      
      return;
    }
  }
  void long_on()
  {
    msg_.data = long_pos_;
    state = true;
  }
  void short_on()
  {
    msg_.data = short_pos_;
    state = true;
  }
  void off()
  {
    msg_.data = off_pos_;
    state = false;
  }
  bool getState() const
  {
    return state;
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<std_msgs::msg::Float64>::sendCommand(time);
  }
  void setZero() override{};

private:
  bool state{};
  double long_pos_{}, short_pos_{}, off_pos_{};
};

class JointJogCommandSender : public CommandSenderBase<std_msgs::msg::Float64>
{
public:
  explicit JointJogCommandSender(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::JointState& joint_state)
    : CommandSenderBase<std_msgs::msg::Float64>(node), joint_state_(joint_state)
  {
    if (!node->get_parameter("joint", joint_))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter joint no defined (namespace: %s)", node->get_namespace());      
      return;
    }
    if (!node->get_parameter("step", step_))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter step no defined (namespace: %s)", node->get_namespace());      
      return;
    }
  }
  void reset()
  {
    auto i = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_);
    if (i != joint_state_.name.end())
      msg_.data = joint_state_.position[std::distance(joint_state_.name.begin(), i)];
    else
      msg_.data = NAN;
  }
  void plus()
  {
    if (msg_.data != NAN)
    {
      msg_.data += step_;
      sendCommand(rclcpp::Time());
    }
  }
  void minus()
  {
    if (msg_.data != NAN)
    {
      msg_.data -= step_;
      sendCommand(rclcpp::Time());
    }
  }
  const std::string& getJoint()
  {
    return joint_;
  }

private:
  std::string joint_{};
  const sensor_msgs::msg::JointState& joint_state_;
  double step_{};
};

class JointPointCommandSender : public CommandSenderBase<std_msgs::msg::Float64>
{
public:
  explicit JointPointCommandSender(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::JointState& joint_state)
    : CommandSenderBase<std_msgs::msg::Float64>(node), node_(node), joint_state_(joint_state)
  {
    if (!node_->get_parameter("joint", joint_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Parameter joint no defined (namespace: %s)", node_->get_namespace());      
      return;
    }
  }
  void setPoint(double point)
  {
    msg_.data = point;
  }
  int getIndex()
  {
    auto i = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_);
    if (i != joint_state_.name.end())
    {
      index_ = std::distance(joint_state_.name.begin(), i);
      return index_;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Can not find joint %s", joint_.c_str());
      return -1;
    }
  }
  void setZero() override{};

private:
  rclcpp::Node::SharedPtr node_; 
  std::string joint_{};
  int index_{};
  const sensor_msgs::msg::JointState& joint_state_;
};

class CameraSwitchCommandSender : public CommandSenderBase<std_msgs::msg::String>
{
public:
  explicit CameraSwitchCommandSender(rclcpp::Node::SharedPtr node) : CommandSenderBase<std_msgs::msg::String>(node)
  {
    if (!node->get_parameter("camera1_name", camera1_name_) || !node->get_parameter("camera2_name", camera2_name_))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter camera1_name or camera2_name no defined (namespace: %s)", node->get_namespace());      
      return;
    }
    msg_.data = camera1_name_;
  }
  void switchCamera()
  {
    msg_.data = msg_.data == camera1_name_ ? camera2_name_ : camera1_name_;
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<std_msgs::msg::String>::sendCommand(time);
  }
  void setZero() override{};

private:
  std::string camera1_name_{}, camera2_name_{};
};

class MultiDofCommandSender : public TimeStampCommandSenderBase<rm2_msgs::msg::MultiDofCmd>
{
public:
  explicit MultiDofCommandSender(rclcpp::Node::SharedPtr node) : TimeStampCommandSenderBase<rm2_msgs::msg::MultiDofCmd>(node)
  {
  }
  ~MultiDofCommandSender() = default;
  void setMode(int mode)
  {
    msg_.mode = mode;
  }
  int getMode()
  {
    return msg_.mode;
  }
  void setGroupValue(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y,
                     double angular_z)
  {
    msg_.linear.x = linear_x;
    msg_.linear.y = linear_y;
    msg_.linear.z = linear_z;
    msg_.angular.x = angular_x;
    msg_.angular.y = angular_y;
    msg_.angular.z = angular_z;
  }
  void setZero() override
  {
    msg_.linear.x = 0;
    msg_.linear.y = 0;
    msg_.linear.z = 0;
    msg_.angular.x = 0;
    msg_.angular.y = 0;
    msg_.angular.z = 0;
  }

private:
  rclcpp::Time time_;
};

class DoubleBarrelCommandSender
{
public:
  DoubleBarrelCommandSender(rclcpp::Node::SharedPtr node): node_(node)
  {
    auto shooter_ID1_node = node_->create_sub_node("shooter_ID1");
    shooter_ID1_cmd_sender_ = new ShooterCommandSender(shooter_ID1_node);
    auto shooter_ID2_node = node_->create_sub_node("shooter_ID2");
    shooter_ID2_cmd_sender_ = new ShooterCommandSender(shooter_ID2_node);
    auto barrel_node = node_->create_sub_node("barrel");
    barrel_command_sender_ = new rm2_common::JointPointCommandSender(barrel_node, joint_state_);

    try
    {
      barrel_node->get_parameter("is_double_barrel", is_double_barrel_);
      barrel_node->get_parameter("id1_point", id1_point_);
      barrel_node->get_parameter("id2_point", id2_point_);
      barrel_node->get_parameter("frequency_threshold", frequency_threshold_);
      barrel_node->get_parameter("check_launch_threshold", check_launch_threshold_);
      barrel_node->get_parameter("check_switch_threshold", check_switch_threshold_);
      barrel_node->get_parameter("ready_duration", ready_duration_);
      barrel_node->get_parameter("switching_duration", switching_duration_); 
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException& e)
    {
      RCLCPP_ERROR(barrel_node->get_logger(), "Parameter not declared: %s", e.what());
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& data) {
        this->jointStateCallback(data);});

    trigger_state_sub_ = node_->create_subscription<control_msgs::msg::JointControllerState>(
      "/controllers/shooter_controller/trigger/state",
      rclcpp::QoS(10),
      [this](const control_msgs::msg::JointControllerState::ConstSharedPtr& data) {
        this->triggerStateCallback(data);});

  }

  void updateGameRobotStatus(const rm2_msgs::msg::GameRobotStatus data)
  {
    shooter_ID1_cmd_sender_->updateGameRobotStatus(data);
    shooter_ID2_cmd_sender_->updateGameRobotStatus(data);
  }
  void updatePowerHeatData(const rm2_msgs::msg::PowerHeatData data)
  {
    shooter_ID1_cmd_sender_->heat_limit_->setCoolingHeatOfShooter(data);
    shooter_ID2_cmd_sender_->heat_limit_->setCoolingHeatOfShooter(data);
  }
  void updateRefereeStatus(bool status)
  {
    shooter_ID1_cmd_sender_->updateRefereeStatus(status);
    shooter_ID2_cmd_sender_->updateRefereeStatus(status);
  }
  void updateGimbalDesError(const rm2_msgs::msg::GimbalDesError& error)
  {
    shooter_ID1_cmd_sender_->updateGimbalDesError(error);
    shooter_ID2_cmd_sender_->updateGimbalDesError(error);
  }
  void updateTrackData(const rm2_msgs::msg::TrackData& data)
  {
    shooter_ID1_cmd_sender_->updateTrackData(data);
    shooter_ID2_cmd_sender_->updateTrackData(data);
  }
  void updateSuggestFireData(const std_msgs::msg::Bool& data)
  {
    shooter_ID1_cmd_sender_->updateSuggestFireData(data);
    shooter_ID2_cmd_sender_->updateSuggestFireData(data);
  }
  void updateShootBeforehandCmd(const rm2_msgs::msg::ShootBeforehandCmd& data)
  {
    shooter_ID1_cmd_sender_->updateShootBeforehandCmd(data);
    shooter_ID2_cmd_sender_->updateShootBeforehandCmd(data);
  }

  void setMode(int mode)
  {
    getBarrel()->setMode(mode);
  }
  void setZero()
  {
    getBarrel()->setZero();
  }
  void checkError(const rclcpp::Time& time)
  {
    getBarrel()->checkError(time);
  }
  void sendCommand(const rclcpp::Time& time)
  {
    if (checkSwitch())
      need_switch_ = true;
    if (need_switch_)
      switchBarrel();
    checklaunch();
    if (getBarrel()->getMsg()->mode == rm2_msgs::msg::ShootCmd::PUSH)
      last_push_time_ = time;
    getBarrel()->sendCommand(time);
  }
  void init()
  {
    rclcpp::Time time = node_->now();
    barrel_command_sender_->setPoint(id1_point_);
    shooter_ID1_cmd_sender_->setMode(rm2_msgs::msg::ShootCmd::STOP);
    shooter_ID2_cmd_sender_->setMode(rm2_msgs::msg::ShootCmd::STOP);
    barrel_command_sender_->sendCommand(time);
    shooter_ID1_cmd_sender_->sendCommand(time);
    shooter_ID2_cmd_sender_->sendCommand(time);
  }
  void setArmorType(uint8_t armor_type)
  {
    shooter_ID1_cmd_sender_->setArmorType(armor_type);
    shooter_ID2_cmd_sender_->setArmorType(armor_type);
  }
  void setShootFrequency(uint8_t mode)
  {
    getBarrel()->setShootFrequency(mode);
  }
  uint8_t getShootFrequency()
  {
    return getBarrel()->getShootFrequency();
  }
  double getSpeed()
  {
    return getBarrel()->getSpeed();
  }

private:
  ShooterCommandSender* getBarrel()
  {
    if (barrel_command_sender_->getMsg()->data == id1_point_)
      is_id1_ = true;
    else
      is_id1_ = false;
    return is_id1_ ? shooter_ID1_cmd_sender_ : shooter_ID2_cmd_sender_;
  }
  void switchBarrel()
  {
    rclcpp::Time time = node_->now();
    bool time_to_switch = (std::fmod(std::abs(trigger_error_), 2. * M_PI) < check_switch_threshold_);
    setMode(rm2_msgs::msg::ShootCmd::READY);
    if (time_to_switch || (time - last_push_time_).seconds() > ready_duration_)
    {
      barrel_command_sender_->getMsg()->data == id2_point_ ? barrel_command_sender_->setPoint(id1_point_) :
                                                             barrel_command_sender_->setPoint(id2_point_);
      barrel_command_sender_->sendCommand(time);
      last_switch_time_ = time;
      need_switch_ = false;
      is_switching_ = true;
    }
  }

  void checklaunch()
  {
    rclcpp::Time time = node_->now();
    if (is_switching_)
    {
      setMode(rm2_msgs::msg::ShootCmd::READY);
      if ((time - last_switch_time_).seconds() > switching_duration_ ||
          (std::abs(joint_state_.position[barrel_command_sender_->getIndex()] -
                    barrel_command_sender_->getMsg()->data) < check_launch_threshold_))
        is_switching_ = false;
    }
  }

  bool checkSwitch()
  {
    if (!is_double_barrel_)
      return false;
    if (shooter_ID1_cmd_sender_->heat_limit_->getCoolingLimit() == 0 ||
        shooter_ID2_cmd_sender_->heat_limit_->getCoolingLimit() == 0)
    {
      RCLCPP_WARN_ONCE(node_->get_logger(),"Can not get cooling limit");
      return false;
    }
    if (shooter_ID1_cmd_sender_->heat_limit_->getShootFrequency() < frequency_threshold_ ||
        shooter_ID2_cmd_sender_->heat_limit_->getShootFrequency() < frequency_threshold_)
    {
      if (getBarrel() == shooter_ID1_cmd_sender_)
        return getBarrel()->heat_limit_->getShootFrequency() < frequency_threshold_ &&
               shooter_ID2_cmd_sender_->heat_limit_->getShootFrequency() > frequency_threshold_;
      else
        return getBarrel()->heat_limit_->getShootFrequency() < frequency_threshold_ &&
               shooter_ID1_cmd_sender_->heat_limit_->getShootFrequency() > frequency_threshold_;
    }
    else
      return false;
  }
  void triggerStateCallback(const control_msgs::msg::JointControllerState::ConstSharedPtr& data)
  {
    trigger_error_ = data->error;
  }
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& data)
  {
    joint_state_ = *data;
  }
  rclcpp::Node::SharedPtr node_; 
  ShooterCommandSender* shooter_ID1_cmd_sender_;
  ShooterCommandSender* shooter_ID2_cmd_sender_;
  JointPointCommandSender* barrel_command_sender_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;  
  rclcpp::Subscription<control_msgs::msg::JointControllerState>::SharedPtr trigger_state_sub_;
  sensor_msgs::msg::JointState joint_state_;
  bool is_double_barrel_{ false }, need_switch_{ false }, is_switching_{ false };
  rclcpp::Time last_switch_time_, last_push_time_;
  double ready_duration_, switching_duration_;
  double trigger_error_;
  bool is_id1_{ false };
  double id1_point_, id2_point_;
  double frequency_threshold_;
  double check_launch_threshold_, check_switch_threshold_;
};

}  // namespace rm2_common
