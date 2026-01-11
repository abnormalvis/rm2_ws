//
// Created by ch on 2025/9/21.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <rm2_msgs/msg/game_robot_status.hpp>
#include <rm2_msgs/msg/power_heat_data.hpp>
#include <rm2_msgs/msg/shoot_cmd.hpp>
#include <rm2_msgs/msg/local_heat_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rm2_common
{
class HeatLimit
{
public:
  HeatLimit(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    if (!node_->get_parameter("low_shoot_frequency", low_shoot_frequency_))
      RCLCPP_ERROR(node_->get_logger(), "Low shoot frequency no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("high_shoot_frequency", high_shoot_frequency_))
      RCLCPP_ERROR(node_->get_logger(), "High shoot frequency no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("burst_shoot_frequency", burst_shoot_frequency_))
      RCLCPP_ERROR(node_->get_logger(), "Burst shoot frequency no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("minimal_shoot_frequency", minimal_shoot_frequency_))
      RCLCPP_ERROR(node_->get_logger(), "Minimal shoot frequency no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("safe_shoot_frequency", safe_shoot_frequency_))
      RCLCPP_ERROR(node_->get_logger(), "Safe shoot frequency no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("heat_coeff", heat_coeff_))
      RCLCPP_ERROR(node_->get_logger(), "Heat coeff no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("type", type_))
      RCLCPP_ERROR(node_->get_logger(), "Shooter type no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("local_heat_protect_threshold", heat_protect_threshold_))
      RCLCPP_ERROR(node_->get_logger(), "Local heat protect threshold no defined (namespace: %s)", node_->get_namespace());
    node_->declare_parameter("use_local_heat",true);
    node_->get_parameter("use_local_heat", use_local_heat_);
    if (type_ == "ID1_42MM")
      bullet_heat_ = 100.;
    else
      bullet_heat_ = 10.;
    local_heat_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/local_heat_state/local_cooling_heat",
      rclcpp::QoS(10));
    shoot_state_sub_ = node->create_subscription<rm2_msgs::msg::LocalHeatState>(
      "/local_heat_state/shooter_state",
      rclcpp::QoS(50),
      [this](const rm2_msgs::msg::LocalHeatState::ConstSharedPtr& msg) {
        this->heatCB(msg);});
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), 
      [this](void) {
        this->timerCB();});

  }

  typedef enum
  {
    LOW = 0,
    HIGH = 1,
    BURST = 2,
    MINIMAL = 3
  } ShootHz;

  void heatCB(const rm2_msgs::msg::LocalHeatState::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(heat_mutex_);
    if (msg->has_shoot && last_shoot_state_ != msg->has_shoot)
      local_shooter_cooling_heat_ += bullet_heat_;
    last_shoot_state_ = msg->has_shoot;
  }

  void timerCB()
  {
    std::lock_guard<std::mutex> lock(heat_mutex_);
    if (local_shooter_cooling_heat_ > 0.0)
      local_shooter_cooling_heat_ -= shooter_cooling_rate_ * 0.1;
    if (local_shooter_cooling_heat_ < 0.0)
      local_shooter_cooling_heat_ = 0.0;
    std_msgs::msg::Float64 msg;
    msg.data = local_shooter_cooling_heat_;
    local_heat_pub_->publish(msg);
  }

  void setStatusOfShooter(const rm2_msgs::msg::GameRobotStatus data)
  {
    shooter_cooling_limit_ = data.shooter_cooling_limit - heat_protect_threshold_;
    shooter_cooling_rate_ = data.shooter_cooling_rate;
  }

  void setCoolingHeatOfShooter(const rm2_msgs::msg::PowerHeatData data)
  {
    if (type_ == "ID1_17MM")
    {
      shooter_cooling_heat_ = data.shooter_id_1_17_mm_cooling_heat;
    }
    else if (type_ == "ID2_17MM")
    {
      shooter_cooling_heat_ = data.shooter_id_2_17_mm_cooling_heat;
    }
    else if (type_ == "ID1_42MM")
    {
      shooter_cooling_heat_ = data.shooter_id_1_42_mm_cooling_heat;
    }
  }

  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }

  double getShootFrequency() const
  {
    std::lock_guard<std::mutex> lock(heat_mutex_);
    if (state_ == BURST)
      return shoot_frequency_;
    double shooter_cooling_heat =
        (use_local_heat_ || !referee_is_online_) ? local_shooter_cooling_heat_ : shooter_cooling_heat_;
    if (shooter_cooling_limit_ - shooter_cooling_heat < bullet_heat_)
      return 0.0;
    else if (shooter_cooling_limit_ - shooter_cooling_heat == bullet_heat_)
      return shooter_cooling_rate_ / bullet_heat_;
    else if (shooter_cooling_limit_ - shooter_cooling_heat <= bullet_heat_ * heat_coeff_)
      return (shooter_cooling_limit_ - shooter_cooling_heat) / (bullet_heat_ * heat_coeff_) *
                 (shoot_frequency_ - shooter_cooling_rate_ / bullet_heat_) +
             shooter_cooling_rate_ / bullet_heat_;
    else
      return shoot_frequency_;
  }

  int getSpeedLimit()
  {
    updateExpectShootFrequency();
    if (type_ == "ID1_17MM")
      return rm2_msgs::msg::ShootCmd::SPEED_30M_PER_SECOND;
    else if (type_ == "ID2_17MM")
      return rm2_msgs::msg::ShootCmd::SPEED_30M_PER_SECOND;
    else if (type_ == "ID1_42MM")
      return rm2_msgs::msg::ShootCmd::SPEED_16M_PER_SECOND;
    return -1;  // TODO unsafe!
  }

  int getCoolingLimit()
  {
    return shooter_cooling_limit_;
  }

  int getCoolingHeat()
  {
    return shooter_cooling_heat_;
  }

  void setShootFrequency(uint8_t mode)
  {
    state_ = mode;
  }

  bool getShootFrequencyMode() const
  {
    return state_;
  }

private:
  void updateExpectShootFrequency()
  {
    if (state_ == HeatLimit::BURST)
    {
      shoot_frequency_ = burst_shoot_frequency_;
      burst_flag_ = true;
    }
    else if (state_ == HeatLimit::LOW)
    {
      shoot_frequency_ = low_shoot_frequency_;
      burst_flag_ = false;
    }
    else if (state_ == HeatLimit::HIGH)
    {
      shoot_frequency_ = high_shoot_frequency_;
      burst_flag_ = false;
    }
    else if (state_ == HeatLimit::MINIMAL)
    {
      shoot_frequency_ = minimal_shoot_frequency_;
      burst_flag_ = false;
    }
    else
    {
      shoot_frequency_ = safe_shoot_frequency_;
      burst_flag_ = false;
    }
  }

  rclcpp::Node::SharedPtr node_;
  uint8_t state_{};
  std::string type_{};
  bool burst_flag_ = false;
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{}, shoot_frequency_{}, low_shoot_frequency_{},
      high_shoot_frequency_{}, burst_shoot_frequency_{}, minimal_shoot_frequency_{};

  bool referee_is_online_, use_local_heat_, last_shoot_state_{};
  int shooter_cooling_limit_, shooter_cooling_rate_, shooter_cooling_heat_;
  double local_shooter_cooling_heat_{}, heat_protect_threshold_{};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr local_heat_pub_;
  rclcpp::Subscription<rm2_msgs::msg::LocalHeatState>::SharedPtr shoot_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex heat_mutex_;
};

}  // namespace rm2_common
