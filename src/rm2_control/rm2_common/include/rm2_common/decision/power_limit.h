//
// Created by ch on 2025/9/21.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rm2_msgs/msg/chassis_cmd.hpp>
#include <rm2_msgs/msg/game_status.hpp>
#include <rm2_msgs/msg/game_robot_status.hpp>
#include <rm2_msgs/msg/power_heat_data.hpp>
#include <rm2_msgs/msg/power_management_sample_and_status_data.hpp>

namespace rm2_common
{
class PowerLimit
{
public:
  PowerLimit(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    if (!node_->get_parameter("safety_power", safety_power_))
      RCLCPP_ERROR(node_->get_logger(), "Safety power no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("capacitor_threshold", capacitor_threshold_))
      RCLCPP_ERROR(node_->get_logger(), "Capacitor threshold no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("disable_cap_gyro_threshold", disable_cap_gyro_threshold_))
      RCLCPP_ERROR(node_->get_logger(), "Disable cap gyro threshold no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("enable_cap_gyro_threshold", enable_cap_gyro_threshold_))
      RCLCPP_ERROR(node_->get_logger(), "Enable cap gyro threshold no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("charge_power", charge_power_))
      RCLCPP_ERROR(node_->get_logger(), "Charge power no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("extra_power", extra_power_))
      RCLCPP_ERROR(node_->get_logger(), "Extra power no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("burst_power", burst_power_))
      RCLCPP_ERROR(node_->get_logger(), "Burst power no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("standard_power", standard_power_))
      RCLCPP_ERROR(node_->get_logger(), "Standard power no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("max_power_limit", max_power_limit_))
      RCLCPP_ERROR(node_->get_logger(), "Max power limit no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("power_gain", power_gain_))
      RCLCPP_ERROR(node_->get_logger(), "Power gain no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("buffer_threshold", buffer_threshold_))
      RCLCPP_ERROR(node_->get_logger(), "Buffer threshold no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("is_new_capacitor", is_new_capacitor_))
      RCLCPP_ERROR(node_->get_logger(), "Is new capacitor no defined (namespace: %s)", node_->get_namespace());
    if (!node_->get_parameter("total_burst_time", total_burst_time_))
      RCLCPP_ERROR(node_->get_logger(), "Total burst time no defined (namespace: %s)", node_->get_namespace());
  }
  typedef enum
  {
    CHARGE = 0,
    BURST = 1,
    NORMAL = 2,
    ALLOFF = 3,
    TEST = 4,
  } Mode;

  void updateSafetyPower(int safety_power)
  {
    if (safety_power > 0)
      safety_power_ = safety_power;
    RCLCPP_INFO(node_->get_logger(),"update safety power: %d", safety_power);
  }
  void updateState(uint8_t state)
  {
    if (!capacitor_is_on_)
      expect_state_ = ALLOFF;
    else
      expect_state_ = state;
  }
  void updateCapSwitchState(bool state)
  {
    capacitor_is_on_ = state;
  }
  void setGameRobotData(const rm2_msgs::msg::GameRobotStatus data)
  {
    robot_id_ = data.robot_id;
    chassis_power_limit_ = data.chassis_power_limit;
  }
  void setChassisPowerBuffer(const rm2_msgs::msg::PowerHeatData data)
  {
    chassis_power_buffer_ = data.chassis_power_buffer;
    power_buffer_threshold_ = chassis_power_buffer_ * 0.8;
  }
  void setCapacityData(const rm2_msgs::msg::PowerManagementSampleAndStatusData data)
  {
    capacity_is_online_ = node_->now() - data.stamp < rclcpp::Duration::from_seconds(0.3);
    cap_energy_ = data.capacity_remain_charge;
    cap_state_ = data.state_machine_running_state;
  }
  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }
  void setStartBurstTime(const rclcpp::Time start_burst_time)
  {
    start_burst_time_ = start_burst_time;
  }
  rclcpp::Time getStartBurstTime() const
  {
    return start_burst_time_;
  }
  uint8_t getState()
  {
    return expect_state_;
  }
  void setGyroPower(rm2_msgs::msg::ChassisCmd& chassis_cmd)
  {
    if (!allow_gyro_cap_ && cap_energy_ >= enable_cap_gyro_threshold_)
      allow_gyro_cap_ = true;
    if (allow_gyro_cap_ && cap_energy_ <= disable_cap_gyro_threshold_)
      allow_gyro_cap_ = false;
    if (allow_gyro_cap_ && chassis_power_limit_ < 80)
      chassis_cmd.power_limit = chassis_power_limit_ + extra_power_;
    else
      expect_state_ = NORMAL;
  }
  void setLimitPower(rm2_msgs::msg::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    if (robot_id_ == rm2_msgs::msg::GameRobotStatus::BLUE_ENGINEER || robot_id_ == rm2_msgs::msg::GameRobotStatus::RED_ENGINEER)
      chassis_cmd.power_limit = 400;
    else
    {  // standard and hero
      if (referee_is_online_)
      {
        if (capacity_is_online_ && expect_state_ != ALLOFF)
        {
          if (chassis_power_limit_ > burst_power_)
            chassis_cmd.power_limit = burst_power_;
          else
          {
            switch (is_new_capacitor_ ? expect_state_ : cap_state_)
            {
              case NORMAL:
                normal(chassis_cmd);
                break;
              case BURST:
                burst(chassis_cmd, is_gyro);
                break;
              case CHARGE:
                charge(chassis_cmd);
                break;
              default:
                zero(chassis_cmd);
                break;
            }
          }
        }
        else
          normal(chassis_cmd);
      }
      else
        chassis_cmd.power_limit = safety_power_;
    }
  }

private:
  void charge(rm2_msgs::msg::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = chassis_power_limit_ * 0.70;
  }
  void normal(rm2_msgs::msg::ChassisCmd& chassis_cmd)
  {
    double buffer_energy_error = chassis_power_buffer_ - buffer_threshold_;
    double plus_power = buffer_energy_error * power_gain_;
    chassis_cmd.power_limit = chassis_power_limit_ + plus_power;
    // TODO:Add protection when buffer<5
    if (chassis_cmd.power_limit > max_power_limit_)
      chassis_cmd.power_limit = max_power_limit_;
  }
  void zero(rm2_msgs::msg::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = 0.0;
  }
  void burst(rm2_msgs::msg::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    if (cap_state_ != ALLOFF && cap_energy_ > capacitor_threshold_ && chassis_power_buffer_ > power_buffer_threshold_)
    {
      if (is_gyro)
        setGyroPower(chassis_cmd);
      else if (node_->now() - start_burst_time_ < rclcpp::Duration::from_seconds(total_burst_time_))
        chassis_cmd.power_limit = burst_power_;
      else
        chassis_cmd.power_limit = standard_power_;
    }
    else
      expect_state_ = NORMAL;
  }

  rclcpp::Node::SharedPtr node_;  
  int chassis_power_buffer_;
  int robot_id_, chassis_power_limit_;
  int max_power_limit_{ 70 };
  float cap_energy_;
  double safety_power_{};
  double capacitor_threshold_{};
  double power_buffer_threshold_{ 50.0 };
  double enable_cap_gyro_threshold_{}, disable_cap_gyro_threshold_{};
  double charge_power_{}, extra_power_{}, burst_power_{}, standard_power_{};
  double buffer_threshold_{};
  double power_gain_{};
  bool is_new_capacitor_{ false };
  uint8_t expect_state_{}, cap_state_{};
  bool capacitor_is_on_{ true };
  bool allow_gyro_cap_{ false };

  rclcpp::Time start_burst_time_{};
  int total_burst_time_{};

  bool referee_is_online_{ false };
  bool capacity_is_online_{ false };
};
}  // namespace rm2_common
