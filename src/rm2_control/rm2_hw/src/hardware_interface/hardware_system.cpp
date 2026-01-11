//
// Created by ch on 2025/10/25.
//

#include "rm2_hw/hardware_interface/hardware_system.h"
#include "rm2_hw/hardware_interface/can_bus.h"
#include "rm2_hw/hardware_interface/types.h"
#include <algorithm>
#include <array>
#include <hardware_interface/system_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <fmt/core.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

namespace rm2_hw
{
CallbackReturn RmHardwareSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  if (auto locked_executor = params.executor.lock())
  {
    node_ = std::make_shared<rclcpp::Node>("hw_node");
    locked_executor->add_node(node_->get_node_base_interface());  
    node_->declare_parameter("thread_priority",0);
    node_->declare_parameter("actuator_coefficient_names",std::vector<std::string>{});
    node_->declare_parameter("actuators_names",std::vector<std::string>{});
    node_->declare_parameter("bus",std::vector<std::string>{});
    if (node_->get_parameter("thread_priority").as_int() == 0)
      RCLCPP_ERROR(node_->get_logger(),"No thread_priority specified");
    else 
      setCanBusThreadPriority(node_->get_parameter("thread_priority").as_int());
    // Parse actuator coefficient specified by user (stored on ROS2 parameter server)
    if (node_->get_parameter("actuator_coefficient_names").as_string_array().empty())
      RCLCPP_WARN(node_->get_logger(),"No actuator coefficient specified");
    else if (!parseActCoeffs(node_->get_parameter("actuator_coefficient_names").as_string_array()))
      return CallbackReturn::ERROR;
    // Parse actuator specified by user (stored on ROS2 parameter server)
    if (node_->get_parameter("actuators_names").as_string_array().empty())
      RCLCPP_WARN(node_->get_logger(),"No actuator specified");
    else if (!parseActData(node_->get_parameter("actuators_names").as_string_array()))
      return CallbackReturn::ERROR;
    // CAN Bus
    if (node_->get_parameter("bus").as_string_array().empty())
      RCLCPP_WARN(node_->get_logger(),"No bus specified");
    else if (node_->get_parameter("bus").get_type() == rclcpp::PARAMETER_STRING_ARRAY)
    {
      std::vector<std::string> bus_names = node_->get_parameter("bus").as_string_array();
      for (auto& bus_name : bus_names)
      {
        if (bus_name.find("can") != std::string::npos)
          can_buses_.emplace_back(new CanBus(node_, 
                                                  bus_name,
                                         CanDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                              .id2act_data_ = &bus_id2act_data_[bus_name] },
                                  thread_priority_));
        else
          RCLCPP_ERROR_STREAM(node_->get_logger(),"Unknown bus: " << bus_name);
      }
    } 

    // Initialize transmission
    if (!setupTransmission())
    { 
      RCLCPP_ERROR(node_->get_logger(),"Error occurred while setting up transmission");
      return CallbackReturn::ERROR;
    }
    pub_node_ = std::make_shared<rclcpp::Node>("actuator_state_pub");
    locked_executor->add_node(pub_node_->get_node_base_interface());
    actuator_state_pub_ =
      node_->create_publisher<rm2_msgs::msg::ActuatorState>("actuator_state", rclcpp::SystemDefaultsQoS());
    actuator_state_pub_rt_ =
      std::make_shared<realtime_tools::RealtimePublisher<rm2_msgs::msg::ActuatorState>>(actuator_state_pub_);
    return CallbackReturn::SUCCESS;  
  }
  return CallbackReturn::ERROR;
}

CallbackReturn RmHardwareSystem::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  RCLCPP_INFO(node_->get_logger(), "on_configure called");
  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmHardwareSystem::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_activate(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  RCLCPP_INFO(node_->get_logger(), "on_activate called");
  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmHardwareSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  RCLCPP_INFO(node_->get_logger(), "on_deactivate called");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmHardwareSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto& joint : info_.joints)
  {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface = joint_name2joint_interfaces_.find(joint.name);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_interface->second.state_[0]);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_interface->second.state_[1]);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_interface->second.state_[2]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmHardwareSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto& joint : info_.joints)
  {
    auto joint_interface = joint_name2joint_interfaces_.find(joint.name);

    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_interface->second.command_[2]);
  }
  return command_interfaces;
}

hardware_interface::return_type RmHardwareSystem::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{ 
  for (auto& bus : can_buses_)
    bus->read(time);
  for (auto& id2act_datas : bus_id2act_data_)
    for (auto& act_data : id2act_datas.second)
    {
      try
      {  // Duration will be out of dual 32-bit range while motor failure
        act_data.second.halted = (time - act_data.second.stamp).seconds() > 0.1 || act_data.second.temp > 99;
      }
      catch (std::runtime_error& ex)
      {
      }
      if (act_data.second.halted)
      {
        act_data.second.seq = 0;
        act_data.second.vel = 0;
        act_data.second.effort = 0;
        act_data.second.calibrated = false;  // set the actuator no calibrated
      }

      auto actuator_interface = actuator_name2actuator_interfaces_.find(act_data.second.name);
      if (actuator_interface != actuator_name2actuator_interfaces_.end())
      {
        actuator_interface->second.state_[0] = act_data.second.pos;

        actuator_interface->second.state_[1] = act_data.second.vel;
        actuator_interface->second.state_[2] = act_data.second.effort;
      }
      else
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), "(read)Actuator interface not found for: " << act_data.second.name);
    }

  if (is_actuator_specified_)
  {
    // actuator: state -> transmission
    for(auto& actuator_interface : actuator_name2actuator_interfaces_)
      actuator_interface.second.transmissionPassthrough_ = actuator_interface.second.state_;
    // transmission: actuator -> joint
    for(auto& transmission : transmissions_)
      transmission->actuator_to_joint();
    // joint: transmission -> state
    for(auto& joint_interface : joint_name2joint_interfaces_)
    {
      joint_interface.second.state_ = joint_interface.second.transmissionPassthrough_;
    }
      
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmHardwareSystem::write(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  if (is_actuator_specified_)
  {
    // joint: command -> transmission
    for(auto& joint_interface : joint_name2joint_interfaces_)
      joint_interface.second.transmissionPassthrough_ = joint_interface.second.command_;
    // transmission: joint -> actuator
    for(auto& transmission : transmissions_)
      transmission->joint_to_actuator();
    // actuator: transmission -> command
    for(auto& actuator_interface : actuator_name2actuator_interfaces_)
      actuator_interface.second.command_ = actuator_interface.second.transmissionPassthrough_;

    for (auto& id2act_datas : bus_id2act_data_)
      for (auto& act_data : id2act_datas.second)
      {
        auto actuator_interface = actuator_name2actuator_interfaces_.find(act_data.second.name);
        if (actuator_interface != actuator_name2actuator_interfaces_.end())
        {
          act_data.second.exe_effort = actuator_interface->second.command_[2];
        }
        else
        {
          RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), "(write)Actuator interface not found for: " << act_data.second.name);
        }
      }
  }
  for (auto& bus : can_buses_)
    bus->write();
  publishActuatorState(time);
  return hardware_interface::return_type::OK;
}

void RmHardwareSystem::publishActuatorState(const rclcpp::Time& /*time*/)
{
  if (actuator_state_pub_rt_->trylock())
  {
    rm2_msgs::msg::ActuatorState actuator_state;
    for (const auto& id2act_datas : bus_id2act_data_)
      for (const auto& act_data : id2act_datas.second)
      {
        actuator_state.stamp.push_back(act_data.second.stamp);
        actuator_state.name.push_back(act_data.second.name);
        actuator_state.type.push_back(act_data.second.type);
        actuator_state.bus.push_back(id2act_datas.first);
        actuator_state.id.push_back(act_data.first);
        actuator_state.halted.push_back(act_data.second.halted);
        actuator_state.need_calibration.push_back(act_data.second.need_calibration);
        actuator_state.calibrated.push_back(act_data.second.calibrated);
        actuator_state.calibration_reading.push_back(act_data.second.calibration_reading);
        actuator_state.position_raw.push_back(act_data.second.q_raw);
        actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
        actuator_state.temperature.push_back(act_data.second.temp);
        actuator_state.circle.push_back(act_data.second.q_circle);
        actuator_state.last_position_raw.push_back(act_data.second.q_last);
        actuator_state.frequency.push_back(act_data.second.frequency);
        actuator_state.position.push_back(act_data.second.pos);
        actuator_state.velocity.push_back(act_data.second.vel);
        actuator_state.effort.push_back(act_data.second.effort);
        actuator_state.commanded_effort.push_back(act_data.second.cmd_effort);
        actuator_state.executed_effort.push_back(act_data.second.exe_effort);
        actuator_state.offset.push_back(act_data.second.offset);
      }
    actuator_state_pub_rt_->msg_ = actuator_state;
    actuator_state_pub_rt_->unlockAndPublish();
  }
}

void RmHardwareSystem::setCanBusThreadPriority(int thread_priority)
{
  thread_priority_ = thread_priority;
}
}  // namespace rm2_hw

PLUGINLIB_EXPORT_CLASS(rm2_hw::RmHardwareSystem, hardware_interface::SystemInterface)
