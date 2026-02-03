/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/3/21.
// Ported to ROS2 by idris on 2026/2/3
//

#include "robot_state_controller/robot_state_controller.h"

#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robot_state_controller
{

controller_interface::CallbackReturn RobotStateController::on_init()
{
  try {
    // Declare parameters
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("use_tf_static", true);
    auto_declare<bool>("ignore_timestamp", false);
    auto_declare<double>("buffer_duration", 10.0);
    auto_declare<std::string>("robot_description", "");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
RobotStateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration 
RobotStateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn RobotStateController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Read parameters
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  use_tf_static_ = get_node()->get_parameter("use_tf_static").as_bool();
  ignore_timestamp_ = get_node()->get_parameter("ignore_timestamp").as_bool();
  double buffer_duration = get_node()->get_parameter("buffer_duration").as_double();
  
  // Initialize TF buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  
  // Initialize broadcasters
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node());
  
  // Subscribe to TF topics
  tf_sub_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      std::bind(&RobotStateController::tfSubCallback, this, std::placeholders::_1));
      
  tf_static_sub_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local(),
      std::bind(&RobotStateController::staticSubCallback, this, std::placeholders::_1));
  
  // Load URDF
  std::string robot_description = get_node()->get_parameter("robot_description").as_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  if (!model_.initString(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF from robot_description");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Build KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to extract KDL tree from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  addChildren(tree.getRootSegment());
  
  // Extract mimic joints
  for (const auto& joint : model_.joints_) {
    if (joint.second->mimic) {
      mimic_.insert(std::make_pair(joint.first, joint.second->mimic));
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Robot state controller configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStateController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Initialize joint positions map from state interfaces
  joint_positions_.clear();
  
  for (const auto& interface : state_interfaces_) {
    if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      std::string joint_name = interface.get_prefix_name();
      joint_positions_[joint_name] = 0.0;
      RCLCPP_INFO(get_node()->get_logger(), "Added joint state: %s", joint_name.c_str());
    }
  }
  
  num_hw_joints_ = joint_positions_.size();
  RCLCPP_INFO(get_node()->get_logger(), "Robot state controller activated with %u joints", num_hw_joints_);
  
  last_update_ = get_node()->now();
  last_publish_time_ = get_node()->now();
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotStateController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  joint_positions_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

std::string stripSlash(const std::string& in)
{
  if (!in.empty() && in[0] == '/') {
    return in.substr(1);
  }
  return in;
}

controller_interface::return_type RobotStateController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  if (last_update_ > time) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Moved backwards in time (probably because ROS clock was reset), clear all tf buffer!");
    tf_buffer_->clear();
  }
  last_update_ = time;
  
  // Update joint positions from state interfaces
  for (const auto& interface : state_interfaces_) {
    if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      std::string joint_name = interface.get_prefix_name();
      auto pos_opt = interface.get_optional<double>();
      if (pos_opt) {
        joint_positions_[joint_name] = *pos_opt;
      }
    }
  }
  
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms, tf_static_transforms;
  geometry_msgs::msg::TransformStamped tf_transform;
  
  // Loop over all float segments
  for (auto& item : segments_) {
    auto jnt_iter = joint_positions_.find(item.first);
    auto mimic_iter = mimic_.find(item.first);
    
    if (jnt_iter != joint_positions_.end()) {
      tf_transform = tf2::kdlToTransform(item.second.segment.pose(jnt_iter->second));
    } else if (mimic_iter != mimic_.end()) {
      auto mimic_joint_iter = joint_positions_.find(mimic_iter->second->joint_name);
      if (mimic_joint_iter != joint_positions_.end()) {
        double mimic_pos = mimic_joint_iter->second * mimic_iter->second->multiplier + 
                          mimic_iter->second->offset;
        tf_transform = tf2::kdlToTransform(item.second.segment.pose(mimic_pos));
      } else {
        continue;
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 10000,
                          "Joint state with name: \"%s\" was received but not found in URDF", 
                          item.first.c_str());
      continue;
    }
    
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(item.second.root);
    tf_transform.child_frame_id = stripSlash(item.second.tip);
    tf_transforms.push_back(tf_transform);
  }
  
  // Loop over all fixed segments
  for (const auto& seg : segments_fixed_) {
    tf_transform = tf2::kdlToTransform(seg.second.segment.pose(0));
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(seg.second.root);
    tf_transform.child_frame_id = stripSlash(seg.second.tip);
    tf_static_transforms.push_back(tf_transform);
  }
  
  // Set transforms in buffer
  for (const auto& tran : tf_transforms) {
    tf_buffer_->setTransform(tran, "robot_state_controller", false);
  }
  for (const auto& tran : tf_static_transforms) {
    tf_buffer_->setTransform(tran, "robot_state_controller", true);
  }
  
  // Publish transforms if rate is exceeded
  if (publish_rate_ > 0.0 && 
      last_publish_time_ + rclcpp::Duration::from_seconds(1.0 / publish_rate_) <= time) {
    tf_broadcaster_->sendTransform(tf_transforms);
    if (use_tf_static_) {
      static_tf_broadcaster_->sendTransform(tf_static_transforms);
    } else {
      tf_broadcaster_->sendTransform(tf_static_transforms);
    }
    last_publish_time_ = time;
  }
  
  tf_transforms.clear();
  tf_static_transforms.clear();
  
  // Process subscribed TF messages
  for (const auto& item : tf_msg_.readFromRT()->transforms) {
    try {
      if (item.header.stamp != 
          tf_buffer_->lookupTransform(item.child_frame_id, item.header.frame_id, 
                                     tf2::TimePointZero).header.stamp) {
        tf_transforms.push_back(item);
      }
    } catch (tf2::TransformException& ex) {
      tf_transforms.push_back(item);
    }
  }
  
  for (const auto& item : tf_static_msg_.readFromRT()->transforms) {
    try {
      if (item.header.stamp != 
          tf_buffer_->lookupTransform(item.child_frame_id, item.header.frame_id, 
                                     tf2::TimePointZero).header.stamp) {
        tf_static_transforms.push_back(item);
      }
    } catch (tf2::TransformException& ex) {
      tf_static_transforms.push_back(item);
    }
  }
  
  for (const auto& tran : tf_transforms) {
    tf_buffer_->setTransform(tran, "outside", false);
  }
  for (const auto& tran : tf_static_transforms) {
    tf_buffer_->setTransform(tran, "outside", true);
  }
  
  return controller_interface::return_type::OK;
}

void RobotStateController::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();
  const std::vector<KDL::SegmentMap::const_iterator>& children = 
      GetTreeElementChildren(segment->second);
      
  for (auto i : children) {
    const KDL::Segment& child = GetTreeElementSegment(i->second);
    SegmentPair s(GetTreeElementSegment(i->second), root, child.getName());
    
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) &&
          model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        RCLCPP_INFO(get_node()->get_logger(),
            "Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info",
            root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(std::make_pair(child.getJoint().getName(), s));
        RCLCPP_DEBUG(get_node()->get_logger(), 
                    "Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    } else {
      segments_.insert(std::make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(get_node()->get_logger(), 
                  "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(i);
  }
}

void RobotStateController::tfSubCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  tf_msg_.writeFromNonRT(*msg);
}

void RobotStateController::staticSubCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  tf_static_msg_.writeFromNonRT(*msg);
}

}  // namespace robot_state_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_state_controller::RobotStateController, 
                      controller_interface::ControllerInterface)
