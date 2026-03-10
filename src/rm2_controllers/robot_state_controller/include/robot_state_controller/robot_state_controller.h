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

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_state_controller
{

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, std::string p_root, std::string p_tip)
    : segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip))
  {
  }

  KDL::Segment segment{};
  std::string root, tip;
};

class RobotStateController : public controller_interface::ControllerInterface
{
public:
  RobotStateController() = default;
  ~RobotStateController() override = default;

  // Lifecycle callbacks
  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  void tfSubCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void staticSubCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  urdf::Model model_{};
  std::map<std::string, urdf::JointMimicSharedPtr> mimic_{};
  unsigned int num_hw_joints_{};
  bool use_tf_static_{};
  bool ignore_timestamp_{};
  double publish_rate_{};
  rclcpp::Time last_update_;
  rclcpp::Time last_publish_time_;

  std::map<std::string, double> joint_positions_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  
  realtime_tools::RealtimeBuffer<tf2_msgs::msg::TFMessage> tf_msg_;
  realtime_tools::RealtimeBuffer<tf2_msgs::msg::TFMessage> tf_static_msg_;
};

}  // namespace robot_state_controller
