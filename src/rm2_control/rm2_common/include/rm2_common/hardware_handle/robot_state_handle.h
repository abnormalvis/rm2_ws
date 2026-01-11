//
// Created by ch on 2025/10/5.
//

#pragma once

#include <stdexcept>
#include <utility>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include "rm2_common/tf_rt_broadcaster.h"

namespace rm2_control
{
class RobotStateHandle
{
public:
  RobotStateHandle() = default;
  RobotStateHandle(std::string name, tf2_ros::Buffer* buffer) : name_(std::move(name)), buffer_(buffer)
  {
    if (!buffer)
      throw std::runtime_error("Cannot create handle '" + name +
                               "'. Tf Buffer data pointer is null.");
  };

  geometry_msgs::msg::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const rclcpp::Time& time)
  {
    return buffer_->lookupTransform(target_frame, source_frame, time);
  }

  bool setTransform(const geometry_msgs::msg::TransformStamped& transform, const std::string& authority,
                    bool is_static = false) const
  {
    return buffer_->setTransform(transform, authority, is_static);
  }

  bool setTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, const std::string& authority,
                    bool is_static = false) const
  {
    for (const auto& transform : transforms)
      buffer_->setTransform(transform, authority, is_static);
    return true;
  }

  std::string getName() const
  {
    return name_;
  }

private:
  std::string name_;
  tf2_ros::Buffer* buffer_{};
};

}  // namespace rm2_control
