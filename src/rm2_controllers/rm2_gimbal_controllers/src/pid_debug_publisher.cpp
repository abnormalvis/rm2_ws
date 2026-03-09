#include "rm2_gimbal_controllers/pid_debug_publisher.h"

#include <rclcpp/qos.hpp>

namespace rm2_gimbal_controllers {

PIDDebugPublisher::PIDDebugPublisher(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
    : node_(std::move(node)) {}

void PIDDebugPublisher::setPublishRate(double publish_rate) {
  publish_rate_ = publish_rate;
}

void PIDDebugPublisher::reset(const rclcpp::Time &time) {
  for (auto &[topic, last_publish_time] : last_publish_times_) {
    (void)topic;
    last_publish_time = time;
  }
}

auto PIDDebugPublisher::getOrCreatePublisher(const std::string &topic)
    -> Publisher::SharedPtr {
  auto it = publishers_.find(topic);
  if (it != publishers_.end()) {
    return it->second;
  }

  auto publisher =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic,
                                                                  rclcpp::QoS(10));
  publishers_.emplace(topic, publisher);
  return publisher;
}

bool PIDDebugPublisher::shouldPublish(const std::string &topic,
                                      const rclcpp::Time &time) const {
  if (!(publish_rate_ > 0.0)) {
    return false;
  }

  const auto it = last_publish_times_.find(topic);
  if (it == last_publish_times_.end()) {
    return true;
  }

  return (it->second + rclcpp::Duration::from_seconds(1.0 / publish_rate_)) <
         time;
}

void PIDDebugPublisher::publish(const std::string &topic,
                                const std::string &frame_id,
                                const rclcpp::Time &time,
                                double position_error,
                                double outer_loop_velocity_reference,
                                double inner_loop_velocity_error) {
  if (!shouldPublish(topic, time)) {
    return;
  }

  auto publisher = getOrCreatePublisher(topic);
  if (!publisher) {
    return;
  }

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;
  msg.vector.x = position_error;
  msg.vector.y = outer_loop_velocity_reference;
  msg.vector.z = inner_loop_velocity_error;
  publisher->publish(msg);

  last_publish_times_[topic] = time;
}

} // namespace rm2_gimbal_controllers