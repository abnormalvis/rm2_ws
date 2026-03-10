#pragma once

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

namespace rm2_gimbal_controllers {

class PIDDebugPublisher {
public:
  explicit PIDDebugPublisher(
      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

  void setPublishRate(double publish_rate);
  void reset(const rclcpp::Time &time);
  void publish(const std::string &topic, const std::string &frame_id,
               const rclcpp::Time &time, double position_error,
               double outer_loop_velocity_reference,
               double inner_loop_velocity_error);

private:
  using Publisher = rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>;

  Publisher::SharedPtr getOrCreatePublisher(const std::string &topic);
  bool shouldPublish(const std::string &topic, const rclcpp::Time &time) const;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::map<std::string, Publisher::SharedPtr> publishers_;
  std::map<std::string, rclcpp::Time> last_publish_times_;
  double publish_rate_ = 100.0;
};

} // namespace rm2_gimbal_controllers