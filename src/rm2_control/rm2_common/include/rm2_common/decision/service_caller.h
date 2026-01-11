//
// Created by ch on 2025/9/19.
//

#pragma once

#include <chrono>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <control_msgs/srv/query_calibration_state.hpp>
#include <rm2_msgs/srv/status_change.hpp>

namespace rm2_common
{

template <class ServiceType>
class ServiceCallerBase
{
public:
  explicit ServiceCallerBase(rclcpp::Node::SharedPtr node, const std::string& service_name = "")
    : node_(node), fail_count_(0), fail_limit_(0)
  {
    node_->declare_parameter<int>("fail_limit", 0);
    node_->get_parameter("fail_limit", fail_limit_);

    node_->declare_parameter<std::string>("service_name", service_name);
    node_->get_parameter("service_name", service_name_);
    if (service_name_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Service name not defined (namespace: %s)",
                   node_->get_namespace());
      return;
    }

    client_ = node_->create_client<ServiceType>(service_name_);
  }

  ServiceCallerBase(rclcpp::Node::SharedPtr node, std::string& service_name)
    : node_(node), service_name_(service_name), fail_count_(0), fail_limit_(0)
  {
    client_ = node_->create_client<ServiceType>(service_name_);
  }

  ServiceCallerBase(std::string& param_prefix, rclcpp::Node::SharedPtr node, 
                    const std::string& service_name = "")
    : node_(node), fail_count_(0), fail_limit_(0)
  {
    node_->declare_parameter<std::string>(param_prefix + "service_name", service_name);
    node_->get_parameter(param_prefix + "service_name", service_name_);
    if (service_name_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Service name not defined (namespace: %s)",
                   node_->get_namespace());
      return;
    }

    client_ = node_->create_client<ServiceType>(service_name_);
  }

  ~ServiceCallerBase()
  {
    if (thread_ && thread_->joinable()) {
      thread_->join();
    }
    delete thread_;
  }

  void callService()
  {
    if (isCalling())
      return;
    thread_ = new std::thread(&ServiceCallerBase::callingThread, this);
    thread_->detach();
  }

  typename ServiceType::Request& getRequest()
  {
    return request_;
  }

  typename ServiceType::Response& getResponse()
  {
    return response_;
  }

  bool isCalling()
  {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }

protected:
  void callingThread()
  {
    std::lock_guard<std::mutex> guard(mutex_);

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Service %s not available. Waiting...", service_name_.c_str());
      return;
    }

    auto result_future = client_->async_send_request(std::make_shared<typename ServiceType::Request>(request_));

    if (result_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Service call to %s timed out", service_name_.c_str());
      handleFailure();
      return;
    }

    auto response = result_future.get();
    if (!response) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Service call to %s failed", service_name_.c_str());
      handleFailure();
      return;
    }

    response_ = *response;
  }

  void handleFailure()
  {
    if (fail_limit_ != 0) {
      fail_count_++;
      if (fail_count_ >= fail_limit_) {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "Failed to call service %s", service_name_.c_str());
        fail_count_ = 0;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string service_name_;
  typename rclcpp::Client<ServiceType>::SharedPtr client_;
  typename ServiceType::Request request_;
  typename ServiceType::Response response_;
  std::thread* thread_{};
  std::mutex mutex_;
  int fail_count_, fail_limit_;
};

class SwitchControllersServiceCaller : public ServiceCallerBase<controller_manager_msgs::srv::SwitchController>
{
public:
  explicit SwitchControllersServiceCaller(rclcpp::Node::SharedPtr node)
      : rm2_common::ServiceCallerBase<controller_manager_msgs::srv::SwitchController>(
        node, "/controller_manager/switch_controller")
  {
    request_.strictness = request_.BEST_EFFORT;
    request_.activate_asap = true;
  }

  void startControllers(const std::vector<std::string>& controllers)
  {
    request_.activate_controllers = controllers;
  }

  void stopControllers(const std::vector<std::string>& controllers)
  {
    request_.deactivate_controllers = controllers;
  }

  bool getOk()
  {
    if (isCalling())
      return false;
    return response_.ok;
  }
};

class QueryCalibrationServiceCaller : public ServiceCallerBase<control_msgs::srv::QueryCalibrationState>
{
public:
  explicit QueryCalibrationServiceCaller(rclcpp::Node::SharedPtr node)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(node)
  {
  }

  QueryCalibrationServiceCaller(rclcpp::Node::SharedPtr node, std::string& service_name)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(node, service_name)
  {
  }

  QueryCalibrationServiceCaller(std::string& param_prefix, rclcpp::Node::SharedPtr node)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(param_prefix, node)
  {
  }

  bool isCalibrated()
  {
    if (isCalling())
      return false;
    return response_.is_calibrated;
  }
};

class SwitchDetectionCaller : public ServiceCallerBase<rm2_msgs::srv::StatusChange>
{
public:
  explicit SwitchDetectionCaller(rclcpp::Node::SharedPtr node)
    : ServiceCallerBase<rm2_msgs::srv::StatusChange>(node, "/detection_nodelet/status_switch")
  {
    request_.target = rm2_msgs::srv::StatusChange::Request::ARMOR;
    request_.exposure = rm2_msgs::srv::StatusChange::Request::EXPOSURE_LEVEL_0;
    request_.armor_target = rm2_msgs::srv::StatusChange::Request::ARMOR_ALL;
    callService();
  }

  explicit SwitchDetectionCaller(rclcpp::Node::SharedPtr node, std::string service_name)
    : ServiceCallerBase<rm2_msgs::srv::StatusChange>(node, service_name)
  {
    request_.target = rm2_msgs::srv::StatusChange::Request::ARMOR;
    request_.exposure = rm2_msgs::srv::StatusChange::Request::EXPOSURE_LEVEL_0;
    request_.armor_target = rm2_msgs::srv::StatusChange::Request::ARMOR_ALL;
    callService();
  }

  void setEnemyColor(const int& robot_id_, const std::string& robot_color_)
  {
    if (robot_id_ != 0)
    {
      request_.color =
          robot_color_ == "blue" ? rm2_msgs::srv::StatusChange::Request::RED :
                                   rm2_msgs::srv::StatusChange::Request::BLUE;
      RCLCPP_INFO_STREAM(node_->get_logger(), "Set enemy color: " <<
                         (request_.color == request_.RED ? "red" : "blue"));

      callService();
    }
    else
      RCLCPP_INFO_STREAM(node_->get_logger(), "Set enemy color failed: referee offline");
  }

  void setColor(uint8_t color)
  {
    request_.color = color;
  }

  void switchEnemyColor()
  {
    request_.color = request_.color == rm2_msgs::srv::StatusChange::Request::RED;
  }

  void switchTargetType()
  {
    request_.target = request_.target == rm2_msgs::srv::StatusChange::Request::ARMOR;
  }

  void setTargetType(uint8_t target)
  {
    request_.target = target;
  }

  void switchArmorTargetType()
  {
    request_.armor_target = request_.armor_target == rm2_msgs::srv::StatusChange::Request::ARMOR_ALL;
  }

  void setArmorTargetType(uint8_t armor_target)
  {
    request_.armor_target = armor_target;
  }

  void switchExposureLevel()
  {
    request_.exposure = request_.exposure == rm2_msgs::srv::StatusChange::Request::EXPOSURE_LEVEL_4 ?
                        rm2_msgs::srv::StatusChange::Request::EXPOSURE_LEVEL_0 :
                        static_cast<uint8_t>(request_.exposure + 1);
  }

  int getColor() const
  {
    return request_.color;
  }

  int getTarget() const
  {
    return request_.target;
  }

  int getArmorTarget() const
  {
    return request_.armor_target;
  }

  uint8_t getExposureLevel() const
  {
    return request_.exposure;
  }

  bool getIsSwitch()
  {
    if (isCalling())
      return false;
    return response_.switch_is_success;
  }

};

}  // namespace rm2_common