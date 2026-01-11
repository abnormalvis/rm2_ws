//
// Created by ch on 2025/9/20.
//

#pragma once

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include "rm2_common/decision/service_caller.h"

namespace rm2_common
{
class ControllerManager
{
public:
  explicit ControllerManager(rclcpp::Node::SharedPtr node)
  : node_(node),
    switch_caller_(node)
  {
    load_client_ = node_->create_client<controller_manager_msgs::srv::LoadController>(
      "/controller_manager/load_controller");
    
    while (!load_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for load_controller service...");
    }

    if (node_->get_parameter("controllers_list.state_controllers", state_controllers_)) {
      for (const auto & controller : state_controllers_) {
        loadController(controller);
      }
    }

    if (node_->get_parameter("controllers_list.main_controllers", main_controllers_)) {
      for (const auto & controller : main_controllers_) {
        loadController(controller);
      }
    }

    if (node_->get_parameter("controllers_list.calibration_controllers", calibration_controllers_)) {
      for (const auto & controller : calibration_controllers_) {
        loadController(controller);
      }
    }
  }

  void update()
  {
    if (!switch_caller_.isCalling()) {
      switch_caller_.startControllers(start_buffer_);
      switch_caller_.stopControllers(stop_buffer_);
      if (!start_buffer_.empty() || !stop_buffer_.empty()) {
        switch_caller_.callService();
        start_buffer_.clear();
        stop_buffer_.clear();
      }
    }
  }

  void loadController(const std::string & controller)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    request->name = controller;

    auto future = load_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->ok) {
        RCLCPP_INFO(node_->get_logger(), "Loaded %s", controller.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load %s", controller.c_str());
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call load_controller service");
    }
  }

  void startController(const std::string& controller) 
  {
    if (std::find(start_buffer_.begin(), start_buffer_.end(), controller) == start_buffer_.end())
      start_buffer_.push_back(controller);
    // AVoid setting controller to start and stop in the same time
    auto item = std::find(stop_buffer_.begin(), stop_buffer_.end(), controller);
    if (item != stop_buffer_.end())
      stop_buffer_.erase(item);
  }
  void stopController(const std::string& controller) 
  {
    if (std::find(stop_buffer_.begin(), stop_buffer_.end(), controller) == stop_buffer_.end())
      stop_buffer_.push_back(controller);
    // AVoid setting controller to start and stop in the same time
    auto item = std::find(start_buffer_.begin(), start_buffer_.end(), controller);
    if (item != start_buffer_.end())
      start_buffer_.erase(item);
  }
  void startControllers(const std::vector<std::string>& controllers) 
  {
    for (const auto& controller : controllers)
      startController(controller);
  }
  void stopControllers(const std::vector<std::string>& controllers) 
  {
    for (const auto& controller : controllers)
      stopController(controller);
  }
  void startStateControllers() 
  { 
    startControllers(state_controllers_);
  }
  void startMainControllers() 
  { 
    startControllers(main_controllers_);
  }
  void stopMainControllers() 
  { 
    stopControllers(main_controllers_);
  }
  void startCalibrationControllers() 
  { 
    startControllers(calibration_controllers_); 
  }
  void stopCalibrationControllers() 
  { 
    stopControllers(calibration_controllers_); 
  }
  bool isCalling() 
  { 
    return switch_caller_.isCalling(); 
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_client_;
  std::vector<std::string> state_controllers_, main_controllers_, calibration_controllers_;
  std::vector<std::string> start_buffer_, stop_buffer_;
  SwitchControllersServiceCaller switch_caller_;
};

}  // namespace rm2_common