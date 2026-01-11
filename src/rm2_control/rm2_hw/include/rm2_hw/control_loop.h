//
// Created by ch on 2025/10/29.
//

#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

namespace rm2_hw
{
using namespace std::chrono;
using clock = high_resolution_clock;

class RmSystemLoop
{
public:
  RmSystemLoop(std::shared_ptr<rclcpp::Node> node,  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);

  ~RmSystemLoop();

  void update();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string urdf_string_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;  
  // Settings
  double cycle_time_error_threshold_{};

  // Timing
  std::thread loop_thread_;
  std::atomic_bool loop_running_ = true;
  double loop_hz_{};
  rclcpp::Duration elapsed_time_;
  clock::time_point last_time_;



};
}  // namespace rm2_hw
