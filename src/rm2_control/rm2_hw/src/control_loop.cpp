//
// Created by ch on 2025/10/29.
//

#include "rm2_hw/control_loop.h"
#include <controller_manager/controller_manager.hpp>
#include <ctime>
#include <hardware_interface/resource_manager.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

namespace rm2_hw
{
RmSystemLoop::RmSystemLoop(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
  : node_(node), elapsed_time_(0.0, 0.0)
{
  // Load ros2 params
  int thread_priority;
  node->declare_parameter("loop_frequency",1.0);
  node->declare_parameter("cycle_time_error_threshold",0.001);
  node->declare_parameter("thread_priority",95);
  node->declare_parameter("robot_description","");

  loop_hz_ = node_->get_parameter("loop_frequency").as_double();
  cycle_time_error_threshold_ = node_->get_parameter("cycle_time_error_threshold").as_double();
  thread_priority = node_->get_parameter("thread_priority").as_int();
  urdf_string_ = node_->get_parameter("robot_description").as_string();

  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(executor,urdf_string_,true));
  // Get current time for use with first update
  last_time_ = clock::now();

  // Setup loop thread
  loop_thread_ = std::thread([&]() {
    while (loop_running_)
    {
      if (loop_running_)
        update();
    }
  });
  sched_param sched{ .sched_priority = thread_priority };
  if (pthread_setschedparam(loop_thread_.native_handle(), SCHED_FIFO, &sched) != 0)
    RCLCPP_WARN(node_->get_logger(), "Failed to set threads priority (one possible reason could be that the user and the group permissions "
             "are not set properly.).\n");
}

void RmSystemLoop::update()
{
  const auto current_time = clock::now();
  // Compute desired duration rounded to clock decimation
  const duration<double> desired_duration(1.0 / loop_hz_);
  // Get change in time
  auto time_span = current_time - last_time_;
  elapsed_time_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(time_span));
  last_time_ = current_time;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsed_time_ - rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(desired_duration))).seconds();
  if (cycle_time_error > cycle_time_error_threshold_)
    RCLCPP_WARN_STREAM(node_->get_logger(),"Cycle time exceeded error threshold by: " << cycle_time_error - cycle_time_error_threshold_ << "s, "
                                                                                      << "cycle time: " << elapsed_time_.seconds() << "s, "
                                                                                      << "threshold: " << cycle_time_error_threshold_ << "s");
  // Input
  // get the hardware's state
  controller_manager_->read(node_->now(), elapsed_time_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controller_manager_->update(node_->now(), elapsed_time_);
  // Output
  // send the new command to hardware
  controller_manager_->write(node_->now(), elapsed_time_);

  // Sleep
  const auto sleep_till = current_time + duration_cast<clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);
}

RmSystemLoop::~RmSystemLoop()
{
  RCLCPP_INFO(node_->get_logger(),"loop stop!!!!!!!!!");
  loop_running_ = false;
  if (loop_thread_.joinable())
    loop_thread_.join();
}
}  // namespace rm2_hw
