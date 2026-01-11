//
// Created by ch on 2025/10/29.
//

#include "rm2_hw/control_loop.h"
#include <memory>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rm2_hw");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
  executor->add_node(node);
  try
  {
    rm2_hw::RmSystemLoop control_loop(node, executor);
    executor->spin();
  }
  catch (const rclcpp::exceptions::RCLError& e)
  {
    RCLCPP_FATAL_STREAM(node->get_logger(),"Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }
  rclcpp::shutdown();
  executor->cancel();

  return 0;
}
