//
// Created by ch on 2025/10/18.
//

#pragma once

#include <fcntl.h>
#include <map>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <rm2_common/hardware_handle/gpio_handle.h>

namespace rm2_hw
{
class GpioManager
{
public:
  explicit GpioManager(rclcpp::Node::SharedPtr node);
  ~GpioManager();

  void setGpioDirection(rm2_control::GpioData gpioData);
  void readGpio();
  void writeGpio();

  std::vector<rm2_control::GpioData> gpio_state_values;
  std::vector<rm2_control::GpioData> gpio_command_values;
protected:
  rclcpp::Node::SharedPtr node_;
};
}  // namespace rm2_hw
