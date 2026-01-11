//
// Created by ch on 2025/10/18.
//

#include <rclcpp/parameter_map.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rm2_hw/hardware_interface/gpio_manager.h>
#include <stdexcept>

namespace rm2_hw
{
GpioManager::GpioManager(rclcpp::Node::SharedPtr node) : node_(node)
{
}

GpioManager::~GpioManager()
{
}
void GpioManager::setGpioDirection(rm2_control::GpioData gpioData)
{
  std::string file = "/sys/class/gpio/gpio" + std::to_string(gpioData.pin) + "/direction";
  int fd;
  fd = open(file.data(), O_WRONLY);
  if (fd == -1)
  {
    RCLCPP_ERROR(node_->get_logger(),"[gpio]Unable to open %s", file.data());
  }
  else
  {
    if (gpioData.type == rm2_control::OUTPUT)
    {
      if (write(fd, "out", 3) != 3)
      {
        RCLCPP_ERROR(node_->get_logger(),"[gpio]Failed to set direction of gpio%d", gpioData.pin);
      }
    }
    else
    {
      if (write(fd, "in", 2) != 2)
      {
        RCLCPP_ERROR(node_->get_logger(),"[gpio]Failed to set direction of gpio%d", gpioData.pin);
      }
    }
  }
  close(fd);
}

void GpioManager::readGpio()
{
  for (auto iter = gpio_state_values.begin(); iter != gpio_state_values.end(); iter++)
  {
    if (iter->type == rm2_control::INPUT)
    {
      std::string file = "/sys/class/gpio/gpio" + std::to_string(iter->pin) + "/value";
      FILE* fp = fopen(file.c_str(), "r");
      if (fp == NULL)
      {
        RCLCPP_ERROR(node_->get_logger(),"[gpio]Unable to read /sys/class/gpio/gpio%d/value", iter->pin);
      }
      else
      {
        char state = fgetc(fp);
        bool value = (state == 0x31);
        *iter->value = value;
        fclose(fp);
      }
    }
  }
}

void GpioManager::writeGpio()
{
  char buffer[1] = { '1' };
  for (auto iter : gpio_command_values)
  {
    std::string file = "/sys/class/gpio/gpio" + std::to_string(iter.pin) + "/value";
    int fd = open(file.c_str(), O_WRONLY);
    if (fd == -1)
    {
      RCLCPP_ERROR(node_->get_logger(),"[gpio]Unable to write /sys/class/gpio/gpio%i/value", iter.pin);
    }
    else
    {
      lseek(fd, 0, SEEK_SET);
      if (*iter.value)
      {
        buffer[0] = '1';
        int ref = write(fd, buffer, 1);
        if (ref == -1)
          RCLCPP_ERROR(node_->get_logger(),"[GPIO]Failed to write to gpio%d.", iter.pin);
      }
      else
      {
        buffer[0] = '0';
        int ref = write(fd, buffer, 1);
        if (ref == -1)
          RCLCPP_ERROR(node_->get_logger(),"[GPIO]Failed to write to gpio%d.", iter.pin);
      }
    }
    close(fd);
  }
}
}  // namespace rm2_hw
