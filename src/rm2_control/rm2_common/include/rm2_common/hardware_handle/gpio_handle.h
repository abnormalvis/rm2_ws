//
// Created by ch on 2025/10/5.
//

#pragma once

#include <cassert>
#include <stdexcept>

namespace rm2_control
{
enum GpioType
{
  INPUT,
  OUTPUT
};

struct GpioData
{
  std::string name;
  GpioType type;
  int pin;
  bool* value;
};

class GpioStateHandle
{
public:
  GpioStateHandle() = default;
  GpioStateHandle(std::string name, GpioType type, bool* value) : name_(std::move(name)), type_(type), value_(value)
  {
    if (!value)
      throw std::runtime_error("Cannot create handle '" + name +
                                                           "'. value pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  GpioType getType() const
  {
    return type_;
  }
  bool getValue() const
  {
    assert(value_);
    return *value_;
  }

private:
  std::string name_;
  GpioType type_;
  bool* value_ = { nullptr };
};

class GpioCommandHandle
{
public:
  GpioCommandHandle() = default;
  GpioCommandHandle(std::string name, GpioType type, bool* cmd) : name_(std::move(name)), type_(type), cmd_(cmd)
  {
    if (!cmd)
      throw std::runtime_error("Cannot create handle '" + name +
                                                           "'. command pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  bool getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }

  void setCommand(bool value)
  {
    assert(cmd_);
    *cmd_ = value;
  }

private:
  std::string name_;
  GpioType type_;
  bool* cmd_ = { nullptr };
};

}  // namespace rm2_control
