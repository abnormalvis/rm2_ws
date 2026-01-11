//
// Created by ch on 2025/10/5.
//

#pragma once

#include <cassert>
#include <stdexcept>

namespace rm2_control
{
class TofRadarHandle
{
public:
  TofRadarHandle() = default;

  TofRadarHandle(std::string name, double* distance, double* strength)
    : name_(std::move(name)), distance_(distance), strength_(strength)
  {
    if (!distance_)
      throw std::runtime_error("Cannot create handle '" + name +
                               "'. distance_ pointer is null.");
    if (!strength_)
      throw std::runtime_error("Cannot create handle '" + name +
                               "'. strength_ pointer is null.");
  }

  std::string getName() const
  {
    return name_;
  }

  double getDistance() const
  {
    assert(distance_);
    return *distance_;
  }

  double getStrength() const
  {
    assert(strength_);
    return *strength_;
  }

private:
  std::string name_;
  double* distance_;
  double* strength_;
};

}  // namespace rm2_control
