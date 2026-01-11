//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "rm2_ecat_standard_slave/Configuration.h"

namespace rm2_ecat {
namespace standard {
class Statusword {
 public:
  friend std::ostream& operator<<(std::ostream& os, const Statusword& statusword);

  uint32_t getRaw() const { return statusword_; }
  void setRaw(uint32_t raw) { statusword_ = raw; }

  // Motor
  bool isOnline(CanBus bus, size_t id) const;

  // Imu
  bool isAngularVelocityUpdated(CanBus bus) const;
  bool isLinearAccelerationUpdated(CanBus bus) const;
  bool isTriggered(CanBus bus) const;
  bool isTriggerEnabled(CanBus bus) const;

 private:
  uint32_t statusword_{0};
};
}  // namespace standard
}  // namespace rm2_ecat
