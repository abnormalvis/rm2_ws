//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>

#include "rm2_ecat_standard_slave/Configuration.h"

namespace rm2_ecat {
namespace standard {
class Command {
 public:
  Command() = default;
  Command(const Command& other);

  void configureCommand(const Configuration& configuration);

  Command& operator=(const Command& other);

  // Set factors
  void setMaxOut(CanBus bus, size_t id, int16_t factor);
  void setTorqueFactorNmToInteger(CanBus bus, size_t id, double factor);
  // Set (SI units)
  void setTargetCommand(CanBus bus, size_t id, double targetTorque);
  void setDigitalOutput(uint8_t id, bool value);

  // Get (raw)
  int16_t getTargetTorqueRaw(CanBus bus, size_t id) const;
  uint8_t getDigitalOutputs() const;

  friend std::ostream& operator<<(std::ostream& os, Command& command);

 private:
  uint32_t controlWord_{0};
  double targetTorque_[16]{0};
  int16_t maxOut_[16]{0};
  double torqueFactorNmToInteger_[16]{1};
  uint8_t digitalOutputs_{0};
};
}  // namespace standard
}  // namespace rm2_ecat
