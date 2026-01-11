//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "rm2_ecat_mit/Configuration.h"

namespace rm2_ecat {
namespace mit {

struct target {
  double targetPosition;
  double targetVelocity;
  double targetTorque;
  double kp;
  double kd;
};

class Command {
 public:
  Command() = default;
  Command(const Command& other);
  Command& operator=(const Command& other);

  // Set factors
  void setMaxOut(CanBus bus, size_t id, uint16_t factor);
  void setTorqueFactorNmToInteger(CanBus bus, size_t id, double factor);
  void setVelocityFactorRadPerSecToInteger(CanBus bus, size_t id, double factor);
  void setPositionFactorRadToInteger(CanBus bus, size_t id, double factor);
  void setKpToInteger(CanBus bus, size_t id, double factor);
  void setKdToInteger(CanBus bus, size_t id, double factor);
  void setTorqueOffset(CanBus bus, size_t id, double factor);
  void setVelocityOffset(CanBus bus, size_t id, double factor);
  void setPositionOffset(CanBus bus, size_t id, double factor);
  // Set (SI units)
  void setTargetCommand(CanBus bus, size_t id, const target& target);
  void setDigitalOutput(uint8_t id, bool value);

  // Get (raw)
  uint64_t getMotorCommandRaw(CanBus bus, size_t id) const;
  uint8_t getDigitalOutputs() const;

  friend std::ostream& operator<<(std::ostream& os, Command& command);

 private:
  uint32_t controlWord_{0};
  double targetTorque_[motorNumEachBus * 2]{0};
  double targetPosition_[motorNumEachBus * 2]{0};
  double targetVelocity_[motorNumEachBus * 2]{0};
  double targetKp_[motorNumEachBus * 2]{0};
  double targetKd_[motorNumEachBus * 2]{0};
  uint16_t maxOut_[motorNumEachBus * 2]{0};
  double torqueFactorNmToInteger_[motorNumEachBus * 2]{1};
  double positionFactorRadToInteger_[motorNumEachBus * 2]{1};
  double velocityFactorRadPerSecToInteger_[motorNumEachBus * 2]{1};
  double kpToInteger_[motorNumEachBus * 2]{1};
  double kdToInteger_[motorNumEachBus * 2]{1};
  double torqueOffset[motorNumEachBus * 2]{0};
  double velocityOffset[motorNumEachBus * 2]{0};
  double positionOffset[motorNumEachBus * 2]{0};
  uint8_t digitalOutputs_{0};
};
}  // namespace mit
}  // namespace rm2_ecat
