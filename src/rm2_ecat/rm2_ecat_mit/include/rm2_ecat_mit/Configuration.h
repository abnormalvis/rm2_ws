//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_map>

namespace rm2_ecat {
namespace mit {
enum class CanBus { CAN0 = 0, CAN1 = 1 };

static constexpr size_t motorNumEachBus = 4;

class MotorConfiguration {
 public:
  std::string name_;
  uint16_t maxOut_;
  double torqueOffset;
  double positionOffset;
  double velocityOffset;
  double kpFactorToInteger;
  double kdFactorToInteger;
  double torqueFactorIntegerToNm_;
  double torqueFactorNmToInteger_;
  double positionFactorIntegerToRad_;
  double positionFactorRadToInteger_;
  double velocityFactorIntegerPerMinusToRadPerSec_;
  double velocityFactorRadPerSecToInteger_;
  bool needCalibration_ = false;
};

class GpioConfiguration {
 public:
  std::string name_;
  uint16_t mode_;
};

class Configuration {
 public:
  std::unordered_map<uint8_t, MotorConfiguration> can0MotorConfigurations_;
  std::unordered_map<uint8_t, MotorConfiguration> can1MotorConfigurations_;
  std::unordered_map<uint8_t, GpioConfiguration> gpioConfigurations_;

  bool sanityCheck(bool silent = false) const;

  uint8_t getGpioModes() const;

  friend std::ostream& operator<<(std::ostream& os, const Configuration& configuration);
};

inline size_t getIndex(CanBus bus, size_t id) {
  return static_cast<uint16_t>(bus) * motorNumEachBus + id - 1;
}
}  // namespace mit
}  // namespace rm2_ecat
