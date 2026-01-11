//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "Sdo.h"

namespace rm2_ecat {
namespace standard {
enum class CanBus { CAN0 = 0, CAN1 = 1 };

class MotorConfiguration {
 public:
  std::string name_;
  uint16_t maxOut_;
  double torqueFactorIntegerToNm_;
  double torqueFactorNmToInteger_;
  bool needCalibration_ = false;
};

class ImuConfiguration {
 public:
  std::string name_;
  double angularVelFactorIntegerToRadPerSecond_ = 0.0010652644;
  double linearAccelFactorIntegerToMeterPerSecondSquared_ = 0.0017944335;
  double angularVelBias_[3]{0};
  double gainAccel_ = 0.0003;
  double biasAlpha_ = 0.01;
  bool doBiasEstimation_ = false;
  bool doAdaptiveGain_ = true;
};

class GpioConfiguration {
 public:
  std::string name_;
  uint16_t mode_;
};

class DbusConfiguration {
 public:
  std::string name_;
};

class Configuration {
 public:
  std::unordered_map<uint8_t, MotorConfiguration> can0MotorConfigurations_;
  std::unordered_map<uint8_t, MotorConfiguration> can1MotorConfigurations_;
  std::shared_ptr<ImuConfiguration> can0ImuConfiguration_, can1ImuConfiguration_;
  std::unordered_map<uint8_t, GpioConfiguration> gpioConfigurations_;
  std::shared_ptr<DbusConfiguration> dbusConfiguration;

  bool sanityCheck(bool silent = false) const;

  uint8_t getGpioModes() const;

  friend std::ostream& operator<<(std::ostream& os, const Configuration& configuration);
};

inline size_t getIndex(CanBus bus, size_t id) {
  return static_cast<uint16_t>(bus) * 8 + id - 1;
}
}  // namespace standard
}  // namespace rm2_ecat
