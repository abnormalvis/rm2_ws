//
// Created by kook on 12/29/23.
//

#pragma once
#include <chrono>
#include <cstdint>

#include <memory>
#include <string>
#include <vector>

#include "rm2_ecat_mit/Configuration.h"
#include "rm2_ecat_mit/Statusword.h"
#include "rm2_ecat_standard_slave/LowPassFilter.h"
#include "rm2_msgs/msg/gpio_data.hpp"

namespace rm2_ecat {
namespace mit {
using ReadingTimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

inline double getTime(ReadingTimePoint stamp) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(stamp.time_since_epoch()).count()) +
         1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count() % 1000000000);
}

class Reading {
 public:
  Reading();
  void configureReading(const Configuration& configuration);

  const ReadingTimePoint& getStamp() const { return stamp_; }
  void setStamp(const ReadingTimePoint& stamp) { stamp_ = stamp; }
  const Statusword& getStatusword() const { return statusword_; }
  void setStatusword(const Statusword& statusword) { statusword_ = statusword; }

  std::vector<size_t> getEnabledMotorIds(CanBus bus) const;
  std::vector<uint8_t> getEnabledDigitalInputIds() const;
  /*!
   * raw get methods
   */
  std::string getMotorName(CanBus bus, size_t id) const;
  uint16_t getRawReading(CanBus bus, size_t id) const;

  /*!
   * SI units get methods
   */
  // Motors
  double getPosition(CanBus bus, size_t id) const;
  double getVelocity(CanBus bus, size_t id) const;
  double getTorque(CanBus bus, size_t id) const;
  // Digital inputs
  bool getDigitalInput(uint8_t id) const;
  std::string getGpioName(uint8_t id) const;

  /*!
   * set methods (only raw)
   */
  // Motors
  void setRawReading(CanBus bus, size_t id, uint64_t data);
  // Digital inputs
  void setDigitalInputs(uint8_t value);

 protected:
  ReadingTimePoint stamp_;
  Statusword statusword_;

  // Motors
  std::string names_[motorNumEachBus * 2]{""};
  uint64_t rawReadings_[motorNumEachBus * 2]{0};
  bool isMotorEnabled_[motorNumEachBus * 2]{false};
  bool hasRawReading_[motorNumEachBus * 2]{false};

  double torqueOffset[motorNumEachBus * 2]{0};
  double velocityOffset[motorNumEachBus * 2]{0};
  double positionOffset[motorNumEachBus * 2]{0};
  double positionFactorIntegerToRad_[motorNumEachBus * 2]{1};
  double velocityFactorIntegerPerMinusToRadPerSec_[motorNumEachBus * 2]{1};
  double torqueFactorIntegerToNm_[motorNumEachBus * 2]{1};

  std::shared_ptr<LowPassFilter> velocityFilters_[motorNumEachBus * 2];

  // Digital inputs
  bool isDigitalInputEnabled_[8]{false};
  bool digitalInputs_[8]{false};
  std::string gpioNames_[8]{""};
};
}  // namespace mit
}  // namespace rm2_ecat
