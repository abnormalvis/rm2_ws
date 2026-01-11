//
// Created by qiayuan on 23-5-14.
//

#pragma once
#include <chrono>

#include <memory>
#include <string>
#include <vector>

#include "rm2_ecat_standard_slave/ComplementaryFilter.h"
#include "rm2_ecat_standard_slave/Configuration.h"
#include "rm2_ecat_standard_slave/LowPassFilter.h"
#include "rm2_ecat_standard_slave/Statusword.h"
#include "rm2_msgs/msg/dbus_data.hpp"
#include "rm2_msgs/msg/gpio_data.hpp"

namespace rm2_ecat {
namespace standard {
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
  std::vector<CanBus> getEnabledImuBuss() const;
  std::vector<uint8_t> getEnabledDigitalInputIds() const;
  bool getEnabledDbus() const;
  /*!
   * raw get methods
   */
  std::string getMotorName(CanBus bus, size_t id) const;
  uint16_t getPositionRaw(CanBus bus, size_t id) const;
  int16_t getVelocityRaw(CanBus bus, size_t id) const;
  int16_t getCurrentRaw(CanBus bus, size_t id) const;
  uint8_t getTemperatureRaw(CanBus bus, size_t id) const;
  int64_t getCircleRaw(CanBus bus, size_t id) const;

  /*!
   * SI units get methods
   */
  // Motors
  double getPosition(CanBus bus, size_t id) const;
  double getVelocity(CanBus bus, size_t id) const;
  double getTorque(CanBus bus, size_t id) const;
  double getTemperature(CanBus bus, size_t id) const;
  // IMUs
  std::string getImuName(CanBus bus) const;
  void getOrientation(CanBus bus, double& w, double& x, double& y, double& z) const;
  void getAngularVelocity(CanBus bus, double& x, double& y, double& z) const;
  void getLinearAcceleration(CanBus bus, double& x, double& y, double& z) const;
  // Digital inputs
  bool getDigitalInput(uint8_t id) const;
  std::string getGpioName(uint8_t id) const;
  // Dbus
  std::string getDbusName() const;
  rm2_msgs::msg::DbusData getDbusData() const;
  bool getDbusStatus() const;

  /*!
   * set methods (only raw)
   */
  // Motors
  void setPosition(CanBus bus, size_t id, uint16_t position);
  void setVelocity(CanBus bus, size_t id, int16_t velocity);
  void setTorque(CanBus bus, size_t id, int16_t current);
  void setTemperature(CanBus bus, size_t id, uint8_t temperature);
  // IMUs
  void setAngularVelocity(CanBus bus, const int16_t* angularVelocity);
  void setLinearAcceleration(CanBus bus, const int16_t* linearAcceleration);
  // Digital inputs
  void setDigitalInputs(uint8_t value);
  // Dbus data
  void setDbusData(const int16_t* dbus_data_1, const int16_t* dbus_data_2);

 protected:
  ReadingTimePoint stamp_;
  Statusword statusword_;

  // Motors
  std::string names_[16]{""};
  bool isMotorEnabled_[16]{false};

  uint16_t actualPosition_[16]{0};
  int16_t actualVelocity_[16]{0};
  int16_t actualCurrent_[16]{0};
  uint8_t actualTemperature_[16]{0};
  int64_t actualCircle_[16]{0};
  bool firstReceived_[16]{true};

  double positionFactorIntegerToRad_[16]{1};
  double velocityFactorIntegerPerMinusToRadPerSec_[16]{1};
  double torqueFactorIntegerToNm_[16]{1};

  std::shared_ptr<LowPassFilter> velocityFilters_[16];

  // IMUs
  std::string imuNames_[2]{""};
  bool isImuEnabled_[2]{false};
  double lastUpdateTimes_[2] = {0.};
  double linearAcceleration_[6]{0};
  double angularVelocity_[6]{0};
  double angularVelocityBias_[6]{0};

  double angularVelFactorIntegerToRadPerSecond_[2]{};
  double linearAccelFactorIntegerToMeterPerSecondSquared_[2]{};

  imu_tools::ComplementaryFilter complementaryFilters_[2];

  // Digital inputs
  bool isDigitalInputEnabled_[8]{false};
  bool digitalInputs_[8]{false};
  std::string gpioNames_[8]{""};

  // Dbus readings
  bool isDbusEnabled_{false};
  std::string dbusName_{""};
  rm2_msgs::msg::DbusData dbusData_;
  bool isRcOpen_{false};
};
}  // namespace standard
}  // namespace rm2_ecat
