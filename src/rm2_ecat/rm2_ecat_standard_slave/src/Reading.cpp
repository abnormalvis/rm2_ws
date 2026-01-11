//
// Created by qiayuan on 23-5-14.
//

#include "rm2_ecat_standard_slave/Reading.h"

#include <cmath>
#include "rm2_msgs/msg/dbus_data.hpp"

namespace rm2_ecat {
namespace standard {
Reading::Reading() {
  for (auto& velocityFilter : velocityFilters_) {
    velocityFilter = std::make_unique<LowPassFilter>(100);
  }
}

std::string Reading::getMotorName(CanBus bus, size_t id) const {
  return names_[getIndex(bus, id)];
}

std::vector<size_t> Reading::getEnabledMotorIds(CanBus bus) const {
  std::vector<size_t> enabledMotorIds;
  for (size_t id = 1; id < 9; ++id) {
    if (isMotorEnabled_[getIndex(bus, id)]) {
      enabledMotorIds.push_back(id);
    }
  }
  return enabledMotorIds;
}

std::vector<CanBus> Reading::getEnabledImuBuss() const {
  std::vector<CanBus> buss_;
  if (isImuEnabled_[0]) {
    buss_.push_back(CanBus::CAN0);
  }
  if (isImuEnabled_[1]) {
    buss_.push_back(CanBus::CAN1);
  }
  return buss_;
}

std::vector<uint8_t> Reading::getEnabledDigitalInputIds() const {
  std::vector<uint8_t> ids;
  for (uint8_t id = 0; id < 8; ++id) {
    if (isDigitalInputEnabled_[id]) {
      ids.push_back(id);
    }
  }
  return ids;
}

bool Reading::getEnabledDbus() const {
  return isDbusEnabled_;
}

std::string Reading::getDbusName() const {
  return dbusName_;
}

void Reading::configureReading(const Configuration& configuration) {
  for (const auto& [id, motorConfiguration] : configuration.can0MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN0, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    positionFactorIntegerToRad_[index] = 2. * M_PI / 8191.;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = 2. * M_PI / 60.;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
  }
  for (const auto& [id, motorConfiguration] : configuration.can1MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN1, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    positionFactorIntegerToRad_[index] = 2. * M_PI / 8191.;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = 2. * M_PI / 60.;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
  }
  if (configuration.can0ImuConfiguration_ != nullptr) {
    imuNames_[0] = configuration.can0ImuConfiguration_->name_;
    isImuEnabled_[0] = true;
    angularVelFactorIntegerToRadPerSecond_[0] = configuration.can0ImuConfiguration_->angularVelFactorIntegerToRadPerSecond_;
    linearAccelFactorIntegerToMeterPerSecondSquared_[0] =
        configuration.can0ImuConfiguration_->linearAccelFactorIntegerToMeterPerSecondSquared_;
    complementaryFilters_[0].setGainAcc(configuration.can0ImuConfiguration_->gainAccel_);
    complementaryFilters_[0].setBiasAlpha(configuration.can0ImuConfiguration_->biasAlpha_);
    complementaryFilters_[0].setDoBiasEstimation(configuration.can0ImuConfiguration_->doBiasEstimation_);
    complementaryFilters_[0].setDoAdaptiveGain(configuration.can0ImuConfiguration_->doAdaptiveGain_);
    for (size_t i = 0; i < 3; ++i) {
      angularVelocityBias_[i] = configuration.can0ImuConfiguration_->angularVelBias_[i];
    }
  }
  if (configuration.can1ImuConfiguration_ != nullptr) {
    imuNames_[1] = configuration.can1ImuConfiguration_->name_;
    isImuEnabled_[1] = true;
    angularVelFactorIntegerToRadPerSecond_[1] = configuration.can1ImuConfiguration_->angularVelFactorIntegerToRadPerSecond_;
    linearAccelFactorIntegerToMeterPerSecondSquared_[1] =
        configuration.can1ImuConfiguration_->linearAccelFactorIntegerToMeterPerSecondSquared_;
    complementaryFilters_[1].setGainAcc(configuration.can1ImuConfiguration_->gainAccel_);
    complementaryFilters_[1].setBiasAlpha(configuration.can1ImuConfiguration_->biasAlpha_);
    complementaryFilters_[1].setDoBiasEstimation(configuration.can1ImuConfiguration_->doBiasEstimation_);
    complementaryFilters_[1].setDoAdaptiveGain(configuration.can1ImuConfiguration_->doAdaptiveGain_);
    for (size_t i = 0; i < 3; ++i) {
      angularVelocityBias_[i + 3] = configuration.can1ImuConfiguration_->angularVelBias_[i];
    }
  }
  for (const auto& [id, gpioConfiguration] : configuration.gpioConfigurations_) {
    if (gpioConfiguration.mode_ == 0) {
      isDigitalInputEnabled_[id] = true;
      gpioNames_[id] = gpioConfiguration.name_;
    }
  }
  if (configuration.dbusConfiguration != nullptr) {
    isDbusEnabled_ = true;
    dbusName_ = configuration.dbusConfiguration->name_;
  }
}

uint16_t Reading::getPositionRaw(CanBus bus, size_t id) const {
  return actualPosition_[getIndex(bus, id)];
}

int16_t Reading::getVelocityRaw(CanBus bus, size_t id) const {
  return actualVelocity_[getIndex(bus, id)];
}

int16_t Reading::getCurrentRaw(CanBus bus, size_t id) const {
  return actualCurrent_[getIndex(bus, id)];
}

uint8_t Reading::getTemperatureRaw(CanBus bus, size_t id) const {
  return actualTemperature_[getIndex(bus, id)];
}

int64_t Reading::getCircleRaw(CanBus bus, size_t id) const {
  return actualCircle_[getIndex(bus, id)];
}

double Reading::getPosition(CanBus bus, size_t id) const {
  size_t i = getIndex(bus, id);
  return actualPosition_[i] * positionFactorIntegerToRad_[i] + static_cast<double>(actualCircle_[i]) * 2. * M_PI;
}

double Reading::getVelocity(CanBus bus, size_t id) const {
  return actualVelocity_[getIndex(bus, id)] * velocityFactorIntegerPerMinusToRadPerSec_[getIndex(bus, id)];
}

double Reading::getTorque(CanBus bus, size_t id) const {
  return actualCurrent_[getIndex(bus, id)] * torqueFactorIntegerToNm_[getIndex(bus, id)];
}

double Reading::getTemperature(CanBus bus, size_t id) const {
  return actualTemperature_[getIndex(bus, id)];
}

void Reading::getOrientation(CanBus bus, double& w, double& x, double& y, double& z) const {
  auto i = static_cast<size_t>(bus);
  complementaryFilters_[i].getOrientation(w, x, y, z);
}

void Reading::getAngularVelocity(CanBus bus, double& x, double& y, double& z) const {
  auto i = static_cast<size_t>(bus);
  x = angularVelocity_[3 * i + 0];
  y = angularVelocity_[3 * i + 1];
  z = angularVelocity_[3 * i + 2];
}

void Reading::getLinearAcceleration(CanBus bus, double& x, double& y, double& z) const {
  auto i = static_cast<size_t>(bus);
  x = linearAcceleration_[3 * i + 0];
  y = linearAcceleration_[3 * i + 1];
  z = linearAcceleration_[3 * i + 2];
}

bool Reading::getDigitalInput(uint8_t id) const {
  return digitalInputs_[id];
}

rm2_msgs::msg::DbusData Reading::getDbusData() const {
  return dbusData_;
}

bool Reading::getDbusStatus() const {
  return isRcOpen_;
}

void Reading::setPosition(CanBus bus, size_t id, uint16_t position) {
  size_t i = getIndex(bus, id);
  uint16_t lastPosition = actualPosition_[i];
  if (position - lastPosition > 4096) {
    actualCircle_[i]--;
  } else if (position - lastPosition < -4096) {
    actualCircle_[i]++;
  }
  if (firstReceived_[i]) {
    actualCircle_[i] = 0;
    firstReceived_[i] = false;
  }
  actualPosition_[i] = position;
}

void Reading::setVelocity(CanBus bus, size_t id, int16_t velocity) {
  size_t i = getIndex(bus, id);

  double time = getTime(stamp_);

  velocityFilters_[i]->input(velocity, time);
  //  actualVelocity_[i] = velocityFilters_[i]->output();
  actualVelocity_[getIndex(bus, id)] = velocity;
}

void Reading::setTorque(CanBus bus, size_t id, int16_t current) {
  actualCurrent_[getIndex(bus, id)] = current;
}

void Reading::setTemperature(CanBus bus, size_t id, uint8_t temperature) {
  actualTemperature_[getIndex(bus, id)] = temperature;
}

std::string Reading::getImuName(CanBus bus) const {
  return imuNames_[static_cast<size_t>(bus)];
}

void Reading::setAngularVelocity(CanBus bus, const int16_t* angularVelocity) {
  auto i = static_cast<size_t>(bus);
  angularVelocity_[i * 3 + 0] = angularVelocity[0] * angularVelFactorIntegerToRadPerSecond_[i] + angularVelocityBias_[i * 3 + 0];
  angularVelocity_[i * 3 + 1] = angularVelocity[1] * angularVelFactorIntegerToRadPerSecond_[i] + angularVelocityBias_[i * 3 + 1];
  angularVelocity_[i * 3 + 2] = angularVelocity[2] * angularVelFactorIntegerToRadPerSecond_[i] + angularVelocityBias_[i * 3 + 2];
}

void Reading::setLinearAcceleration(CanBus bus, const int16_t* linearAcceleration) {
  auto i = static_cast<size_t>(bus);
  linearAcceleration_[i * 3 + 0] = linearAcceleration[0] * linearAccelFactorIntegerToMeterPerSecondSquared_[i];
  linearAcceleration_[i * 3 + 1] = linearAcceleration[1] * linearAccelFactorIntegerToMeterPerSecondSquared_[i];
  linearAcceleration_[i * 3 + 2] = linearAcceleration[2] * linearAccelFactorIntegerToMeterPerSecondSquared_[i];
  double dt = getTime(stamp_) - lastUpdateTimes_[i];
  lastUpdateTimes_[i] = getTime(stamp_);
  complementaryFilters_[i].update(linearAcceleration_[i * 3 + 0], linearAcceleration_[i * 3 + 1], linearAcceleration_[i * 3 + 2],
                                  angularVelocity_[i * 3 + 0], angularVelocity_[i * 3 + 1], angularVelocity_[i * 3 + 2], dt);
}

void Reading::setDigitalInputs(uint8_t value) {
  for (size_t i = 0; i < 8; ++i) {
    digitalInputs_[i] = ((value & (1 << i)) != 0);
  }
}

std::string Reading::getGpioName(uint8_t id) const{
  return gpioNames_[id];
}

void Reading::setDbusData(const int16_t* dbus_data_1, const int16_t* dbus_data_2) {
  dbusData_.ch_r_x = static_cast<double>(dbus_data_1[0] / 660.0);
  dbusData_.ch_r_y = static_cast<double>(dbus_data_1[1] / 660.0);
  dbusData_.ch_l_x = static_cast<double>(dbus_data_1[2] / 660.0);
  dbusData_.ch_l_y = static_cast<double>(dbus_data_1[3] / 660.0);
  dbusData_.m_x = static_cast<double>(dbus_data_2[0] / 1600.0);
  dbusData_.m_y = static_cast<double>(dbus_data_2[1] / 1600.0);
  dbusData_.m_z = static_cast<double>(dbus_data_2[2] / 1600.0);
  dbusData_.wheel = static_cast<double>(dbus_data_1[6] / 660.0);

  if (dbus_data_1[5] != 0) {
    dbusData_.s_l = dbus_data_1[5];
  }
  if (dbus_data_1[4] != 0) {
    dbusData_.s_r = dbus_data_1[4];
  }
  dbusData_.p_l = dbus_data_2[3];
  dbusData_.p_r = dbus_data_2[4];

  dbusData_.key_w = dbus_data_2[5] & 0x01 ? true : false;
  dbusData_.key_s = dbus_data_2[5] & 0x02 ? true : false;
  dbusData_.key_a = dbus_data_2[5] & 0x04 ? true : false;
  dbusData_.key_d = dbus_data_2[5] & 0x08 ? true : false;
  dbusData_.key_shift = dbus_data_2[5] & 0x10 ? true : false;
  dbusData_.key_ctrl = dbus_data_2[5] & 0x20 ? true : false;
  dbusData_.key_q = dbus_data_2[5] & 0x40 ? true : false;
  dbusData_.key_e = dbus_data_2[5] & 0x80 ? true : false;
  dbusData_.key_r = (dbus_data_2[5] >> 8) & 0x01 ? true : false;
  dbusData_.key_f = (dbus_data_2[5] >> 8) & 0x02 ? true : false;
  dbusData_.key_g = (dbus_data_2[5] >> 8) & 0x04 ? true : false;
  dbusData_.key_z = (dbus_data_2[5] >> 8) & 0x08 ? true : false;
  dbusData_.key_x = (dbus_data_2[5] >> 8) & 0x10 ? true : false;
  dbusData_.key_c = (dbus_data_2[5] >> 8) & 0x20 ? true : false;
  dbusData_.key_v = (dbus_data_2[5] >> 8) & 0x40 ? true : false;
  dbusData_.key_b = (dbus_data_2[5] >> 8) & 0x80 ? true : false;
  isRcOpen_ = dbus_data_2[6];
}
}  // namespace standard
}  // namespace rm2_ecat
