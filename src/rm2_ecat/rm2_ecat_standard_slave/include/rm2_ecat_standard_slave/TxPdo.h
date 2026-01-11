//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>

namespace rm2_ecat {
namespace standard {
struct TxPdo {
  uint16_t can0MotorPositions_[8];
  uint16_t can1MotorPositions_[8];
  int16_t can0MotorVelocities_[8];
  int16_t can1MotorVelocities_[8];
  int16_t can0MotorCurrents_[8];
  int16_t can1MotorCurrents_[8];
  uint8_t can0MotorTemperatures_[8];
  uint8_t can1MotorTemperatures_[8];
  int16_t can0ImuLinearAcceleration_[3];
  int16_t can1ImuLinearAcceleration_[3];
  int16_t can0ImuAngularVelocity_[3];
  int16_t can1ImuAngularVelocity_[3];
  uint8_t digital_inputs_;
  int16_t dbus_data_1_[8];
  int16_t dbus_data_2_[8];
  uint32_t statusword_;
} __attribute__((packed));
}  // namespace standard
}  // namespace rm2_ecat
