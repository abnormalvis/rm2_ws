//
// Created by kook on 12/29/23.
//
#include "rm2_ecat_mit/Command.h"

#include <cstring>

namespace rm2_ecat {
namespace mit {
void Command::setMaxOut(CanBus bus, size_t id, uint16_t factor) {
  size_t index = getIndex(bus, id);
  maxOut_[index] = factor;
}

void Command::setTorqueFactorNmToInteger(CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  torqueFactorNmToInteger_[index] = factor;
}

void Command::setVelocityFactorRadPerSecToInteger(rm2_ecat::mit::CanBus bus, size_t id, double factor){
  size_t index = getIndex(bus, id);
  velocityFactorRadPerSecToInteger_[index] = factor;
}

void Command::setPositionFactorRadToInteger(CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  positionFactorRadToInteger_[index] = factor;
}

void Command::setKpToInteger(rm2_ecat::mit::CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  kpToInteger_[index] = factor;
}

void Command::setKdToInteger(rm2_ecat::mit::CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  kdToInteger_[index] = factor;
}

void Command::setTorqueOffset(rm2_ecat::mit::CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  torqueOffset[index] = factor;
}

void Command::setVelocityOffset(rm2_ecat::mit::CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  velocityOffset[index] = factor;
}

void Command::setPositionOffset(rm2_ecat::mit::CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  positionOffset[index] = factor;
}

void Command::setDigitalOutput(uint8_t id, bool value) {
  digitalOutputs_ &= ~(static_cast<uint8_t>(1) << id);
  digitalOutputs_ |= (static_cast<uint8_t>(value) << id);
}

void Command::setTargetCommand(rm2_ecat::mit::CanBus bus, size_t id, const rm2_ecat::mit::target& target) {
  size_t index = getIndex(bus, id);
  targetPosition_[index] = target.targetPosition;
  targetVelocity_[index] = target.targetVelocity;
  targetKp_[index] = target.kp;
  targetKd_[index] = target.kd;
  targetTorque_[index] = target.targetTorque;
}

uint8_t Command::getDigitalOutputs() const {
  return digitalOutputs_;
}

Command::Command(const Command& other) {
  controlWord_ = other.controlWord_;
  std::copy(std::begin(other.maxOut_), std::end(other.maxOut_), std::begin(maxOut_));
  std::copy(std::begin(other.targetTorque_), std::end(other.targetTorque_), std::begin(targetTorque_));
  std::copy(std::begin(other.targetPosition_), std::end(other.targetPosition_), std::begin(targetPosition_));
  std::copy(std::begin(other.targetVelocity_), std::end(other.targetVelocity_), std::begin(targetVelocity_));
  std::copy(std::begin(other.targetKp_), std::end(other.targetKp_), std::begin(targetKp_));
  std::copy(std::begin(other.targetKd_), std::end(other.targetKd_), std::begin(targetKd_));
  std::copy(std::begin(other.torqueFactorNmToInteger_), std::end(other.torqueFactorNmToInteger_), std::begin(torqueFactorNmToInteger_));
  std::copy(std::begin(other.positionFactorRadToInteger_), std::end(other.positionFactorRadToInteger_), std::begin(positionFactorRadToInteger_));
  std::copy(std::begin(other.velocityFactorRadPerSecToInteger_), std::end(other.velocityFactorRadPerSecToInteger_), std::begin(velocityFactorRadPerSecToInteger_));
  std::copy(std::begin(other.kpToInteger_), std::end(other.kpToInteger_), std::begin(kpToInteger_));
  std::copy(std::begin(other.kdToInteger_), std::end(other.kdToInteger_), std::begin(kdToInteger_));
  digitalOutputs_ = other.digitalOutputs_;
}

Command& Command::operator=(const Command& other) {
  controlWord_ = other.controlWord_;
  std::copy(std::begin(other.maxOut_), std::end(other.maxOut_), std::begin(maxOut_));
  std::copy(std::begin(other.targetTorque_), std::end(other.targetTorque_), std::begin(targetTorque_));
  std::copy(std::begin(other.targetPosition_), std::end(other.targetPosition_), std::begin(targetPosition_));
  std::copy(std::begin(other.targetVelocity_), std::end(other.targetVelocity_), std::begin(targetVelocity_));
  std::copy(std::begin(other.targetKp_), std::end(other.targetKp_), std::begin(targetKp_));
  std::copy(std::begin(other.targetKd_), std::end(other.targetKd_), std::begin(targetKd_));
  std::copy(std::begin(other.torqueFactorNmToInteger_), std::end(other.torqueFactorNmToInteger_), std::begin(torqueFactorNmToInteger_));
  std::copy(std::begin(other.positionFactorRadToInteger_), std::end(other.positionFactorRadToInteger_),
            std::begin(positionFactorRadToInteger_));
  std::copy(std::begin(other.velocityFactorRadPerSecToInteger_), std::end(other.velocityFactorRadPerSecToInteger_),
            std::begin(velocityFactorRadPerSecToInteger_));
  std::copy(std::begin(other.kpToInteger_), std::end(other.kpToInteger_), std::begin(kpToInteger_));
  std::copy(std::begin(other.kdToInteger_), std::end(other.kdToInteger_), std::begin(kdToInteger_));
  digitalOutputs_ = other.digitalOutputs_;
  return *this;
}

uint64_t Command::getMotorCommandRaw(CanBus bus, size_t id) const {
  size_t index = getIndex(bus, id);
  auto tor = static_cast<uint16_t>(torqueFactorNmToInteger_[index] * (targetTorque_[index] - torqueOffset[index]));
  auto vel = static_cast<uint16_t>(velocityFactorRadPerSecToInteger_[index] * (targetVelocity_[index] - velocityOffset[index]));
  auto pos = static_cast<uint16_t>(positionFactorRadToInteger_[index] * (targetPosition_[index] - positionOffset[index]));
  auto kp = static_cast<uint16_t>(targetKp_[index] * kpToInteger_[index]);
  auto kd = static_cast<uint16_t>(targetKd_[index] * kdToInteger_[index]);
  uint64_t cmd = 0;
  uint8_t data[8] = {0};  // Endianness causes data reversal in memcpy, so don't directly modify uint64_t cmd.
  data[0] = (pos >> 8);
  data[1] = pos;
  data[2] = (vel >> 4);
  data[3] = ((vel & 0xF) << 4) | (kp >> 8);
  data[4] = kp;
  data[5] = (kd >> 4);
  data[6] = ((kd & 0xF) << 4) | (tor >> 8);
  data[7] = tor;
  memcpy(&cmd, data, 8);
  return cmd;
}

}  // namespace mit
}  // namespace rm2_ecat
