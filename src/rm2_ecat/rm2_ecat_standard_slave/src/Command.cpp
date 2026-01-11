//
// Created by qiayuan on 23-5-14.
//
#include "rm2_ecat_standard_slave/Command.h"

namespace rm2_ecat {
namespace standard {
void Command::setMaxOut(CanBus bus, size_t id, int16_t factor) {
  size_t index = getIndex(bus, id);
  maxOut_[index] = factor;
}

void Command::setTorqueFactorNmToInteger(CanBus bus, size_t id, double factor) {
  size_t index = getIndex(bus, id);
  torqueFactorNmToInteger_[index] = factor;
}

void Command::setDigitalOutput(uint8_t id, bool value) {
  digitalOutputs_ &= ~(static_cast<uint8_t>(1) << id);
  digitalOutputs_ |= (static_cast<uint8_t>(value) << id);
}

void Command::setTargetCommand(CanBus bus, size_t id, double targetTorque) {
  targetTorque_[getIndex(bus, id)] = targetTorque;
}

int16_t Command::getTargetTorqueRaw(CanBus bus, size_t id) const {
  size_t index = getIndex(bus, id);

  auto torque = targetTorque_[index] * torqueFactorNmToInteger_[index];
  torque = std::max(static_cast<double>(-maxOut_[index]), std::min(torque, static_cast<double>(maxOut_[index])));
  return static_cast<int16_t>(torque);
}

uint8_t Command::getDigitalOutputs() const {
  return digitalOutputs_;
}

Command::Command(const Command& other) {
  controlWord_ = other.controlWord_;
  std::copy(std::begin(other.targetTorque_), std::end(other.targetTorque_), std::begin(targetTorque_));
  std::copy(std::begin(other.torqueFactorNmToInteger_), std::end(other.torqueFactorNmToInteger_), std::begin(torqueFactorNmToInteger_));
  digitalOutputs_ = other.digitalOutputs_;
}

Command& Command::operator=(const Command& other) {
  controlWord_ = other.controlWord_;
  std::copy(std::begin(other.targetTorque_), std::end(other.targetTorque_), std::begin(targetTorque_));
  std::copy(std::begin(other.torqueFactorNmToInteger_), std::end(other.torqueFactorNmToInteger_), std::begin(torqueFactorNmToInteger_));
  digitalOutputs_ = other.digitalOutputs_;
  return *this;
}
}  // namespace standard

}  // namespace rm2_ecat
