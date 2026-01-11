//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <ecat_manager/EcatSlaveManagerBase.h>
#include <rm2_ecat_standard_slave/RmEcatSlave.h>
#include "rm2_msgs/msg/dbus_data.hpp"

namespace rm2_ecat::standard {
class RmEcatStandardSlaveManager : public ecat_manager::EcatSlaveManagerBase<RmEcatStandardSlave> {
 public:
  // Setup and configuration
  using EcatSlaveManagerBase::EcatSlaveManagerBase;

  ecat_manager::EcatBusManager::EthercatSlaveType getSlaveType() const override {
    return ecat_manager::EcatBusManager::EthercatSlaveType::Rm;
  }

  // Readings
  std::vector<std::string> getMotorNames() const;
  std::vector<bool> getMotorIsOnlines() const;
  std::vector<bool> getMotorNeedCalibrations() const;
  std::vector<double> getMotorPositions() const;
  std::vector<double> getMotorVelocities() const;
  std::vector<double> getMotorTorque() const;
  std::vector<std::string> getImuNames() const;
  std::vector<double> getImuOrientations() const;  // TODO: add structure of orientation (maybe should change the interface of Reading)
  std::vector<double> getImuLinearAccelerations() const;  // TODO: ~
  std::vector<double> getImuAngularVelocities() const;    // TODO: ~
  std::vector<std::string> getDigitalInputNames() const;
  std::vector<bool> getDigitalInputs() const;
  std::vector<rm2_msgs::msg::DbusData> getDbusData() const;

  // Control
  void stageMotorCommands(const std::vector<double>& commands);
  void stageZeroCommands();
  std::vector<std::string> getDigitalOutputNames() const;
  void stageDigitalOutputs(const std::vector<bool>& outputs);
};
}  // namespace rm2_ecat::standard
