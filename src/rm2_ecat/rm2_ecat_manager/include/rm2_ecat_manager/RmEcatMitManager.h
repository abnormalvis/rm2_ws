//
// Created by kook on 12/18/23.
//

#pragma once

#include <ecat_manager/EcatSlaveManagerBase.h>
#include <rm2_ecat_mit/RmEcatMitSlave.h>

namespace rm2_ecat::mit {
class RmEcatMitManager : public ecat_manager::EcatSlaveManagerBase<RmEcatMitSlave> {
 public:
  // Setup and configuration
  using EcatSlaveManagerBase::EcatSlaveManagerBase;

  ecat_manager::EcatBusManager::EthercatSlaveType getSlaveType() const override {
    return ecat_manager::EcatBusManager::EthercatSlaveType::Mit;
  }

  // Readings
  std::vector<std::string> getMotorNames() const;
  std::vector<bool> getMotorIsOnlines() const;
  std::vector<bool> getMotorNeedCalibrations() const;
  std::vector<double> getMotorPositions() const;
  std::vector<double> getMotorVelocities() const;
  std::vector<double> getMotorTorque() const;
  std::vector<std::string> getDigitalInputNames() const;
  std::vector<bool> getDigitalInputs() const;
  void checkMotorsIsonline() const;

  // Control
  void stageMotorCommands(const std::vector<target>& commands);
  void stageZeroCommands();
  std::vector<std::string> getDigitalOutputNames() const;
  void stageDigitalOutputs(const std::vector<bool>& outputs);
};
}  // namespace rm2_ecat::mit

