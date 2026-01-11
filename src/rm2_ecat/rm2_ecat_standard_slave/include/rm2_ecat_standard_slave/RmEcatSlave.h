//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include "rm2_ecat_standard_slave/Command.h"
#include "rm2_ecat_standard_slave/Configuration.h"
#include "rm2_ecat_standard_slave/Controlword.h"
#include "rm2_ecat_standard_slave/Reading.h"
#include "rm2_ecat_standard_slave/Sdo.h"

#include <soem_interface_rsl/EthercatSlaveBase.hpp>

#include <yaml-cpp/yaml.h>

#include <condition_variable>

namespace rm2_ecat {
namespace standard {
class RmEcatStandardSlave : public soem_interface_rsl::EthercatSlaveBase {
 public:
  using SharedPtr = std::shared_ptr<RmEcatStandardSlave>;
  void setTimeStep(double timeStep) { timeStep_ = timeStep; }
  void setState(soem_interface_rsl::ETHERCAT_SM_STATE state);
  bool waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, unsigned int maxRetries);

  static SharedPtr deviceFromFile(const std::string& configFile, const std::string& name, uint32_t address);

  // Constructor
  RmEcatStandardSlave() = default;
  RmEcatStandardSlave(const std::string& name, uint32_t address);

  // pure virtual overwrites
  std::string getName() const override { return name_; }
  bool startup() override;
  void shutdown() override;
  void updateWrite() override;
  void updateRead() override;
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

  // Control
  void stageZeroCommand();
  void stageCommand(const Command& command);
  Command getStageCommand();
  // Readings
  Reading getReading() const;
  void getReading(Reading& reading) const;

  // Configuration
  bool loadConfigFile(const std::string& fileName);
  bool loadConfigNode(const YAML::Node& configNode);
  bool loadConfiguration(const Configuration& configuration);
  Configuration getConfiguration() const;

  // SDO
  bool getStatuswordViaSdo(Statusword& statusword);
  void setImuTrigger(CanBus bus, bool imuTrigger);
  void setGpioModes(uint8_t modes);

 private:
  std::string name_;
  double timeStep_{0.0};

  mutable std::mutex stagedCommandMutex_;
  Command stagedCommand_;

  mutable std::mutex readingMutex_;
  Reading reading_;

  Configuration configuration_{};
  Controlword controlword_{};
  PdoInfo pdoInfo_;
};
}  // namespace standard
}  // namespace rm2_ecat
