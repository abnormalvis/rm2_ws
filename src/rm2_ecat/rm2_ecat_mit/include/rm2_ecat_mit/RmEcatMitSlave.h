//
// Created by kook on 12/29/23.
//

//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include "rm2_ecat_mit/Command.h"
#include "rm2_ecat_mit/Configuration.h"
#include "rm2_ecat_mit/Controlword.h"
#include "rm2_ecat_mit/Reading.h"

#include <soem_interface_rsl/EthercatSlaveBase.hpp>

#include <yaml-cpp/yaml.h>

#include <condition_variable>

namespace rm2_ecat {
namespace mit {
class RmEcatMitSlave : public soem_interface_rsl::EthercatSlaveBase {
 public:
  using SharedPtr = std::shared_ptr<RmEcatMitSlave>;
  void setTimeStep(double timeStep) { timeStep_ = timeStep; }
  void setState(soem_interface_rsl::ETHERCAT_SM_STATE state);
  bool waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, unsigned int maxRetries);

  static SharedPtr deviceFromFile(const std::string& configFile, const std::string& name, uint32_t address);

  // Constructor
  RmEcatMitSlave() = default;
  RmEcatMitSlave(const std::string& name, uint32_t address);

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
  void enableMotors();
  void disableMotors();
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
  bool setGpioModes(uint8_t modes);

  // Controlword
  void setControlWord(Controlword controlword);

 private:
  std::string name_;
  double timeStep_{0.0};

  mutable std::mutex stagedCommandMutex_;
  Command stagedCommand_;

  mutable std::mutex readingMutex_;
  Reading reading_;

  mutable std::mutex controlwordMutex_;
  Controlword controlword_{};

  Configuration configuration_{};
  PdoInfo pdoInfo_;
};
}  // namespace mit
}  // namespace rm2_ecat

