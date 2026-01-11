//
// Created by kook on 12/29/23.
//

#include "rm2_ecat_mit/ConfigurationParser.h"
#include "rm2_ecat_mit/ObjectDictionary.h"
#include "rm2_ecat_mit/RmEcatMitSlave.h"
#include "rm2_ecat_mit/RxPdo.h"
#include "rm2_ecat_mit/TxPdo.h"

#include <thread>

namespace rm2_ecat {
namespace mit {
RmEcatMitSlave::SharedPtr RmEcatMitSlave::deviceFromFile(const std::string& configFile, const std::string& name,
                                                                   const uint32_t address) {
  auto rm = std::make_shared<RmEcatMitSlave>(name, address);
  if (!rm->loadConfigFile(configFile)) {
    MELO_ERROR_STREAM("[rm2_ecat_slave::mit::RmEcatMitSlave::deviceFromFile] loading config file '" << configFile << "' for '" << name
                                                                                          << "' not successful.");
    throw std::runtime_error("[rm2_ecat_slave::mit::RmEcatMitSlave::deviceFromFile] config file loading error");
  }
  return rm;
}

void RmEcatMitSlave::setState(soem_interface_rsl::ETHERCAT_SM_STATE state) {
  bus_->setState(state, address_);
}

bool RmEcatMitSlave::waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, unsigned int maxRetries) {
  return bus_->waitForState(state, address_, maxRetries);
}

RmEcatMitSlave::RmEcatMitSlave(const std::string& name, const uint32_t address) {
  address_ = address;
  name_ = name;
}

bool RmEcatMitSlave::startup() {
  bool success = true;
  success &= waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::PRE_OP, 50);
  //  bus_->syncDistributedClock0(address_, true, timeStep_, timeStep_ / 2.f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  reading_.configureReading(configuration_);
  success &= setGpioModes(configuration_.getGpioModes());
  enableMotors();

  pdoInfo_.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo_.txPdoSize_ = sizeof(TxPdo);

  if (!success) {
    MELO_ERROR_STREAM("[rm2_ecat_slave::mit::RmEcatMitSlave::startup] hardware configuration of '" << name_ << "' not successful!");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return success;
}

void RmEcatMitSlave::shutdown() {
  disableMotors();
  bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT, address_);
}

void RmEcatMitSlave::updateWrite() {
  RxPdo rxPdo{};
  {
    std::lock_guard<std::mutex> controlwordLock(controlwordMutex_);
    rxPdo.controlword_ = controlwordToId(controlword_);
  }
  {
    std::lock_guard<std::mutex> stagedCmdLock(stagedCommandMutex_);
    for (size_t i = 0; i < motorNumEachBus; ++i) {
      rxPdo.can0Commnads_[i] = stagedCommand_.getMotorCommandRaw(CanBus::CAN0, i + 1);
      rxPdo.can1Commnads_[i] = stagedCommand_.getMotorCommandRaw(CanBus::CAN1, i + 1);
    }
    rxPdo.digitalOutputs_ = stagedCommand_.getDigitalOutputs();
  }
  // actually writing to the hardware
  bus_->writeRxPdo(address_, rxPdo);
}

void RmEcatMitSlave::updateRead() {
  TxPdo txPdo{};
  // reading from the bus
  bus_->readTxPdo(address_, txPdo);

  Statusword statusword;
  statusword.setRaw(txPdo.statusword_);

  reading_.setStatusword(statusword);
  reading_.setStamp(bus_->getUpdateReadStamp());

  // Motors
  for (size_t i = 0; i < motorNumEachBus; ++i) {
    reading_.setRawReading(CanBus::CAN0, i + 1, txPdo.can0Measurement_[i]);
    reading_.setRawReading(CanBus::CAN1, i + 1, txPdo.can1Measurement_[i]);
  }
  // Digital inputs
  reading_.setDigitalInputs(txPdo.digitalInputs_);
}

void RmEcatMitSlave::stageZeroCommand() {
  Command command;
  target targetCommand{0., 0., 0., 0., 0.};
  for (size_t id = 1; id < 9; ++id) {
    command.setTargetCommand(CanBus::CAN0, id, targetCommand);
    command.setTargetCommand(CanBus::CAN1, id, targetCommand);
  }
  stageCommand(command);
}

void RmEcatMitSlave::stageCommand(const Command& command) {
  std::lock_guard<std::mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  for (const auto& [id, motorConfiguration] : configuration_.can0MotorConfigurations_) {
    stagedCommand_.setMaxOut(CanBus::CAN0, id, motorConfiguration.maxOut_);
    stagedCommand_.setTorqueFactorNmToInteger(CanBus::CAN0, id, motorConfiguration.torqueFactorNmToInteger_);
    stagedCommand_.setPositionFactorRadToInteger(CanBus::CAN0, id, motorConfiguration.positionFactorRadToInteger_);
    stagedCommand_.setVelocityFactorRadPerSecToInteger(CanBus::CAN0, id, motorConfiguration.velocityFactorRadPerSecToInteger_);
    stagedCommand_.setKpToInteger(CanBus::CAN0, id, motorConfiguration.kpFactorToInteger);
    stagedCommand_.setKdToInteger(CanBus::CAN0, id, motorConfiguration.kdFactorToInteger);
    stagedCommand_.setTorqueOffset(CanBus::CAN0, id, motorConfiguration.torqueOffset);
    stagedCommand_.setPositionOffset(CanBus::CAN0, id, motorConfiguration.positionOffset);
    stagedCommand_.setVelocityOffset(CanBus::CAN0, id, motorConfiguration.velocityOffset);
  }
  for (const auto& [id, motorConfiguration] : configuration_.can1MotorConfigurations_) {
    stagedCommand_.setMaxOut(CanBus::CAN1, id, motorConfiguration.maxOut_);
    stagedCommand_.setTorqueFactorNmToInteger(CanBus::CAN1, id, motorConfiguration.torqueFactorNmToInteger_);
    stagedCommand_.setPositionFactorRadToInteger(CanBus::CAN1, id, motorConfiguration.positionFactorRadToInteger_);
    stagedCommand_.setVelocityFactorRadPerSecToInteger(CanBus::CAN1, id, motorConfiguration.velocityFactorRadPerSecToInteger_);
    stagedCommand_.setKpToInteger(CanBus::CAN1, id, motorConfiguration.kpFactorToInteger);
    stagedCommand_.setKdToInteger(CanBus::CAN1, id, motorConfiguration.kdFactorToInteger);
    stagedCommand_.setTorqueOffset(CanBus::CAN1, id, motorConfiguration.torqueOffset);
    stagedCommand_.setPositionOffset(CanBus::CAN1, id, motorConfiguration.positionOffset);
    stagedCommand_.setVelocityOffset(CanBus::CAN1, id, motorConfiguration.velocityOffset);
  }
}

Command RmEcatMitSlave::getStageCommand() {
  std::lock_guard<std::mutex> lock(stagedCommandMutex_);
  return stagedCommand_;
}

Reading RmEcatMitSlave::getReading() const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  return reading_;
}

void RmEcatMitSlave::getReading(Reading& reading) const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  reading = reading_;
}

bool RmEcatMitSlave::loadConfigFile(const std::string& fileName) {
  ConfigurationParser configurationParser(fileName);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool RmEcatMitSlave::loadConfigNode(const YAML::Node& configNode) {
  ConfigurationParser configurationParser(configNode);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool RmEcatMitSlave::loadConfiguration(const Configuration& configuration) {
  configuration_ = configuration;
  MELO_INFO_STREAM("Configuration Sanity Check of mit '" << getName() << "':");
  return configuration_.sanityCheck();
}

Configuration RmEcatMitSlave::getConfiguration() const {
  return configuration_;
}

bool RmEcatMitSlave::getStatuswordViaSdo(Statusword& statusword) {
  uint32_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setRaw(statuswordValue);
  return success;
}

bool RmEcatMitSlave::setGpioModes(uint8_t mode) {
  return sendSdoWrite(OD_INDEX_GPIO_MODES, 0, false, mode);
}

void RmEcatMitSlave::setControlWord(rm2_ecat::mit::Controlword controlword) {
  controlword_ = controlword;
}

void RmEcatMitSlave::enableMotors() {
  std::lock_guard<std::mutex> lock(controlwordMutex_);
  controlword_ = Controlword::DisableToEnable;
}

void RmEcatMitSlave::disableMotors() {
  std::lock_guard<std::mutex> lock(controlwordMutex_);
  controlword_ = Controlword::EnableToDisable;
}
}  // namespace mit
}  // namespace rm2_ecat
