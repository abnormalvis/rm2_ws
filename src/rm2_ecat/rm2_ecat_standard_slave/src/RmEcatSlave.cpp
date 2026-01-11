//
// Created by qiayuan on 23-5-14.
//

#include "rm2_ecat_standard_slave/RmEcatSlave.h"
#include "rm2_ecat_standard_slave/ConfigurationParser.h"
#include "rm2_ecat_standard_slave/ObjectDictionary.h"
#include "rm2_ecat_standard_slave/RxPdo.h"
#include "rm2_ecat_standard_slave/TxPdo.h"

#include <thread>

namespace rm2_ecat {
namespace standard {
RmEcatStandardSlave::SharedPtr RmEcatStandardSlave::deviceFromFile(const std::string& configFile, const std::string& name,
                                                                   const uint32_t address) {
  auto rm = std::make_shared<RmEcatStandardSlave>(name, address);
  if (!rm->loadConfigFile(configFile)) {
    MELO_ERROR_STREAM("[rm2_ecat_slave:RmEcatSlave::deviceFromFile] loading config file '" << configFile << "' for '" << name
                                                                                          << "' not successful.");
    throw std::runtime_error("[rm2_ecat_slave:RmEcatSlave::deviceFromFile] config file loading error");
  }
  return rm;
}

void RmEcatStandardSlave::setState(soem_interface_rsl::ETHERCAT_SM_STATE state) {
  bus_->setState(state, address_);
}

bool RmEcatStandardSlave::waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, unsigned int maxRetries) {
  return bus_->waitForState(state, address_, maxRetries);
}

RmEcatStandardSlave::RmEcatStandardSlave(const std::string& name, const uint32_t address) {
  address_ = address;
  name_ = name;
}

bool RmEcatStandardSlave::startup() {
  bool success = true;
  success &= waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::PRE_OP, 50);
  //  bus_->syncDistributedClock0(address_, true, timeStep_, timeStep_ / 2.f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  reading_.configureReading(configuration_);
  setGpioModes(configuration_.getGpioModes());

  pdoInfo_.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo_.txPdoSize_ = sizeof(TxPdo);

  if (!success) {
    MELO_ERROR_STREAM("[rm2_ecat_slave:RmEcatSlave::startup] hardware configuration of '" << name_ << "' not successful!");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return success;
}

void RmEcatStandardSlave::shutdown() {
  bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT, address_);
}

void RmEcatStandardSlave::updateWrite() {
  RxPdo rxPdo{};
  {
    std::lock_guard<std::mutex> stagedCmdLock(stagedCommandMutex_);
    rxPdo.controlword_ = controlwordToId(controlword_);

    for (size_t i = 0; i < 8; ++i) {
      rxPdo.can0MotorCommnads_[i] = stagedCommand_.getTargetTorqueRaw(CanBus::CAN0, i + 1);
      rxPdo.can1MotorCommnads_[i] = stagedCommand_.getTargetTorqueRaw(CanBus::CAN1, i + 1);
    }
    rxPdo.digital_outputs_ = stagedCommand_.getDigitalOutputs();
  }

  // actually writing to the hardware
  bus_->writeRxPdo(address_, rxPdo);
}

void RmEcatStandardSlave::updateRead() {
  TxPdo txPdo{};
  // reading from the bus
  bus_->readTxPdo(address_, txPdo);

  Statusword statusword;
  statusword.setRaw(txPdo.statusword_);

  reading_.setStatusword(statusword);
  reading_.setStamp(bus_->getUpdateReadStamp());

  // Motors
  for (size_t i = 0; i < 8; ++i) {
    reading_.setPosition(CanBus::CAN0, i + 1, txPdo.can0MotorPositions_[i]);
    reading_.setVelocity(CanBus::CAN0, i + 1, txPdo.can0MotorVelocities_[i]);
    reading_.setTorque(CanBus::CAN0, i + 1, txPdo.can0MotorCurrents_[i]);
    reading_.setTemperature(CanBus::CAN0, i + 1, txPdo.can0MotorTemperatures_[i]);
    reading_.setPosition(CanBus::CAN1, i + 1, txPdo.can1MotorPositions_[i]);
    reading_.setVelocity(CanBus::CAN1, i + 1, txPdo.can1MotorVelocities_[i]);
    reading_.setTorque(CanBus::CAN1, i + 1, txPdo.can1MotorCurrents_[i]);
    reading_.setTemperature(CanBus::CAN1, i + 1, txPdo.can1MotorTemperatures_[i]);
  }
  // IMUs
  if (statusword.isAngularVelocityUpdated(CanBus::CAN0)) {
    reading_.setAngularVelocity(CanBus::CAN0, txPdo.can0ImuAngularVelocity_);
  }
  if (statusword.isLinearAccelerationUpdated(CanBus::CAN0)) {
    reading_.setLinearAcceleration(CanBus::CAN0, txPdo.can0ImuLinearAcceleration_);
  }
  if (statusword.isAngularVelocityUpdated(CanBus::CAN1)) {
    reading_.setAngularVelocity(CanBus::CAN1, txPdo.can1ImuAngularVelocity_);
  }
  if (statusword.isLinearAccelerationUpdated(CanBus::CAN1)) {
    reading_.setLinearAcceleration(CanBus::CAN1, txPdo.can1ImuLinearAcceleration_);
  }
  // Digital inputs
  reading_.setDigitalInputs(txPdo.digital_inputs_);
  // Dbus data
  reading_.setDbusData(txPdo.dbus_data_1_, txPdo.dbus_data_2_);
}

void RmEcatStandardSlave::stageZeroCommand() {
  Command command;
  for (size_t id = 1; id < 9; ++id) {
    command.setTargetCommand(CanBus::CAN0, id, 0);
    command.setTargetCommand(CanBus::CAN1, id, 0);
  }
  stageCommand(command);
}

void RmEcatStandardSlave::stageCommand(const Command& command) {
  std::lock_guard<std::mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  for (const auto& [id, motorConfiguration] : configuration_.can0MotorConfigurations_) {
    stagedCommand_.setMaxOut(CanBus::CAN0, id, motorConfiguration.maxOut_);
    stagedCommand_.setTorqueFactorNmToInteger(CanBus::CAN0, id, motorConfiguration.torqueFactorNmToInteger_);
  }
  for (const auto& [id, motorConfiguration] : configuration_.can1MotorConfigurations_) {
    stagedCommand_.setMaxOut(CanBus::CAN1, id, motorConfiguration.maxOut_);
    stagedCommand_.setTorqueFactorNmToInteger(CanBus::CAN1, id, motorConfiguration.torqueFactorNmToInteger_);
  }
}

Command RmEcatStandardSlave::getStageCommand() {
  std::lock_guard<std::mutex> lock(stagedCommandMutex_);
  return stagedCommand_;
}

Reading RmEcatStandardSlave::getReading() const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  return reading_;
}

void RmEcatStandardSlave::getReading(Reading& reading) const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  reading = reading_;
}

bool RmEcatStandardSlave::loadConfigFile(const std::string& fileName) {
  ConfigurationParser configurationParser(fileName);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool RmEcatStandardSlave::loadConfigNode(const YAML::Node& configNode) {
  ConfigurationParser configurationParser(configNode);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool RmEcatStandardSlave::loadConfiguration(const Configuration& configuration) {
  configuration_ = configuration;
  MELO_INFO_STREAM("Configuration Sanity Check of rm '" << getName() << "':");
  return configuration_.sanityCheck();
}

Configuration RmEcatStandardSlave::getConfiguration() const {
  return configuration_;
}

bool RmEcatStandardSlave::getStatuswordViaSdo(Statusword& statusword) {
  uint32_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setRaw(statuswordValue);
  return success;
}

void RmEcatStandardSlave::setImuTrigger(CanBus bus, bool imuTrigger) {
  if (bus == CanBus::CAN0) {
    sendSdoWrite(OD_INDEX_CAN0_IMU_TRIGGER, 0, false, static_cast<uint8_t>(imuTrigger));
  } else if (bus == CanBus::CAN1) {
    sendSdoWrite(OD_INDEX_CAN1_IMU_TRIGGER, 0, false, static_cast<uint8_t>(imuTrigger));
  }
}

void RmEcatStandardSlave::setGpioModes(uint8_t mode) {
  sendSdoWrite(OD_INDEX_GPIO_MODES, 0, false, mode);
}
}  // namespace standard
}  // namespace rm2_ecat
