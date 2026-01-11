//
// Created by qiayuan on 23-4-11.
//

#pragma once

#include <signal_handler/SignalHandler.hpp>

namespace ecat_manager {

template <typename SlaveType>
EcatSlaveManagerBase<SlaveType>::EcatSlaveManagerBase(bool standalone, bool installSignalHandler, double timeStep)
    : standalone_(standalone), installSignalHandler_(installSignalHandler), timeStep_(timeStep), shutdownRequested_(false) {
  if (standalone_) {
    updateWorker_ = std::make_shared<any_worker::Worker>("UpdateWorker", timeStep_,
                                                         std::bind(&EcatSlaveManagerBase::updateWorkerCb, this, std::placeholders::_1));
  }
}

template <typename SlaveType>
EcatSlaveManagerBase<SlaveType>::~EcatSlaveManagerBase() {
  if (installSignalHandler_) {
    signal_handler::SignalHandler::unbindAll(&EcatSlaveManagerBase<SlaveType>::handleSignal, this);
  }
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  if (standalone_) {
    updateWorker_->setTimestep(timeStep_);
  }
}

template <typename SlaveType>
bool EcatSlaveManagerBase<SlaveType>::slaveExists(const std::string& name) const {
  for (const auto& slave : slaves_) {
    if (slave->getName() == name) {
      return true;
    }
  }
  return false;
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::setBusManager(const EcatBusManagerPtr& busManager) {
  busManager_ = busManager;
  auto slaves = busManager_->getSlavesOfType<SlaveType>(getSlaveType());
  for (const auto& slave : slaves) {
    slave->setTimeStep(timeStep_);
    addSlave(slave);
  }
}

template <typename SlaveType>
bool EcatSlaveManagerBase<SlaveType>::addSlave(const SlavePtr& slave) {
  if (slaveExists(slave->getName())) {
    return false;
  }
  slaves_.push_back(slave);

  return true;
}

template <typename SlaveType>
std::shared_ptr<SlaveType> EcatSlaveManagerBase<SlaveType>::getSlave(const std::string& name) const {
  for (const auto& slave : slaves_) {
    if (slave->getName() == name) {
      return slave;
    }
  }
  return nullptr;
}

template <typename SlaveType>
std::vector<std::string> EcatSlaveManagerBase<SlaveType>::getNamesOfSlaves() const {
  std::vector<std::string> names;
  for (const auto& slave : slaves_) {
    names.push_back(slave->getName());
  }
  return names;
}

template <typename SlaveType>
bool EcatSlaveManagerBase<SlaveType>::startup() {
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);

  bool success = true;

  if (installSignalHandler_) {
    signal_handler::SignalHandler::bindAll(&EcatSlaveManagerBase<SlaveType>::handleSignal, this);
  }

  if (standalone_) {
    success &= busManager_->startupCommunication();
    if (!success) {
      return success;
    }
  }

  for (const auto& slave : slaves_) {
    if (!slave->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP, 50)) {
      MELO_ERROR_STREAM(slave->getName() << " not in safe op after startup!");
    }
    slave->setState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL);
    success &= slave->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL, 50);
  }

  if (standalone_) {
    success &= updateWorker_->start(48);  // do not set above 48, otherwise starve kernel processes on which SOEM depends.
  }

  isRunning_ = true;
  shutdownRequested_ = false;

  if (!success) MELO_ERROR("[EcatSlaveManagerBase::startup] Startup not successful.");
  return success;
}

template <typename SlaveType>
bool EcatSlaveManagerBase<SlaveType>::update() {
  updateCommunicationManagerReadMessages();
  updateProcessReadings();
  updateSendStagedCommands();
  updateCommunicationManagerWriteMessages();
  return true;
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::updateCommunicationManagerReadMessages() {
  busManager_->readAllBuses();
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::updateProcessReadings() {}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::updateSendStagedCommands() {}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::updateCommunicationManagerWriteMessages() {
  busManager_->writeToAllBuses();
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::shutdown() {
  if (shutdownRequested_) {
    return;
  }
  requestShutdown();
}

template <typename SlaveType>
template <typename CommandType>
void EcatSlaveManagerBase<SlaveType>::stageCommands(const std::vector<CommandType>& commands) {
  for (size_t i = 0; i < commands.size() && i < slaves_.size(); ++i) {
    slaves_[i]->stageCommand(commands[i]);
  }
}

template <typename SlaveType>
template <typename ReadingType>
std::vector<ReadingType> EcatSlaveManagerBase<SlaveType>::getReadings() const {
  std::vector<ReadingType> readings;
  for (const auto& slave : slaves_) {
    readings.push_back(slave->getReading());
  }
  return readings;
}

template <typename SlaveType>
bool EcatSlaveManagerBase<SlaveType>::updateWorkerCb(const any_worker::WorkerEvent& /*event*/) {
  return update();
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::requestShutdown() {
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  isRunning_ = false;
  shutdownRequested_ = true;

  if (standalone_) {
    updateWorker_->stop();
    busManager_->shutdownAllBuses();
  } else {
    for (const auto& slave : slaves_) {
      slave->shutdown();
    }
  }

  MELO_INFO("[EcatSlaveManagerBase::requestShutdown] Shutdown.");
}

template <typename SlaveType>
void EcatSlaveManagerBase<SlaveType>::handleSignal(int /*signum*/) {
  requestShutdown();
}

}  // namespace ecat_manager