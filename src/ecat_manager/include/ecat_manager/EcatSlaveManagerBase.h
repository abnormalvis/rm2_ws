//
// Created by qiayuan on 23-4-11.
//

#pragma once

#include <unistd.h>
#include <any_worker/Worker.hpp>
#include <mutex>

#include "ecat_manager/EcatBusManager.h"

namespace ecat_manager {

class EcatBusManager;
using EcatBusManagerPtr = std::shared_ptr<EcatBusManager>;

template <typename SlaveType>
class EcatSlaveManagerBase {
  using SlavePtr = std::shared_ptr<SlaveType>;
  using Slaves = std::vector<SlavePtr>;

 public:
  // Setup and configuration
  EcatSlaveManagerBase(bool standalone, bool installSignalHandler, double timeStep);
  virtual ~EcatSlaveManagerBase();
  virtual ecat_manager::EcatBusManager::EthercatSlaveType getSlaveType() const = 0;

  void setTimeStep(double timeStep);
  double getTimeStep() const { return timeStep_; }
  void setBusManager(const EcatBusManagerPtr& busManager);
  bool addSlave(const SlavePtr& slave);
  bool slaveExists(const std::string& name) const;
  SlavePtr getSlave(const std::string& name) const;
  Slaves getSlaves() const { return slaves_; }
  unsigned int getNumberOfSlaves() const { return slaves_.size(); }
  std::vector<std::string> getNamesOfSlaves() const;

  // Execution
  virtual bool startup();
  virtual bool update();
  virtual void updateCommunicationManagerReadMessages();
  virtual void updateProcessReadings();
  virtual void updateSendStagedCommands();
  virtual void updateCommunicationManagerWriteMessages();
  virtual void shutdown();
  bool isRunning() const { return isRunning_; }
  bool shutdownRequested() const { return shutdownRequested_; }

  // Control
  template <typename CommandType>
  void stageCommands(const std::vector<CommandType>& commands);
  template <typename ReadingType>
  std::vector<ReadingType> getReadings() const;

 protected:
  bool updateWorkerCb(const any_worker::WorkerEvent& event);
  void requestShutdown();
  void handleSignal(int signum);

  bool standalone_ = true;
  bool installSignalHandler_ = true;
  std::shared_ptr<any_worker::Worker> updateWorker_;
  double timeStep_ = 0.0;
  std::recursive_mutex isRunningMutex_;
  bool isRunning_ = false;
  std::atomic<bool> shutdownRequested_;

  EcatBusManagerPtr busManager_;
  Slaves slaves_;
};

}  // namespace ecat_manager

#include "ecat_manager/EcatSlaveManagerBaseImpl.h"