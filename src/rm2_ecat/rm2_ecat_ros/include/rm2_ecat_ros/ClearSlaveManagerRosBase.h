//
// Created by qiayuan on 23-4-15.
//

#pragma once

namespace cleardrive_ros {

template <typename SlaveManagerType>
class ClearSlaveManagerRosBase : public SlaveManagerType {
 public:
  ClearSlaveManagerRosBase(bool standalone, bool installSignalHandler, double timeStep, std::shared_ptr<rclcpp::Node> node, std::string& rosPrefix);

  bool startup() override;
  void shutdown() override;
  virtual void sendRos() = 0;

 protected:
  virtual void startupRosInterface() = 0;
  virtual void shutdownRosInterface() = 0;
  bool publishWorkerCb(const any_worker::WorkerEvent& event);

  std::shared_ptr<rclcpp::Node> node_;
  std::string rosPrefix_;
  
  const unsigned int maxPublishMessageBufferSize_ = 10;
  std::shared_ptr<any_worker::Worker> publishWorker_;
  std::mutex notifyPublishWorkerMutex_;
  std::condition_variable notifyPublishWorkerCv_;
  std::atomic<bool> shutdownPublishWorkerRequested_;
};

}  // namespace cleardrive_ros

#include "rm2_ecat_ros/ClearSlaveManagerRosBaseImpl.h"