//
// Created by qiayuan on 23-4-11.
//

#pragma once

namespace cleardrive_ros {

template <typename SlaveManagerType>
ClearSlaveManagerRosBase<SlaveManagerType>::ClearSlaveManagerRosBase(bool standalone, bool installSignalHandler, double timeStep,
                                                                     std::shared_ptr<rclcpp::Node> node, std::string& rosPrefix)
    : SlaveManagerType(standalone, installSignalHandler, timeStep), node_(node), rosPrefix_(rosPrefix), shutdownPublishWorkerRequested_(false) {
  if (standalone) {
    publishWorker_ = std::make_shared<any_worker::Worker>(
        "PublishWorker", 10 * timeStep, std::bind(&ClearSlaveManagerRosBase::publishWorkerCb, this, std::placeholders::_1));
  }
}

template <typename SlaveManagerType>
bool ClearSlaveManagerRosBase<SlaveManagerType>::startup() {
  if (!SlaveManagerType::startup()) {
    return false;
  }
  startupRosInterface();
  return true;
}

template <typename SlaveManagerType>
void ClearSlaveManagerRosBase<SlaveManagerType>::shutdown() {
  shutdownRosInterface();
  SlaveManagerType::shutdown();
}

template <typename SlaveManagerType>
bool ClearSlaveManagerRosBase<SlaveManagerType>::publishWorkerCb(const any_worker::WorkerEvent& /*event*/) {
  sendRos();
  return true;
}

}  // namespace cleardrive_ros
