//
// Created by qiayuan on 23-5-14.
//

#include "rm2_ecat_manager/RmEcatStandardSlaveManager.h"
#include "rm2_msgs/msg/dbus_data.hpp"

namespace rm2_ecat::standard {
std::vector<std::string> RmEcatStandardSlaveManager::getMotorNames() const {
  std::vector<std::string> names;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      names.push_back(reading.getMotorName(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      names.push_back(reading.getMotorName(CanBus::CAN1, id));
    }
  }
  return names;
}

std::vector<double> RmEcatStandardSlaveManager::getMotorPositions() const {
  std::vector<double> positions;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      positions.push_back(reading.getPosition(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      positions.push_back(reading.getPosition(CanBus::CAN1, id));
    }
  }
  return positions;
}

std::vector<double> RmEcatStandardSlaveManager::getMotorVelocities() const {
  std::vector<double> velocities;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      velocities.push_back(reading.getVelocity(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      velocities.push_back(reading.getVelocity(CanBus::CAN1, id));
    }
  }
  return velocities;
}

std::vector<double> RmEcatStandardSlaveManager::getMotorTorque() const {
  std::vector<double> torques;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      torques.push_back(reading.getTorque(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      torques.push_back(reading.getTorque(CanBus::CAN1, id));
    }
  }
  return torques;
}

std::vector<bool> RmEcatStandardSlaveManager::getMotorIsOnlines() const {
  std::vector<bool> isOnlines;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      isOnlines.push_back(reading.getStatusword().isOnline(CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      isOnlines.push_back(reading.getStatusword().isOnline(CanBus::CAN1, id));
    }
  }
  return isOnlines;
}

std::vector<bool> RmEcatStandardSlaveManager::getMotorNeedCalibrations() const {
  std::vector<bool> needCalibrations;
  const auto slaves = getSlaves();
  for (const auto& slave : slaves) {
    const auto reading = slave->getReading();
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      needCalibrations.push_back(slave->getConfiguration().can0MotorConfigurations_.at(id).needCalibration_);
    }
    ids = reading.getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      needCalibrations.push_back(slave->getConfiguration().can1MotorConfigurations_.at(id).needCalibration_);
    }
  }
  return needCalibrations;
}

void RmEcatStandardSlaveManager::stageMotorCommands(const std::vector<double>& commands) {
  auto commandItr = commands.begin();
  for (auto& slave : slaves_) {
    // Avoid overriding commands
    Command command = slave->getStageCommand();
    auto ids = slave->getReading().getEnabledMotorIds(CanBus::CAN0);
    for (const auto& id : ids) {
      command.setTargetCommand(CanBus::CAN0, id, *commandItr++);
      if (commandItr == commands.end()) {
        slave->stageCommand(command);
        return;
      }
    }
    ids = slave->getReading().getEnabledMotorIds(CanBus::CAN1);
    for (const auto& id : ids) {
      command.setTargetCommand(CanBus::CAN1, id, *commandItr++);
      if (commandItr == commands.end()) {
        slave->stageCommand(command);
        return;
      }
    }
    slave->stageCommand(command);
  }
}

void RmEcatStandardSlaveManager::stageZeroCommands() {
  for (auto& slave : slaves_) {
    slave->stageZeroCommand();
  }
}

std::vector<std::string> RmEcatStandardSlaveManager::getImuNames() const {
  std::vector<std::string> names;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    const auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      names.push_back(reading.getImuName(bus));
    }
  }
  return names;
}

std::vector<double> RmEcatStandardSlaveManager::getImuOrientations() const {
  std::vector<double> orientations;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    const auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      double w = 1., x = 0., y = 0., z = 0.;
      reading.getOrientation(bus, w, x, y, z);
      orientations.push_back(x);
      orientations.push_back(y);
      orientations.push_back(z);
      orientations.push_back(w);
    }
  }
  return orientations;
}

std::vector<double> RmEcatStandardSlaveManager::getImuLinearAccelerations() const {
  std::vector<double> accelerations;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    const auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      double x = 0., y = 0., z = 0.;
      reading.getLinearAcceleration(bus, x, y, z);
      accelerations.push_back(x);
      accelerations.push_back(y);
      accelerations.push_back(z);
    }
  }
  return accelerations;
}

std::vector<double> RmEcatStandardSlaveManager::getImuAngularVelocities() const {
  std::vector<double> angularVelocities;
  const auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (const auto& reading : readings) {
    const auto buss = reading.getEnabledImuBuss();
    for (const auto& bus : buss) {
      double x = 0., y = 0., z = 0.;
      reading.getAngularVelocity(bus, x, y, z);
      angularVelocities.push_back(x);
      angularVelocities.push_back(y);
      angularVelocities.push_back(z);
    }
  }
  return angularVelocities;
}

std::vector<std::string> RmEcatStandardSlaveManager::getDigitalInputNames() const {
  std::vector<std::string> names;
  for (const auto& slave : slaves_) {
    auto configs = slave->getConfiguration().gpioConfigurations_;
    for (const auto& config : configs) {
      if (config.second.mode_ == 0) {
        names.push_back(config.second.name_);
      }
    }
  }
  return names;
}

std::vector<bool> RmEcatStandardSlaveManager::getDigitalInputs() const {
  std::vector<bool> inputs;
  auto readings = getReadings<rm2_ecat::standard::Reading>();
  for (auto& reading : readings) {
    const auto ids = reading.getEnabledDigitalInputIds();
    for (const auto& id : ids) {
      inputs.push_back(reading.getDigitalInput(id));
    }
  }
  return inputs;
}

std::vector<std::string> RmEcatStandardSlaveManager::getDigitalOutputNames() const {
  std::vector<std::string> names;
  for (const auto& slave : slaves_) {
    const auto configs = slave->getConfiguration().gpioConfigurations_;
    for (const auto& config : configs) {
      if (config.second.mode_ == 1) {
        names.push_back(config.second.name_);
      }
    }
  }
  return names;
}

std::vector<rm2_msgs::msg::DbusData> RmEcatStandardSlaveManager::getDbusData() const {
  std::vector<rm2_msgs::msg::DbusData> dbusDatas_;
  for (const auto& slave : slaves_) {
    if (slave->getReading().getEnabledDbus()) {
      dbusDatas_.push_back(slave->getReading().getDbusData());
    }
  }
  return dbusDatas_;
}

void RmEcatStandardSlaveManager::stageDigitalOutputs(const std::vector<bool>& outputs) {
  auto outputItr = outputs.begin();
  for (auto& slave : slaves_) {
    Command command = slave->getStageCommand();
    const auto configs = slave->getConfiguration().gpioConfigurations_;
    for (const auto& config : configs) {
      if (config.second.mode_ == 1) {
        command.setDigitalOutput(config.first, *outputItr++);
      }
      if (outputItr == outputs.end()) {
        slave->stageCommand(command);
        return;
      }
    }
  }
}
}  // namespace rm2_ecat::standard
