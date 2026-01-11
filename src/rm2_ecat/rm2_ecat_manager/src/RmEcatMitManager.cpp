//
// Created by kook on 12/18/23.
//

#include "rm2_ecat_manager/RmEcatMitManager.h"

namespace rm2_ecat::mit {
std::vector<std::string> RmEcatMitManager::getMotorNames() const {
  std::vector<std::string> names;
  const auto readings = getReadings<rm2_ecat::mit::Reading>();
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

std::vector<double> RmEcatMitManager::getMotorPositions() const {
  std::vector<double> positions;
  const auto readings = getReadings<rm2_ecat::mit::Reading>();
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

std::vector<double> RmEcatMitManager::getMotorVelocities() const {
  std::vector<double> velocities;
  const auto readings = getReadings<rm2_ecat::mit::Reading>();
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

std::vector<double> RmEcatMitManager::getMotorTorque() const {
  std::vector<double> torques;
  const auto readings = getReadings<rm2_ecat::mit::Reading>();
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

std::vector<bool> RmEcatMitManager::getMotorIsOnlines() const {
  std::vector<bool> isOnlines;
  const auto readings = getReadings<rm2_ecat::mit::Reading>();
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

std::vector<bool> RmEcatMitManager::getMotorNeedCalibrations() const {
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

void RmEcatMitManager::stageMotorCommands(const std::vector<target>& commands) {
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

void RmEcatMitManager::stageZeroCommands() {
  for (auto& slave : slaves_) {
    slave->stageZeroCommand();
  }
}

std::vector<std::string> RmEcatMitManager::getDigitalInputNames() const {
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

std::vector<bool> RmEcatMitManager::getDigitalInputs() const {
  std::vector<bool> inputs;
  auto readings = getReadings<rm2_ecat::mit::Reading>();
  for (auto& reading : readings) {
    const auto ids = reading.getEnabledDigitalInputIds();
    for (const auto& id : ids) {
      inputs.push_back(reading.getDigitalInput(id));
    }
  }
  return inputs;
}

std::vector<std::string> RmEcatMitManager::getDigitalOutputNames() const {
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

void RmEcatMitManager::stageDigitalOutputs(const std::vector<bool>& outputs) {
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

void RmEcatMitManager::checkMotorsIsonline() const {
  for (const auto& slave : slaves_) {
    const auto& reading = slave->getReading();
    auto ids = reading.getEnabledMotorIds(CanBus::CAN0);
    size_t online_counts = 0;
    for (const auto id : ids) {
      if (reading.getStatusword().isOnline(CanBus::CAN0, id)) {
        online_counts++;
      }
    }
    //    std::cout << online_counts << std::endl;
    slave->enableMotors();
  }
}

}  // namespace rm2_ecat::mit
