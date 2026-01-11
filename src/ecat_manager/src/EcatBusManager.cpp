//
// Created by qiayuan on 23-4-10.
//

#include "ecat_manager/EcatBusManager.h"

#ifdef _CLEAR_FOC_FOUND_
#include <cleardrive_foc/ClearFocSlave.h>
#endif
#ifdef _CLEAR_IMU_FOUND_
#include <cleardrive_imu/ClearImuSlave.h>
#endif
#ifdef _MIT_FOUND_
#include <rm2_ecat_mit/RmEcatMitSlave.h>
#endif
#ifdef _RM_FOUND_
#include <rm2_ecat_standard_slave/RmEcatSlave.h>
#endif

#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace ecat_manager {

static bool pathExists(std::string& path) {
  return std::filesystem::exists(path);
}

bool EcatBusManager::fromFile(const std::string& file, bool startup) {
  setupFilepath_ = file;
  parseFile(file);
  setup(startup);
  return true;
}

const EcatBusManager::EthercatSlaveEntry& EcatBusManager::getInfoForSlave(
    const std::shared_ptr<soem_interface_rsl::EthercatSlaveBase>& slave) {
  return slave2Entry_[slave];
}

std::set<std::string> EcatBusManager::getAllBusNames() {
    return busNames_;
}


void EcatBusManager::parseFile(std::string path) {
  if (!pathExists(path)) {
    throw std::runtime_error("[EcatBusManager] File not found: " + path);
  }
  YAML::Node node = YAML::LoadFile(path);

  if (node["ethercat_master"].IsDefined()) {
    const auto masterNode = node["ethercat_master"];
    if (masterNode["time_step"].IsDefined()) {
      timeStep_ = masterNode["time_step"].as<double>();
    } else {
      throw std::runtime_error("[EthercatDeviceConfigurator] Node time_step missing in ethercat_master");
    }
  } else {
    throw std::runtime_error("[EthercatDeviceConfigurator] Node ethercat_master is missing in yaml");
  }

  if (node["ethercat_devices"].IsDefined()) {
    // Get all children
    const YAML::Node& nodes = node["ethercat_devices"];
    if (nodes.size() == 0) {
      throw std::runtime_error("[EcatBusManager] No devices defined in yaml");
    }

    // Iterate through child nodes
    for (YAML::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
      const YAML::Node& child = *it;
      EthercatSlaveEntry entry;
      // type
      if (child["type"].IsDefined()) {
        auto type_str = child["type"].as<std::string>();

        if (type_str == "ClearFoc") {
          entry.type = EthercatSlaveType::ClearFoc;
        } else if (type_str == "ClearImu") {
          entry.type = EthercatSlaveType::ClearImu;
        } else if (type_str == "Mit") {
          entry.type = EthercatSlaveType::Mit;
        } else if (type_str == "Rm") {
          entry.type = EthercatSlaveType::Rm;
        } else {
          throw std::runtime_error("[EcatBusManager] " + type_str + " is an undefined type of ethercat device");
        }
      } else {
        throw std::runtime_error("[EcatBusManager] Node: " + child.Tag() + " has no entry type");
      }

      // name
      if (child["name"].IsDefined()) {
        entry.name = child["name"].as<std::string>();
      } else {
        throw std::runtime_error("[EcatBusManager] Node: " + child.Tag() + " has no entry name");
      }

      // configuration_file
      if (child["configuration_file"].IsDefined()) {
        entry.configFilePath = child["configuration_file"].as<std::string>();
      } else {
        throw std::runtime_error("[EcatBusManager] Node: " + child.Tag() + " has no entry configuration_file");
      }

      // ethercat_bus_address
      if (child["ethercat_address"].IsDefined()) {
        entry.ethercatAddress = child["ethercat_address"].as<int>();
      } else {
        throw std::runtime_error("[EcatBusManager] Node: " + child.Tag() + " has no entry ethercat_bus_address");
      }

      // ethercat_bus
      if (child["ethercat_bus"].IsDefined()) {
        entry.ethercatBus = child["ethercat_bus"].as<std::string>();
      } else {
        throw std::runtime_error("[EcatBusManager] Node: " + child.Tag() + " has no entry ethercat_bus");
      }
      slaveEntries_.push_back(entry);
    }
  } else {
    throw std::runtime_error("[EcatBusManager] Node ethercat_devices missing in yaml");
  }
}

void EcatBusManager::setup(bool startup) {
  std::vector<std::unique_ptr<soem_interface_rsl::EthercatBusBase>> ethercatBuses;
  for (auto& entry : slaveEntries_) {
    bool busFound = false;
    soem_interface_rsl::EthercatBusBase* correspondingBus = nullptr;
    busNames_.insert(entry.ethercatBus);

    for (auto& bus : ethercatBuses) {
      if (bus->getName() == entry.ethercatBus) {
        busFound = true;
        correspondingBus = bus.get();
        break;
      }
    }
    if (!busFound) {
        ethercatBuses.push_back(std::make_unique<soem_interface_rsl::EthercatBusBase>(entry.ethercatBus));
        correspondingBus = ethercatBuses.back().get();
    }

    MELO_DEBUG_STREAM("[EcatBusManager] Creating slave: " << entry.name);

    std::shared_ptr<soem_interface_rsl::EthercatSlaveBase> slave;
    std::string configurationFilePath = handleFilePath(entry.configFilePath, setupFilepath_);

    switch (entry.type) {
        case EthercatSlaveType::ClearFoc: {
#ifdef _CLEAR_FOC_FOUND_
            auto foc = cleardrive::foc::ClearFocSlave::deviceFromFile(configurationFilePath, entry.name, entry.ethercatAddress);
            foc->setTimeStep(timeStep_);
            slave = foc;
#else
            throw std::runtime_error("cleardrive_foc not availabe.");
#endif
            break;
        }
        case EthercatSlaveType::ClearImu: {
#ifdef _CLEAR_IMU_FOUND_
            auto imu = cleardrive::imu::ClearImuSlave::deviceFromFile(configurationFilePath, entry.name, entry.ethercatAddress);
            imu->setTimeStep(timeStep_);
            slave = imu;
#else
            throw std::runtime_error("cleardrive_imu not availabe.");
#endif
            break;
        }
        case EthercatSlaveType::Mit: {
#ifdef _MIT_FOUND_
            auto mit = rm2_ecat::mit::RmEcatMitSlave::deviceFromFile(configurationFilePath, entry.name, entry.ethercatAddress);
            mit->setTimeStep(timeStep_);
            slave = mit;
#else
            throw std::runtime_error("rm2_ecat_mit_slave not availabe.");
#endif
            break;
        }
        case EthercatSlaveType::Rm: {
#ifdef _RM_FOUND_
            auto rm = rm2_ecat::standard::RmEcatStandardSlave::deviceFromFile(configurationFilePath, entry.name, entry.ethercatAddress);
            rm->setTimeStep(timeStep_);
            slave = rm;
#else
            throw std::runtime_error("rm2_ecat_standard_slave not availabe.");
#endif
            break;
        }
        default:
            throw std::runtime_error("[EcatBusManager] Not existing EthercatSlaveType passed");
    }

    if (!correspondingBus->addSlave(slave)) {
      throw std::runtime_error("[EcatBusManager] could not add slave: " + slave->getName() +
                               " to bus on interface: " + correspondingBus->getName());
    }
    slave->setEthercatBusBasePointer(correspondingBus);

    slaves_.push_back(slave);
    slave2Entry_.insert({slave, entry});
  }

  for (auto& bus : ethercatBuses) {
    addEthercatBus(std::move(bus));
  }

  if (startup) {
    startupAllBuses();
  }
}

std::string EcatBusManager::handleFilePath(const std::string& path, const std::string& setup_file_path) {
  std::string resultPath;
  if (path.front() == '/') {
    resultPath = path;
    // Path to the configuration file is absolute, we can use it as is.
  } else if (path.front() == '~') {
    // Path to the configuration file is absolute, we need to replace '~' with the home directory.
    const char* homeDirectory = getenv("HOME");
    if (homeDirectory == nullptr) {
      throw std::runtime_error("[ClearConfigurator] Environment variable 'HOME' could not be evaluated.");
    }
    resultPath = path;
    resultPath.erase(resultPath.begin());
    resultPath = homeDirectory + resultPath;
  } else {
    // Path to the configuration file is relative, we need to append it to the path of the setup file.
    resultPath = setup_file_path.substr(0, setup_file_path.find_last_of('/') + 1) + path;
  }
  if (!pathExists(resultPath)) {
    throw std::runtime_error("Path: " + resultPath + " does not exist");
  }
  return resultPath;
}

}  // namespace ecat_manager
