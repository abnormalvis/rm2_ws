//
// Created by qiayuan on 23-4-10.
//

#pragma once

#include <fstream>
#include <soem_interface_rsl/EthercatBusManagerBase.hpp>

namespace ecat_manager {
class EcatBusManager : public soem_interface_rsl::EthercatBusManagerBase {
 public:
  using SharedPtr = std::shared_ptr<EcatBusManager>;

  enum class EthercatSlaveType { ClearFoc, ClearImu, Mit, Rm, NA };

  struct EthercatSlaveEntry {
    EthercatSlaveType type;
    std::string name;
    std::string configFilePath;

    uint32_t ethercatAddress;
    std::string ethercatBus;
  };

  bool fromFile(const std::string& file, bool startup);

  const EthercatSlaveEntry& getInfoForSlave(const std::shared_ptr<soem_interface_rsl::EthercatSlaveBase>& slave);

  std::set<std::string> getAllBusNames();

  template <typename T, typename dummy = std::enable_if_t<std::is_base_of_v<soem_interface_rsl::EthercatSlaveBase, T>>>
  std::vector<std::shared_ptr<T>> getSlavesOfType(EthercatSlaveType ethercatSlaveType) {
    std::vector<std::shared_ptr<T>> slaves;

    for (auto& slave : slaves_) {
      if (getInfoForSlave(slave).type == ethercatSlaveType) {
        auto ptr = std::dynamic_pointer_cast<T>(slave);
        if (ptr) {
          slaves.push_back(ptr);
        } else {
          throw std::runtime_error("getSlavesOfType: ethercatSlaveTyp and provided Type does not match!");
        }
      }
    }
    return slaves;
  }

  bool onActivate(const std::string &ethercatBus) {
      if (buses_.find(ethercatBus) == buses_.end()) {
          throw std::runtime_error("onActivate: can not find the corresponding bus!");
      }
      buses_[ethercatBus]->setState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT);
      if (!buses_[ethercatBus]->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT, 0, 0)) {
          return false;
      }
      buses_[ethercatBus]->setState(soem_interface_rsl::ETHERCAT_SM_STATE::PRE_OP);
      if (!buses_[ethercatBus]->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::PRE_OP, 0, 0)) {
          return false;
      }
      buses_[ethercatBus]->setState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP);
      if (!buses_[ethercatBus]->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP, 0, 0)) {
          return false;
      }
      buses_[ethercatBus]->setState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL);
      if (!buses_[ethercatBus]->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL, 0, 0)) {
          return false;
      }
      return true;
  }

  bool onDeactivate(const std::string &ethercatBus) {
      if (buses_.find(ethercatBus) == buses_.end()) {
          throw std::runtime_error("onDeactivate: can not find the corresponding bus!");
      }
      buses_[ethercatBus]->setState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP);
      return buses_[ethercatBus]->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP, 0, 0);
  }

  bool busMonitoring(const std::string &ethercatBus) {
      if (buses_.find(ethercatBus) == buses_.end()) {
          throw std::runtime_error("busMonitoring: can not find the corresponding bus!");
      }
      return buses_[ethercatBus]->doBusMonitoring(false);
  }

  protected:
  void parseFile(std::string path);

  void setup(bool startup);

  static std::string handleFilePath(const std::string& path, const std::string& setup_file_path);

  std::vector<std::shared_ptr<soem_interface_rsl::EthercatSlaveBase>> slaves_;

  std::vector<EthercatSlaveEntry> slaveEntries_;
  std::map<std::shared_ptr<soem_interface_rsl::EthercatSlaveBase>, EthercatSlaveEntry> slave2Entry_;
  std::set<std::string> busNames_;

  std::string setupFilepath_;
  double timeStep_ = 0.0;
};

}  // namespace ecat_manager
