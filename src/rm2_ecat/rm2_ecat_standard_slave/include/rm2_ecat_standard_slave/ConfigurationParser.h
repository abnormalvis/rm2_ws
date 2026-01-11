//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <yaml-cpp/yaml.h>
#include <message_logger/message_logger.hpp>

#include "rm2_ecat_standard_slave/Configuration.h"

namespace rm2_ecat {
namespace standard {
enum class MotorType { M3508, M2006, GM6020 };

class ConfigurationParser {
 public:
  ConfigurationParser() = delete;

  explicit ConfigurationParser(const std::string& filename);

  explicit ConfigurationParser(const YAML::Node& configNode);

  Configuration getConfiguration() const { return configuration_; }

 private:
  void parseConfiguration(YAML::Node configNode);

  static uint16_t getMaxOut(MotorType motorType);
  static double getTorqueFactorIntegerToNm(MotorType motorType);
  static double getTorqueFactorNmToInteger(MotorType motorType);
  static MotorType getMotoTypeFromString(const std::string& motorTypeString);

  Configuration configuration_{};
};
}  // namespace standard
}  // namespace rm2_ecat
