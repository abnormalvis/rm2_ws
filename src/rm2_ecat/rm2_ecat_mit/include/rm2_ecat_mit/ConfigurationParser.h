//
// Created by kook on 12/29/23.
//

#pragma once

#include <yaml-cpp/yaml.h>
#include <message_logger/message_logger.hpp>

#include "rm2_ecat_mit/Configuration.h"

namespace rm2_ecat {
namespace mit {
enum class MotorType { DM4310, DM4340, HT8115, DM8009, DM6006 };

class ConfigurationParser {
 public:
  ConfigurationParser() = delete;

  explicit ConfigurationParser(const std::string& filename);

  explicit ConfigurationParser(const YAML::Node& configNode);

  Configuration getConfiguration() const { return configuration_; }

 private:
  void parseConfiguration(YAML::Node configNode);

  static uint16_t getMaxOut(MotorType motorType);
  static double getTorqueOffset(MotorType motorType);
  static double getPositionOffset(MotorType motorType);
  static double getVelocityOffset(MotorType motorType);
  static double getKpFactorToInteger(MotorType motorType);
  static double getKdFactorToInteger(MotorType motorType);
  static double getTorqueFactorIntegerToNm(MotorType motorType);
  static double getPositionFactorIntegerToRad(MotorType motorType);
  static double getVelocityFactorIntegerPerMinusToRadPerSec(MotorType motorType);
  static double getTorqueFactorNmToInteger(MotorType motorType);
  static double getPositionFactorRadToInteger(MotorType motorType);
  static double getVelocityFactorRadPerSecToInteger(MotorType motorType);
  static MotorType getMotoTypeFromString(const std::string& motorTypeString);

  Configuration configuration_{};
};
}  // namespace mit
}  // namespace rm2_ecat
