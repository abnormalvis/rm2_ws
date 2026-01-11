//
// Created by kook on 12/29/23.
//

#include "rm2_ecat_mit/ConfigurationParser.h"

namespace rm2_ecat {
namespace mit {
template <typename T>
bool getValueFromFile(YAML::Node& yamlNode, const std::string& varName, T& var) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM("[rm2_ecat::mit::ConfigurationParser::getValueFromFile]: field '" << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getValueFromFile] Error while parsing value \"" << varName
                                                                                                      << "\", default values will be used");
    return false;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(filename);
  } catch (...) {
    MELO_FATAL_STREAM("[rm2_ecat::mit::ConfigurationParser::ConfigurationParser] Loading YAML configuration file '" << filename << "' failed.");
  }
  parseConfiguration(configNode);
}

ConfigurationParser::ConfigurationParser(const YAML::Node& configNode) {
  parseConfiguration(configNode);
}

void ConfigurationParser::parseConfiguration(YAML::Node configNode) {
  if (configNode["can_motors"].IsDefined()) {
    YAML::Node motors = configNode["can_motors"];
    if (!motors.IsSequence()) {
      return;
    }
    for (YAML::const_iterator it = motors.begin(); it != motors.end(); ++it) {
      YAML::Node child = *it;
      uint16_t id = 0, bus = 0;
      getValueFromFile(child, "can_id", id);
      getValueFromFile(child, "can_bus", bus);
      MotorType type = getMotoTypeFromString(child["type"].as<std::string>());
      auto motorConfiguration = std::make_pair(
          id, MotorConfiguration{child["name"].as<std::string>(), getMaxOut(type), getTorqueOffset(type), getPositionOffset(type),
                                 getVelocityOffset(type), getKpFactorToInteger(type), getKdFactorToInteger(type),
                                 getTorqueFactorIntegerToNm(type), getTorqueFactorNmToInteger(type), getPositionFactorIntegerToRad(type),
                                 getPositionFactorRadToInteger(type), getVelocityFactorIntegerPerMinusToRadPerSec(type),
                                 getVelocityFactorRadPerSecToInteger(type)});
      getValueFromFile(child, "need_calibration", motorConfiguration.second.needCalibration_);

      if (bus == 0) {
        configuration_.can0MotorConfigurations_.insert(motorConfiguration);
      } else if (bus == 1) {
        configuration_.can1MotorConfigurations_.insert(motorConfiguration);
      } else {
        MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::parseConfiguration] Unknown can bus");
      }
    }
  }
  if (configNode["gpios"].IsDefined()) {
    YAML::Node gpios = configNode["gpios"];
    if (!gpios.IsSequence()) {
      return;
    }
    for (YAML::const_iterator it = gpios.begin(); it != gpios.end(); ++it) {
      YAML::Node child = *it;
      GpioConfiguration gpioConfiguration;
      getValueFromFile(child, "name", gpioConfiguration.name_);
      getValueFromFile(child, "mode", gpioConfiguration.mode_);
      configuration_.gpioConfigurations_.insert(std::make_pair(child["pin"].as<uint16_t>(), gpioConfiguration));
    }
  }
}

uint16_t ConfigurationParser::getMaxOut(MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM6006:
    case MotorType::DM8009:
      return 2048;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getMaxOut] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getTorqueOffset(rm2_ecat::mit::MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::HT8115:
      return -18.0;
    case MotorType::DM4340:
      return -27.0;
    case MotorType::DM8009:
      return -54.0;
    case MotorType::DM6006:
      return -10.0;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getTorqueOffset] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getPositionOffset(rm2_ecat::mit::MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
      return -12.56;
    case MotorType::DM4340:
    case MotorType::DM6006:
    case MotorType::DM8009:
      return -12.5;
    case MotorType::HT8115:
      return -95.5;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getPositionOffset] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getVelocityOffset(rm2_ecat::mit::MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM8009:
    case MotorType::HT8115:
      return -45.;
    case MotorType::DM4340:
      return -5.44;
    case MotorType::DM6006:
      return -30.;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getVelocityOffset] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getKpFactorToInteger(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 8.19;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getKpFactorToInteger] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getKdFactorToInteger(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 819.;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getKdFactorToInteger] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getTorqueFactorIntegerToNm(MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
      return 18. * 2. / 4095;
    case MotorType::DM4340:
      return 27. * 2. / 4096;
    case MotorType::HT8115:
      return 18. * 2. / 4095;
    case MotorType::DM8009:
      return 54. * 2. / 4095;
    case MotorType::DM6006:
      return 10. * 2. / 4095;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getTorqueFactorIntegerToNm] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getPositionFactorIntegerToRad(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
      return 12.56 * 2. / 65536;
    case MotorType::DM4340:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 12.5 * 2. / 65536;
    case MotorType::HT8115:
      return 95.5 * 2. / 65536;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getPositionFactorIntegerToRad] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getVelocityFactorIntegerPerMinusToRadPerSec(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::HT8115:
    case MotorType::DM8009:
      return 45. * 2. / 4095;
    case MotorType::DM4340:
      return 5.44 * 2. / 4095;
    case MotorType::DM6006:
      return 30. * 2. / 4095;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getVelocityFactorIntegerPerMinusToRadPerSec] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getTorqueFactorNmToInteger(MotorType motorType) {
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 1. / getTorqueFactorIntegerToNm(motorType);
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getTorqueFactorNmToInteger] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getPositionFactorRadToInteger(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 1. / getPositionFactorIntegerToRad(motorType);
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getPositionFactorRadToInteger] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getVelocityFactorRadPerSecToInteger(rm2_ecat::mit::MotorType motorType){
  switch (motorType) {
    case MotorType::DM4310:
    case MotorType::DM4340:
    case MotorType::HT8115:
    case MotorType::DM8009:
    case MotorType::DM6006:
      return 1. / getVelocityFactorIntegerPerMinusToRadPerSec(motorType);
    default:
      MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getVelocityFactorRadPerSecToInteger] Unknown motor type");
      return 0;
  }
}

MotorType ConfigurationParser::getMotoTypeFromString(const std::string& motorTypeString) {
  if (motorTypeString == "DM4310") {
    return MotorType::DM4310;
  } else if (motorTypeString == "DM4340") {
    return MotorType::DM4340;
  } else if (motorTypeString == "HT8115") {
    return MotorType::HT8115;
  } else if (motorTypeString == "DM8009") {
    return MotorType::DM8009;
  } else if (motorTypeString == "DM6006") {
    return MotorType::DM6006;
  } else {
    MELO_ERROR_STREAM("[rm2_ecat::mit::ConfigurationParser::getMotoTypeFromString] Unknown motor type");
    return MotorType::DM4310;
  }
}
}  // namespace mit
}  // namespace rm2_ecat
