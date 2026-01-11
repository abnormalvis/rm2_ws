//
// Created by qiayuan on 23-5-14.
//

#include "rm2_ecat_standard_slave/ConfigurationParser.h"

namespace rm2_ecat {
namespace standard {
template <typename T>
bool getValueFromFile(YAML::Node& yamlNode, const std::string& varName, T& var) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM("[rm2_ecat::ConfigurationParser::getValueFromFile]: field '" << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::getValueFromFile] Error while parsing value \"" << varName
                                                                                                      << "\", default values will be used");
    return false;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(filename);
  } catch (...) {
    MELO_FATAL_STREAM("[rm2_ecat::ConfigurationParser::ConfigurationParser] Loading YAML configuration file '" << filename << "' failed.");
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
      auto motorConfiguration = std::make_pair(id, MotorConfiguration{child["name"].as<std::string>(), getMaxOut(type),
                                                                      getTorqueFactorIntegerToNm(type), getTorqueFactorNmToInteger(type)});
      getValueFromFile(child, "need_calibration", motorConfiguration.second.needCalibration_);

      if (bus == 0) {
        configuration_.can0MotorConfigurations_.insert(motorConfiguration);
      } else if (bus == 1) {
        configuration_.can1MotorConfigurations_.insert(motorConfiguration);
      } else {
        MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::parseConfiguration] Unknown can bus");
      }
    }
  }
  if (configNode["can_imus"].IsDefined()) {
    YAML::Node imus = configNode["can_imus"];
    if (!imus.IsSequence()) {
      return;
    }
    for (YAML::const_iterator it = imus.begin(); it != imus.end(); ++it) {
      YAML::Node child = *it;
      uint16_t bus = 0;
      getValueFromFile(child, "can_bus", bus);
      ImuConfiguration imuConfiguration;
      getValueFromFile(child, "name", imuConfiguration.name_);
      getValueFromFile(child, "gain_accel", imuConfiguration.gainAccel_);
      getValueFromFile(child, "bias_alpha_", imuConfiguration.biasAlpha_);
      getValueFromFile(child, "do_bias_estimation_", imuConfiguration.doBiasEstimation_);
      getValueFromFile(child, "do_adaptive_gain", imuConfiguration.doAdaptiveGain_);
      if (child["bias"].IsDefined()) {
        YAML::Node bias = child["bias"];
        if (bias.IsSequence() && bias.size() == 3) {
          for (size_t i = 0; i < 3; ++i) {
            imuConfiguration.angularVelBias_[i] = bias[i].as<double>();
          }
        } else {
          MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::parseConfiguration] Bias must be a sequence of 3 elements");
        }
      }
      if (bus == 0) {
        configuration_.can0ImuConfiguration_ = std::make_shared<ImuConfiguration>(imuConfiguration);
      } else if (bus == 1) {
        configuration_.can1ImuConfiguration_ = std::make_shared<ImuConfiguration>(imuConfiguration);
      } else {
        MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::parseConfiguration] Unknown can bus");
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
  if (configNode["dbus"].IsDefined()) {
    YAML::Node dBuss = configNode["dbus"];
    if (!dBuss.IsSequence()) {
      return;
    }
    for (YAML::const_iterator it = dBuss.begin(); it != dBuss.end(); ++it) {
      YAML::Node child = *it;
      DbusConfiguration dbusConfiguration;
      getValueFromFile(child, "name", dbusConfiguration.name_);
      configuration_.dbusConfiguration = std::make_shared<DbusConfiguration>(dbusConfiguration);
    }
  }
}

uint16_t ConfigurationParser::getMaxOut(MotorType motorType) {
  switch (motorType) {
    case MotorType::M3508:
      return 16384;
    case MotorType::M2006:
      return 10000;
    case MotorType::GM6020:
      return 30000;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::getMaxOut] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getTorqueFactorIntegerToNm(MotorType motorType) {
  switch (motorType) {
    case MotorType::M3508:
      return 20. / 16384. * 0.0156223893;
    case MotorType::M2006:
      return 10. / 10000. * 0.18 / 36.;
    case MotorType::GM6020:
      return 5.880969e-5;  // special coefficient;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::getTorqueFactorIntegerToNm] Unknown motor type");
      return 0;
  }
}

double ConfigurationParser::getTorqueFactorNmToInteger(MotorType motorType) {
  switch (motorType) {
    case MotorType::M3508:
    case MotorType::M2006:
      return 1. / getTorqueFactorIntegerToNm(motorType);
    case MotorType::GM6020:
      return 25000;
    default:
      MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::getTorqueFactorIntegerToNm] Unknown motor type");
      return 0;
  }
}

MotorType ConfigurationParser::getMotoTypeFromString(const std::string& motorTypeString) {
  if (motorTypeString == "M3508") {
    return MotorType::M3508;
  } else if (motorTypeString == "M2006") {
    return MotorType::M2006;
  } else if (motorTypeString == "GM6020") {
    return MotorType::GM6020;
  } else {
    MELO_ERROR_STREAM("[rm2_ecat::ConfigurationParser::getMotoTypeFromString] Unknown motor type");
    return MotorType::M3508;
  }
}
}  // namespace standard
}  // namespace rm2_ecat
