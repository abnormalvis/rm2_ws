//
// Created by ch on 2026/1/2.
//

#pragma once

#include <array>
#include <hardware_interface/hardware_info.hpp>
#include <joint_limits/data_structures.hpp>
#include <joint_limits/joint_limiter_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_saturation_limiter.hpp>
#include <joint_limits/joint_soft_limiter.hpp>
#include <memory>
#include <message_logger/log/log_messages.hpp>

#include <rclcpp/time.hpp>
#include <string>
#include <unordered_map>
#include <utility>

namespace rm2_ecat {
struct ActData {
  double pos, vel, effort;
  double commandUnlimited, command;
  double offset;
  bool halted, needCalibration, calibrated, calibrationReading;
};

struct ImuData {
  double stamp;
  std::array<double, 3> angularVel, linearAccel, orientationCovariance, angularVelocityCovariance, linearAccelerationCovariance;
  std::array<double, 4> orientation;
};

enum GpioType
{
  INPUT,
  OUTPUT
};
struct GpioData
{
  
  GpioType type;
  double pin;
  bool value;
};

struct TransmissionData
{
  std::array<double, 3> state{ 0, 0, 0 };
  std::array<double, 3> command{ 0, 0, 0 };
  std::array<double, 3> transmissionPassthrough{ 0, 0, 0 };
};

class ActuatorStateMap {
private:
  struct ActuatorStateData {
    double* pos;
    double* vel;
    double* eff;
    double* commandUnlimited;
    double* command;
  };
  std::unordered_map<std::string, ActuatorStateData> map_;

public:
  ActuatorStateMap() = default;
  
  void addMap(std::string& name, double* pos, double* vel, double* eff, double* commandUnlimited, double* command) {
    map_.emplace(name, ActuatorStateData{std::move(pos), std::move(vel), std::move(eff), std::move(commandUnlimited), std::move(command)});
  }
  ActuatorStateData* find(const std::string& name) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        return &(it->second);
    }
    return nullptr;
  }
  bool setPos(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->pos = value;
      return true;
    }
    return false;
  }
  bool setVel(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->vel = value;
      return true;
    }
    return false;
  }
  bool setEff(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->eff = value;
      return true;
    }
    return false;
  }
  bool setCommandUnlimited(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->commandUnlimited = value;
      return true;
    }
    return false;
  }
  bool setCommand(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->command = value;
      return true;
    }
    return false;
  }
  double getPos(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->pos;
    }
    return 0.;
  }
  double getVel(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->vel;
    }
    return 0.;
  }
  double getEff(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->eff;
    }
    return 0.;
  }
  double getCommandUnlimited(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->commandUnlimited;
    }
    return 0.;
  }
  double getCommand(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->command;
    }
    return 0.;
  }
};

class ActuatorExtraMap {
private:
  struct ActuatorExtraData {
    bool* halted;
    bool* needCalibration;
    bool* calibrated;
    bool* calibrationReading;
    double* offset;
  };
  std::unordered_map<std::string, ActuatorExtraData> map_;

public:
  ActuatorExtraMap() = default;
  
  void addMap(std::string& name, bool* halted,
              bool* need_calibration, bool* calibrated,
              bool* calibration_reading, double* offset) {
    map_.emplace(name, ActuatorExtraData{std::move(halted), 
                               std::move(need_calibration), 
                                     std::move(calibrated), 
                            std::move(calibration_reading), 
                                         std::move(offset)});
  }
  ActuatorExtraData* find(const std::string& name) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        return &(it->second);
    }
    return nullptr;
  }
  bool setHalted(const std::string& name, bool value) {
    if (find(name) != nullptr)
    {
      *find(name)->halted = value;
      return true;
    }
    return false;
  }
  bool setNeedCalibration(const std::string& name, bool value) {
    if (find(name) != nullptr)
    {
      *find(name)->needCalibration = value;
      return true;
    }
    return false;
  }
  bool setCalibrated(const std::string& name, bool value) {
    if (find(name) != nullptr)
    {
      *find(name)->calibrated = value;
      return true;
    }
    return false;
  }
  bool setCalibrationReading(const std::string& name, bool value) {
    if (find(name) != nullptr)
    {
      *find(name)->calibrationReading = value;
      return true;
    }
    return false;
  }
  bool setOffset(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->offset = value;
      return true;
    }
    return false;
  }
  bool getHalted(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->halted;
    }
    return true;
  }
  bool getNeedCalibration(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->needCalibration;
    }
    return false;
  }
  bool getCalibrated(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->calibrated;
    }
    return false;
  }
  bool getCalibrationReading(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->calibrationReading;
    }
    return false;
  }
  double getOffset(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->offset;
    }
    return 0.;
  }
};

class ImuSensorMap {
private:
  struct ImuSensorData {
    std::array<double, 4>* orientation;
    std::array<double, 3>* angularVel;
    std::array<double, 3>* linearAccel;
    std::array<double, 3>* orientationCovariance;
    std::array<double, 3>* angularVelocityCovariance;
    std::array<double, 3>* linearAccelerationCovariance;
    double* stamp;
  };
  std::unordered_map<std::string, ImuSensorData> map_;

public:
  ImuSensorMap() = default;
  
  void addMap(std::string& name, std::array<double, 4>* orientation,
                                 std::array<double, 3>* angularVel,
                                 std::array<double, 3>* linearAccel,
                                 std::array<double, 3>* orientationCovariance,
                                 std::array<double, 3>* angularVelocityCovariance,
                                 std::array<double, 3>* linearAccelerationCovariance,
                                 double* stamp) {
    map_.emplace(name, ImuSensorData{std::move(orientation), 
                                      std::move(angularVel), 
                                     std::move(linearAccel), 
                           std::move(orientationCovariance), 
                       std::move(angularVelocityCovariance), 
                    std::move(linearAccelerationCovariance),
                                           std::move(stamp)});
  }
  ImuSensorData* find(const std::string& name) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        return &(it->second);
    }
    return nullptr;
  }
  bool setOrientation(const std::string& name, std::array<double, 4> value) {
    if (find(name) != nullptr)
    {
      *find(name)->orientation = value;
      return true;
    }
    return false;
  }
  bool setAngularVel(const std::string& name, std::array<double, 3> value) {
    if (find(name) != nullptr)
    {
      *find(name)->angularVel = value;
      return true;
    }
    return false;
  }
  bool setLinearAccel(const std::string& name, std::array<double, 3> value) {
    if (find(name) != nullptr)
    {
      *find(name)->linearAccel = value;
      return true;
    }
    return false;
  }
  bool setOrientationCovariance(const std::string& name, std::array<double, 3> value) {
    if (find(name) != nullptr)
    {
      *find(name)->orientationCovariance = value;
      return true;
    }
    return false;
  }
  bool setAngularVelocityCovariance(const std::string& name, std::array<double, 3> value) {
    if (find(name) != nullptr)
    {
      *find(name)->angularVelocityCovariance = value;
      return true;
    }
    return false;
  }
  bool setLinearAccelerationCovariance(const std::string& name, std::array<double, 3> value) {
    if (find(name) != nullptr)
    {
      *find(name)->linearAccelerationCovariance = value;
      return true;
    }
    return false;
  }
  bool setStamp(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      *find(name)->stamp = value;
      return true;
    }
    return false;
  }
  std::array<double, 4> getOrientation(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->orientation;
    }
    return {};
  }
  std::array<double, 3> getAngularVel(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->angularVel;
    }
    return {};
  }
  std::array<double, 3> getLinearAccel(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->linearAccel;
    }
    return {};
  }
  std::array<double, 3> getOrientationCovariance(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->orientationCovariance;
    }
    return {};
  }
  std::array<double, 3> getAngularVelocityCovariance(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->angularVelocityCovariance;
    }
    return {};
  }
  std::array<double, 3> getLinearAccelerationCovariance(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->linearAccelerationCovariance;
    }
    return {};
  }
  double getStamp(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->stamp;
    }
    return 0.;
  }
};

class GpioStateMap {
private:
  struct GpioStateData {
    double type;
    bool* value;
  };
  std::unordered_map<std::string, GpioStateData> map_;

public:
  GpioStateMap() = default;
  
  void addMap(std::string& name, double type, bool* value) {
    map_.emplace(name, GpioStateData{std::move(type), std::move(value)});
  }
  GpioStateData* find(const std::string& name) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        return &(it->second);
    }
    return nullptr;
  }
  bool setType(const std::string& name, double value) {
    if (find(name) != nullptr)
    {
      find(name)->type = value;
      return true;
    }
    return false;
  }
  bool setValue(const std::string& name, bool value) {
    if (find(name) != nullptr)
    {
      *find(name)->value = value;
      return true;
    }
    return false;
  }
  double getType(const std::string& name) {
    if (find(name) != nullptr) {
      return find(name)->type;
    }
    return 0.;
  }
  bool getValue(const std::string& name) {
    if (find(name) != nullptr) {
      return *find(name)->value;
    }
    return false;
  }
};

class JointLimiters {
private:
  struct LimiterData {
    bool hasJointLimits{}, hasSoftLimits{};
    joint_limits::JointSaturationLimiter<joint_limits::JointControlInterfacesData> saturationLimiter;
    joint_limits::JointSoftLimiter softLimiter;
  };

  std::unordered_map<std::string, std::shared_ptr<LimiterData>> limiters_;
  hardware_interface::HardwareInfo info_;
public:
  JointLimiters() = default;

  bool init(const hardware_interface::HardwareInfo& info, 
          const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf,
          const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf) {
    info_ = info;

    for (const auto& joint : info_.joints) {
      std::string joint_name = joint.name;
      auto joint_limiter_data = std::make_shared<LimiterData>();

      auto limits_it = info_.limits.find(joint_name);
      if (limits_it != info_.limits.end()) {
        joint_limiter_data->hasJointLimits = true;

        if (limits_it->second.has_position_limits) {
          limits_it->second.min_position += std::numeric_limits<double>::epsilon();
          limits_it->second.max_position -= std::numeric_limits<double>::epsilon();
        }
        
        if (!joint_limiter_data->saturationLimiter.init({joint_name}, 
            {limits_it->second}, {},
            param_itf, logging_itf)) {
          MELO_ERROR_STREAM("Failed to initialize saturation limiter for joint: " << joint_name);
          return false;
        }
            
        MELO_DEBUG_STREAM("Joint " << joint_name << " has URDF position limits.");

        auto soft_limits_it = info_.soft_limits.find(joint_name);
        if (soft_limits_it != info_.soft_limits.end()) {
          joint_limiter_data->hasSoftLimits = true;

          if (!joint_limiter_data->softLimiter.init({joint_name}, 
              {limits_it->second}, {soft_limits_it->second}, 
              param_itf, logging_itf)) {
            MELO_ERROR_STREAM("Failed to initialize soft limiter for joint: " << joint_name);
            return false;
          }

          MELO_DEBUG_STREAM("Joint " << joint_name << " has soft joint limits from URDF.");
        }

        limiters_.emplace(std::move(joint_name),std::move(joint_limiter_data));
      }
    }
    return true;
  }

  bool enforceJoint(const std::string & joint_name, const joint_limits::JointControlInterfacesData & current_joint_states,
      joint_limits::JointControlInterfacesData & desired_joint_states, const rclcpp::Duration & dt) {
    auto limiter_it = limiters_.find(joint_name);
    if (limiter_it != limiters_.end()) {
      if (limiter_it->second->hasSoftLimits) {
        if (!limiter_it->second->softLimiter.configure(current_joint_states)) {
          MELO_ERROR_STREAM("Configure failed for " << joint_name << "soft limiter");
          return false;
        }
        if (limiter_it->second->softLimiter.enforce(current_joint_states, desired_joint_states, dt)) {
          MELO_DEBUG_STREAM("Command enforced for " << joint_name << "soft limiter");
        }
      } else if (limiter_it->second->hasJointLimits) {
        if (!limiter_it->second->saturationLimiter.configure(current_joint_states)) {
          MELO_ERROR_STREAM("Configure failed for " << joint_name << "saturation limiter");
          return false;
        }
        if (limiter_it->second->saturationLimiter.enforce(current_joint_states, desired_joint_states, dt)) {
          MELO_DEBUG_STREAM("Command enforced for " << joint_name << "saturation limiter");
        }
      }
    } else {
      MELO_DEBUG_STREAM("Joint " << joint_name << "has no joint limits");
    }
    return true;
  }
};
}  // namespace rm2_ecat
