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

enum GpioType
{
  INPUT,
  OUTPUT
};

struct TransmissionData
{
  std::array<double, 3> state{ 0, 0, 0 };
  std::array<double, 3> command{ 0, 0, 0 };
  std::array<double, 3> transmissionPassthrough{ 0, 0, 0 };
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
