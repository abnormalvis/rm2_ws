#pragma once
#include "rm2_gimbal_controllers/gimbal_base.h"
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace rm2_gimbal_controllers {
class GimbalController : public GimbalControllerBase {
public:
  GimbalController() = default;
  ~GimbalController() override = default;
};
} // namespace rm2_gimbal_controllers
