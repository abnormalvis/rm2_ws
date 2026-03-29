#pragma once
#include "rm2_gimbal_controllers/gimbal_base.h"
#include "rm2_gimbal_controllers/gimbal_controller_parameters.hpp"
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace rm2_gimbal_controllers {
class GimbalController : public GimbalControllerBase {
public:
  GimbalController() = default;
  ~GimbalController() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

private:
  std::shared_ptr<gimbal_controller::ParamListener> param_listener_;
  gimbal_controller::Params params_;
};
} // namespace rm2_gimbal_controllers
