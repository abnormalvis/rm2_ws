#pragma once

#include "rm2_calibration_controllers/calibration_base.h"

namespace rm2_calibration_controllers
{
class MechanicalCalibrationController : public CalibrationBase
{
public:
  MechanicalCalibrationController() = default;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum State
  {
    MOVING_POSITIVE = 3,
    MOVING_NEGATIVE,
  };
  int countdown_{};
  double velocity_threshold_{}, position_threshold_{};
  double positive_position_{}, negative_position_{}, target_position_{};
  bool is_return_{}, is_center_{};
};

} // namespace rm2_calibration_controllers