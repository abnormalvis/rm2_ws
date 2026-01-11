//
// Created by ch on 2025/10/4.
//

#pragma once

#include "rm2_common/filters/imu_filter_base.h"
#include <imu_complementary_filter/complementary_filter.h>

namespace rm2_common
{
class ImuComplementaryFilter : public ImuFilterBase
{
public:
  explicit ImuComplementaryFilter(rclcpp::Node::SharedPtr node) : ImuFilterBase(node)
  {
  }
  void getOrientation(double& q0, double& q1, double& q2, double& q3) override;

private:
  void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) override;
  bool initFilter(std::string& imu_data_prefix) override;
  void resetFilter() override;
  // Parameters:
  double gain_acc_;
  double gain_mag_;
  bool do_bias_estimation_;
  double bias_alpha_;
  bool do_adaptive_gain_;
  bool use_mag_;
  std::shared_ptr<imu_tools::ComplementaryFilter> filter_;
};
}  // namespace rm2_common
