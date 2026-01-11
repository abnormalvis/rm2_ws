//
// Created by ch on 2025/10/4.
//

#include "rm2_common/filters/imu_complementary_filter.h"

namespace rm2_common
{
void ImuComplementaryFilter::filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt)
{
  filter_->update(ax, ay, az, wx, wy, wz, dt);
}
void ImuComplementaryFilter::getOrientation(double& q0, double& q1, double& q2, double& q3)
{
  filter_->getOrientation(q0, q1, q2, q3);
}
bool ImuComplementaryFilter::initFilter(std::string& imu_data_prefix)
{
  use_mag_ = node_->declare_parameter(imu_data_prefix + "use_mag",false);
  gain_acc_ = node_->declare_parameter(imu_data_prefix + "gain_acc",0.01);
  gain_mag_ = node_->declare_parameter(imu_data_prefix + "gain_mag",0.01);
  do_bias_estimation_ = node_->declare_parameter(imu_data_prefix + "do_bias_estimation",true);
  bias_alpha_ = node_->declare_parameter(imu_data_prefix + "bias_alpha",0.01);
  do_adaptive_gain_ = node_->declare_parameter(imu_data_prefix + "do_adaptive_gain",true);
  resetFilter();
  return true;
}
void ImuComplementaryFilter::resetFilter()
{
  filter_ = std::make_shared<imu_tools::ComplementaryFilter>();
  filter_->setDoBiasEstimation(do_bias_estimation_);
  filter_->setDoAdaptiveGain(do_adaptive_gain_);
  if (!filter_->setGainAcc(gain_acc_))
    RCLCPP_WARN(node_->get_logger(),"Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if (!filter_->setGainMag(gain_mag_))
      RCLCPP_WARN(node_->get_logger(),"Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation_)
  {
    if (!filter_->setBiasAlpha(bias_alpha_))
      RCLCPP_WARN(node_->get_logger(),"Invalid bias_alpha passed to ComplementaryFilter.");
  }
  return;
}
}  // namespace rm2_common
