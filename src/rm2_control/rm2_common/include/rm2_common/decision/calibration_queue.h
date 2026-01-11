//
// Created by ch on 2025/9/21.
//

#pragma once

#include "rm2_common/decision/service_caller.h"
#include "rm2_common/decision/controller_manager.h"
#include <string>

namespace rm2_common
{
class CalibrationService
{
public:
  CalibrationService(std::string& param_prefix, rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    if (!node_->get_parameter(param_prefix + "start_controllers", start_controllers))
      RCLCPP_ERROR(node_->get_logger(), "Start_controllers not found");
    if (!node_->get_parameter(param_prefix + "stop_controllers", stop_controllers))
      RCLCPP_ERROR(node_->get_logger(), "Stop_controllers not found");
    if (node_->get_parameter(param_prefix + "services_name", services_name)) {
        for (auto & service_name : services_name) {
            query_services.push_back(new QueryCalibrationServiceCaller(node_, service_name));
      }
    }
    else
      RCLCPP_ERROR(node_->get_logger(), "Services_name not found");

  }
  void setCalibratedFalse()
  {
    for (auto& service : query_services)
			service->getResponse().is_calibrated = false;
		
  }
  bool isCalibrated()
  {
    bool is_calibrated = true;
    for (auto& service : query_services)
      is_calibrated &= service->isCalibrated();
    return is_calibrated;
  }
  void callService()
  {
    for (auto& service : query_services)
      service->callService();
  }
  std::vector<std::string> start_controllers, stop_controllers, services_name;
  std::vector<QueryCalibrationServiceCaller*> query_services;

private:
  rclcpp::Node::SharedPtr node_;
};

class CalibrationQueue
{
public:
  explicit CalibrationQueue(std::string& param_prefix, rclcpp::Node::SharedPtr node, ControllerManager& controller_manager)
    : node_(node), controller_manager_(controller_manager), switched_(false)
  {
    // Don't calibration if using simulation
    bool use_sim_time = false;
    std::vector<std::string> param_members;
		node_->get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time || !node_->get_parameter(param_prefix, param_members))
      return;
    for (auto & param_member : param_members)
      calibration_services_.emplace_back(param_prefix + "." + param_member + ".", node);
    last_query_ = node -> now();
    calibration_itr_ = calibration_services_.end();
    // Start with calibrated, you should use reset() to start calibration.
  }
  void reset()
  {
    if (calibration_services_.empty())
      return;
    calibration_itr_ = calibration_services_.begin();
    switched_ = false;
    for (auto service : calibration_services_)
      service.setCalibratedFalse();
  }
  void update(const rclcpp::Time& time, bool flip_controllers)
  {
    if (calibration_services_.empty())
      return;
    if (isCalibrated())
      return;
    if (switched_)
    {
      if (calibration_itr_->isCalibrated())
      {
        if (flip_controllers)
          controller_manager_.startControllers(calibration_itr_->stop_controllers);
        controller_manager_.stopControllers(calibration_itr_->start_controllers);
        calibration_itr_++;
        switched_ = false;
      }
      else if ((time - last_query_).seconds() > 0.2)
      {
        last_query_ = time;
        calibration_itr_->callService();
      }
    }
    else
    {
      // Switch controllers
      switched_ = true;
      if (calibration_itr_ != calibration_services_.end())
      {
        controller_manager_.startControllers(calibration_itr_->start_controllers);
        controller_manager_.stopControllers(calibration_itr_->stop_controllers);
      }
    }
  }
  void update(const rclcpp::Time& time)
  {
    update(time, true);
  }
  bool isCalibrated()
  {
    return calibration_itr_ == calibration_services_.end();
  }
  void stopController()
  {
    if (calibration_services_.empty())
      return;
    if (calibration_itr_ != calibration_services_.end() && switched_)
      controller_manager_.stopControllers(calibration_itr_->stop_controllers);
  }
  void stop()
  {
    if (switched_)
    {
      calibration_itr_ = calibration_services_.end();
      switched_ = false;
    }
  }

private:
	rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_query_;
  std::vector<CalibrationService> calibration_services_;
  std::vector<CalibrationService>::iterator calibration_itr_;
  ControllerManager& controller_manager_;
  bool switched_;
};
}  // namespace rm2_common