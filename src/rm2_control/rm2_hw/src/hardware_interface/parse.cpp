//
// Created by ch on 2025/10/25.
//

#include "rm2_hw/hardware_interface/hardware_system.h"

#include <hardware_interface/handle.hpp>
#include <joint_limits/joint_limiter_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_soft_limiter.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rm2_common/filters/imu_complementary_filter.h>
#include <joint_limits/joint_limits_urdf.hpp>
#include <joint_limits/joint_limits_rosparam.hpp>
#include <utility>
#include <vector>

namespace rm2_hw
{

bool RmHardwareSystem::parseActCoeffs(const std::vector<std::string>& act_coeffs)
{
  try
  {
    for(auto& act_coeff_name : act_coeffs)
    {
      ActCoeff act_coeff{};

      // All motor
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2pos",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2vel",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2effort",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"pos2act",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"vel2act",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"effort2act",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"max_out",0.0);

      act_coeff.act2pos = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2pos", 0.0);
      act_coeff.act2vel = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2vel", 0.0);
      act_coeff.act2effort = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2effort", 0.0);
      act_coeff.pos2act = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"pos2act", 0.0);
      act_coeff.vel2act = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"vel2act", 0.0);
      act_coeff.effort2act = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"effort2act", 0.0);
      act_coeff.max_out = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"max_out", 0.0);


      // MIT Cheetah Motor
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2pos_offset",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2vel_offset",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"act2effort_offset",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"kp2act",0.0);
      node_->declare_parameter("actuator_coefficient." + act_coeff_name + "." +"kd2act",0.0);

      act_coeff.act2pos_offset = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2pos_offset", 0.0);
      act_coeff.act2vel_offset = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2vel_offset", 0.0);
      act_coeff.act2effort_offset = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"act2effort_offset", 0.0);
      act_coeff.kp2act = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"kp2act", 0.0);
      act_coeff.kd2act = node_->get_parameter_or("actuator_coefficient." + act_coeff_name + "." +"kd2act", 0.0);

      if(act_coeff.act2pos_offset == 0.0)
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"Actuator type " << act_coeff_name << " has no associated act2pos_offset.");
      if(act_coeff.act2vel_offset == 0.0)
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"Actuator type " << act_coeff_name << " has no associated act2vel_offset.");
      if(act_coeff.act2effort_offset == 0.0)
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"Actuator type " << act_coeff_name << " has no associated act2effort_offset.");
      if(act_coeff.kp2act == 0.0)
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"Actuator type " << act_coeff_name << " has no associated kp2act.");
      if(act_coeff.kd2act == 0.0)
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"Actuator type " << act_coeff_name << " has no associated kd2act.");

      if (type2act_coeffs_.find(act_coeff_name) == type2act_coeffs_.end())
        type2act_coeffs_.insert(std::make_pair(act_coeff_name, act_coeff));
      else
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Repeat actuator coefficient of type: " << act_coeff_name);
    }
  }
  catch (rclcpp::ParameterTypeException e)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(),"Parameter type error: " << e.what() << "\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  catch (rclcpp::exceptions::InvalidParametersException e)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(),"Invalid parameter name: " << e.what() << "\n"
                     << "Please check the configuration.");
    return false;
  }
  return true;
}

bool RmHardwareSystem::parseActData(const std::vector<std::string>& act_datas)
{
  try
  {
    for (auto& act_data_name : act_datas)
    {
      node_->declare_parameter("actuators." + act_data_name + "." + "bus","");
      node_->declare_parameter("actuators." + act_data_name + "." + "type","");
      node_->declare_parameter("actuators." + act_data_name + "." + "id",0);
      node_->declare_parameter("actuators." + act_data_name + "." + "lp_cutoff_frequency",0.0);
      node_->declare_parameter("actuators." + act_data_name + "." + "need_calibration",false);

      std::string bus = node_->get_parameter("actuators." + act_data_name + "." + "bus").as_string();
      std::string type = node_->get_parameter("actuators." + act_data_name + "." + "type").as_string();
      int id = node_->get_parameter("actuators." + act_data_name + "." + "id").as_int();
      double lp_cutoff_frequency = node_->get_parameter("actuators." + act_data_name + "." + "lp_cutoff_frequency").as_double();
      bool need_calibration = node_->get_parameter_or("actuators." + act_data_name + "." + "need_calibration",false);

      if (bus == "")
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Actuator " << act_data_name << " has no associated bus.");
        node_->undeclare_parameter("actuators." + act_data_name + "." + "bus");
        continue;
      }
      else if (type == "") 
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Actuator " << act_data_name << " has no associated type.");
        node_->undeclare_parameter("actuators." + act_data_name + "." + "type");
        continue;
      }
      else if (id == 0) 
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Actuator " << act_data_name << " has no associated ID.");
        node_->undeclare_parameter("actuators." + act_data_name + "." + "id");
        continue;
      }
      else if (lp_cutoff_frequency == 0.0) 
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Actuator " << act_data_name << " has no associated lp_cutoff_frequency.");
        node_->undeclare_parameter("actuators." + act_data_name + "." + "lp_cutoff_frequency");
        continue;
      }

      // check define of act_coeffs
      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Type " << type << " has no associated coefficient.");
        return false;
      }
      // for bus interface
      if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
        bus_id2act_data_.insert(std::make_pair(bus, std::unordered_map<int, ActData>()));

      if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end()))
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Repeat actuator on bus " << bus << " and ID " << id);
        return false;
      }
      else
      {
        // ros::NodeHandle nh = ros::NodeHandle(robot_hw_nh, "actuators/" + it->first);
        bus_id2act_data_[bus].insert(std::make_pair(id, ActData{ .name = act_data_name,
                                                                 .type = type,
                                                                 .stamp = node_->now(),
                                                                 .seq = 0,
                                                                 .halted = false,
                                                                 .need_calibration = need_calibration,
                                                                 .calibrated = false,
                                                                 .calibration_reading = false,
                                                                 .q_raw = 0,
                                                                 .qd_raw = 0,
                                                                 .temp = 0,
                                                                 .q_circle = 0,
                                                                 .q_last = 0,
                                                                 .frequency = 0,
                                                                 .pos = 0,
                                                                 .vel = 0,
                                                                 .effort = 0,
                                                                 .cmd_pos = 0,
                                                                 .cmd_vel = 0,
                                                                 .cmd_effort = 0,
                                                                 .exe_effort = 0,
                                                                 .offset = 0,
                                                                 .lp_filter = new LowPassFilter(lp_cutoff_frequency) }));
      }

      // for ros_control interface TODO: delete this all
      // hardware_interface::ActuatorStateHandle act_state(bus_id2act_data_[bus][id].name, &bus_id2act_data_[bus][id].pos,
      //                                                   &bus_id2act_data_[bus][id].vel,
      //                                                   &bus_id2act_data_[bus][id].effort);
      // rm_control::ActuatorExtraHandle act_extra(bus_id2act_data_[bus][id].name, &bus_id2act_data_[bus][id].halted,
      //                                           &bus_id2act_data_[bus][id].need_calibration,
      //                                           &bus_id2act_data_[bus][id].calibrated,
      //                                           &bus_id2act_data_[bus][id].calibration_reading,
      //                                           &bus_id2act_data_[bus][id].pos, &bus_id2act_data_[bus][id].offset);
      // act_state_interface_.registerHandle(act_state);
      // act_extra_interface_.registerHandle(act_extra);

      // RoboMaster motors are effect actuator
    //   if (type.find("rm") != std::string::npos || type.find("cheetah") != std::string::npos)
    //   {
    //     effort_act_interface_.registerHandle(
    //         hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].exe_effort));
    //   }
    //   else
    //   {
    //     ROS_ERROR_STREAM("Actuator " << it->first << "'s type neither RoboMaster(rm_xxx) nor Cheetah(cheetah_xxx)");
    //     return false;
    //   }
      if (type.find("rm") == std::string::npos && type.find("cheetah") == std::string::npos)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Actuator " << act_data_name << "'s type neither RoboMaster(rm_xxx) nor Cheetah(cheetah_xxx)");
      }
    }
    // registerInterface(&act_state_interface_);
    // registerInterface(&act_extra_interface_);
    // registerInterface(&effort_act_interface_);

    is_actuator_specified_ = true;
  }
  catch (rclcpp::ParameterTypeException e)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(),"Parameter type error: " << e.what() << "\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  catch (rclcpp::exceptions::InvalidParametersException e)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(),"Invalid parameter name: " << e.what() << "\n"
                     << "Please check the configuration.");
    return false;
  }
  return true;
}

// bool rm_hw::RmRobotHW::parseImuData(XmlRpc::XmlRpcValue& imu_datas, ros::NodeHandle& robot_hw_nh)
// {
//   ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
//   try
//   {
//     for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
//     {
//       std::string name = it->first;
//       if (!it->second.hasMember("frame_id"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated frame id.");
//         continue;
//       }
//       else if (!it->second.hasMember("bus"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated bus.");
//         continue;
//       }
//       else if (!it->second.hasMember("id"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated ID.");
//         continue;
//       }
//       else if (!it->second.hasMember("orientation_covariance_diagonal"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated orientation covariance diagonal.");
//         continue;
//       }
//       else if (!it->second.hasMember("angular_velocity_covariance"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated angular velocity covariance.");
//         continue;
//       }
//       else if (!it->second.hasMember("linear_acceleration_covariance"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated linear acceleration covariance.");
//         continue;
//       }
//       else if (!it->second.hasMember("angular_vel_offset"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated angular_vel_offset type");
//         continue;
//       }
//       else if (!it->second.hasMember("angular_vel_coeff"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated angular velocity coefficient.");
//         continue;
//       }
//       else if (!it->second.hasMember("accel_coeff"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated linear acceleration coefficient.");
//         continue;
//       }
//       else if (!it->second.hasMember("temp_coeff"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated temperate coefficient.");
//         continue;
//       }
//       else if (!it->second.hasMember("filter"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated filter type");
//         continue;
//       }
//       else if (!it->second.hasMember("angular_vel_offset"))
//       {
//         ROS_ERROR_STREAM("Imu " << name << " has no associated angular_vel_offset type");
//         continue;
//       }
//       XmlRpc::XmlRpcValue angular_vel_offsets = imu_datas[name]["angular_vel_offset"];
//       ROS_ASSERT(angular_vel_offsets.getType() == XmlRpc::XmlRpcValue::TypeArray);
//       ROS_ASSERT(angular_vel_offsets.size() == 3);
//       for (int i = 0; i < angular_vel_offsets.size(); ++i)
//         ROS_ASSERT(angular_vel_offsets[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
//       XmlRpc::XmlRpcValue ori_cov = imu_datas[name]["orientation_covariance_diagonal"];
//       ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
//       ROS_ASSERT(ori_cov.size() == 3);
//       for (int i = 0; i < ori_cov.size(); ++i)
//         ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
//       XmlRpc::XmlRpcValue angular_cov = imu_datas[name]["angular_velocity_covariance"];
//       ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
//       ROS_ASSERT(angular_cov.size() == 3);
//       for (int i = 0; i < angular_cov.size(); ++i)
//         ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
//       XmlRpc::XmlRpcValue linear_cov = imu_datas[name]["linear_acceleration_covariance"];
//       ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
//       ROS_ASSERT(linear_cov.size() == 3);
//       for (int i = 0; i < linear_cov.size(); ++i)
//         ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
//       std::string filter_type = imu_datas[name]["filter"];
//       // TODO(Zhenyu Ye): Add more types of filter.
//       rm_common::ImuFilterBase* imu_filter;
//       if (filter_type.find("complementary") != std::string::npos)
//         imu_filter = new rm_common::ImuComplementaryFilter;
//       else
//       {
//         ROS_ERROR_STREAM("Imu " << name << " doesn't has filter type " << filter_type);
//         return false;
//       }
//       imu_filter->init(it->second, name);

//       std::string frame_id = imu_datas[name]["frame_id"], bus = imu_datas[name]["bus"];
//       int id = static_cast<int>(imu_datas[name]["id"]);

//       // for bus interface
//       if (bus_id2imu_data_.find(bus) == bus_id2imu_data_.end())
//         bus_id2imu_data_.insert(std::make_pair(bus, std::unordered_map<int, ImuData>()));

//       if (!(bus_id2imu_data_[bus].find(id) == bus_id2imu_data_[bus].end()))
//       {
//         ROS_ERROR_STREAM("Repeat Imu on bus " << bus << " and ID " << id);
//         return false;
//       }
//       else
//         bus_id2imu_data_[bus].insert(std::make_pair(
//             id, ImuData{ .time_stamp = {},
//                          .imu_name = name,
//                          .ori = {},
//                          .angular_vel = {},
//                          .linear_acc = {},
//                          .angular_vel_offset = { static_cast<double>(angular_vel_offsets[0]),
//                                                  static_cast<double>(angular_vel_offsets[1]),
//                                                  static_cast<double>(angular_vel_offsets[2]) },
//                          .ori_cov = { static_cast<double>(ori_cov[0]), 0., 0., 0., static_cast<double>(ori_cov[1]), 0.,
//                                       0., 0., static_cast<double>(ori_cov[2]) },
//                          .angular_vel_cov = { static_cast<double>(angular_cov[0]), 0., 0., 0.,
//                                               static_cast<double>(angular_cov[1]), 0., 0., 0.,
//                                               static_cast<double>(angular_cov[2]) },
//                          .linear_acc_cov = { static_cast<double>(linear_cov[0]), 0., 0., 0.,
//                                              static_cast<double>(linear_cov[1]), 0., 0., 0.,
//                                              static_cast<double>(linear_cov[2]) },
//                          .temperature = 0.0,
//                          .angular_vel_coeff = xmlRpcGetDouble(imu_datas[name], "angular_vel_coeff", 0.),
//                          .accel_coeff = xmlRpcGetDouble(imu_datas[name], "accel_coeff", 0.),
//                          .temp_coeff = xmlRpcGetDouble(imu_datas[name], "temp_coeff", 0.),
//                          .temp_offset = xmlRpcGetDouble(imu_datas[name], "temp_offset", 0.),
//                          .accel_updated = false,
//                          .gyro_updated = false,
//                          .camera_trigger = false,
//                          .enabled_trigger = false,
//                          .imu_filter = imu_filter }));
//       // for ros_control interface
//       hardware_interface::ImuSensorHandle imu_sensor_handle(
//           name, frame_id, bus_id2imu_data_[bus][id].ori, bus_id2imu_data_[bus][id].ori_cov,
//           bus_id2imu_data_[bus][id].angular_vel, bus_id2imu_data_[bus][id].angular_vel_cov,
//           bus_id2imu_data_[bus][id].linear_acc, bus_id2imu_data_[bus][id].linear_acc_cov);
//       imu_sensor_interface_.registerHandle(imu_sensor_handle);
//       rm_imu_sensor_interface_.registerHandle(
//           rm_control::RmImuSensorHandle(imu_sensor_handle, &bus_id2imu_data_[bus][id].time_stamp));
//     }
//     registerInterface(&imu_sensor_interface_);
//     registerInterface(&rm_imu_sensor_interface_);
//   }
//   catch (XmlRpc::XmlRpcException& e)
//   {
//     ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
//                      << "configuration: " << e.getMessage() << ".\n"
//                      << "Please check the configuration, particularly parameter types.");
//     return false;
//   }
//   return true;
// }

// bool RmRobotHW::parseGpioData(XmlRpc::XmlRpcValue& gpio_datas, ros::NodeHandle& robot_hw_nh)
// {
//   for (auto it = gpio_datas.begin(); it != gpio_datas.end(); ++it)
//   {
//     if (it->second.hasMember("pin"))
//     {
//       rm_control::GpioData gpio_data;
//       gpio_data.name = it->first;
//       if (std::string(gpio_datas[it->first]["direction"]) == "in")
//       {
//         gpio_data.type = rm_control::INPUT;
//       }
//       else if (std::string(gpio_datas[it->first]["direction"]) == "out")
//       {
//         gpio_data.type = rm_control::OUTPUT;
//       }
//       else
//       {
//         ROS_ERROR("Type set error of %s!", it->first.data());
//         continue;
//       }
//       gpio_data.pin = gpio_datas[it->first]["pin"];
//       gpio_data.value = new bool(false);
//       gpio_manager_.setGpioDirection(gpio_data);
//       gpio_manager_.gpio_state_values.push_back(gpio_data);
//       rm_control::GpioStateHandle gpio_state_handle(it->first, gpio_data.type,
//                                                     gpio_manager_.gpio_state_values.back().value);
//       gpio_state_interface_.registerHandle(gpio_state_handle);

//       if (gpio_data.type == rm_control::OUTPUT)
//       {
//         gpio_manager_.gpio_command_values.push_back(gpio_data);
//         rm_control::GpioCommandHandle gpio_command_handle(it->first, gpio_data.type,
//                                                           gpio_manager_.gpio_command_values.back().value);
//         gpio_command_interface_.registerHandle(gpio_command_handle);
//       }
//     }
//     else
//     {
//       ROS_ERROR("Module %s hasn't set pin ID", it->first.data());
//     }
//   }
//   return true;
// }

// bool rm_hw::RmRobotHW::parseTofData(XmlRpc::XmlRpcValue& tof_datas, ros::NodeHandle& robot_hw_nh)
// {
//   ROS_ASSERT(tof_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
//   try
//   {
//     for (auto it = tof_datas.begin(); it != tof_datas.end(); ++it)
//     {
//       if (!it->second.hasMember("bus"))
//       {
//         ROS_ERROR_STREAM("TOF02-i " << it->first << " has no associated bus.");
//         continue;
//       }
//       else if (!it->second.hasMember("id"))
//       {
//         ROS_ERROR_STREAM("TOF02-i " << it->first << " has no associated ID.");
//         continue;
//       }

//       std::string bus = tof_datas[it->first]["bus"];
//       int id = static_cast<int>(tof_datas[it->first]["id"]);

//       // for bus interface
//       if (bus_id2tof_data_.find(bus) == bus_id2tof_data_.end())
//         bus_id2tof_data_.insert(std::make_pair(bus, std::unordered_map<int, TofData>()));

//       if (!(bus_id2tof_data_[bus].find(id) == bus_id2tof_data_[bus].end()))
//       {
//         ROS_ERROR_STREAM("Repeat TF02 on bus " << bus << " and ID " << id);
//         return false;
//       }
//       else
//         bus_id2tof_data_[bus].insert(std::make_pair(id, TofData{ .strength = {}, .distance = {} }));
//       // for ros_control interface
//       rm_control::TofRadarHandle tof_radar_handle(it->first, &bus_id2tof_data_[bus][id].distance,
//                                                   &bus_id2tof_data_[bus][id].strength);
//       tof_radar_interface_.registerHandle(tof_radar_handle);
//     }
//     registerInterface(&tof_radar_interface_);
//   }
//   catch (XmlRpc::XmlRpcException& e)
//   {
//     ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
//                      << "configuration: " << e.getMessage() << ".\n"
//                      << "Please check the configuration, particularly parameter types.");
//     return false;
//   }
//   return true;
// }

bool RmHardwareSystem::setupTransmission()
{
  if (!is_actuator_specified_)
    return true;
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();
  for (const auto& transmission_info : info_.transmissions)
  {
    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException& e)
    {
      RCLCPP_FATAL(node_->get_logger(), "Error while loading %s: %s", transmission_info.name.c_str(), e.what());
      return false;
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto& joint_info : transmission_info.joints)
    {
      
      const auto joint_interface = 
          joint_name2joint_interfaces_.emplace_hint(joint_name2joint_interfaces_.end(), std::make_pair(joint_info.name, InterfaceData()));
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_POSITION,
                                 &joint_interface->second.transmissionPassthrough_[0]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_VELOCITY,
                                 &joint_interface->second.transmissionPassthrough_[1]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_EFFORT,
                                 &joint_interface->second.transmissionPassthrough_[2]);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto& actuator_info : transmission_info.actuators)
    {
      const auto actuator_interface =
          actuator_name2actuator_interfaces_.emplace_hint(actuator_name2actuator_interfaces_.end(), std::make_pair(actuator_info.name, InterfaceData()));
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_POSITION,
                                    &actuator_interface->second.transmissionPassthrough_[0]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_VELOCITY,
                                    &actuator_interface->second.transmissionPassthrough_[1]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_EFFORT,
                                    &actuator_interface->second.transmissionPassthrough_[2]);
    }

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& e)
    {
      RCLCPP_FATAL(node_->get_logger(), "Error while configuring %s: %s", transmission_info.name.c_str(), e.what());
      return false;
    }

    transmissions_.push_back(transmission);
  }

  return true;
}

}  // namespace rm2_hw
