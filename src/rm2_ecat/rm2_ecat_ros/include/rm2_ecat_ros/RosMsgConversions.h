//
// Created by qiayuan on 23-4-13.
//

#pragma once

#include <rm2_ecat_mit/Reading.h>
#include <rm2_ecat_msgs/msg/rm_ecat_mit_slave_readings.hpp>
#include <rm2_ecat_msgs/msg/rm_ecat_standard_slave_readings.hpp>
#include <rm2_ecat_standard_slave/Reading.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <rclcpp/rclcpp.hpp>

namespace rm2_ecat {

rclcpp::Time createRosTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint);

rm2_ecat_msgs::msg::RmEcatStandardSlaveReading createRmSlaveReadingMsg(const rm2_ecat::standard::Reading& reading);

rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings createRmSlaveReadingsMsg(const std::vector<std::string>& names,
                                                                   const std::vector<rm2_ecat::standard::Reading>& readings);

rm2_ecat_msgs::msg::RmEcatMitSlaveReading createMitSlaveReadingMsg(const rm2_ecat::mit::Reading& reading);

rm2_ecat_msgs::msg::RmEcatMitSlaveReadings createMitSlaveReadingsMsg(const std::vector<std::string>& names,
                                                               const std::vector<rm2_ecat::mit::Reading>& readings);

sensor_msgs::msg::JointState createRmJointStateMsg(const std::vector<rm2_ecat::standard::Reading>& readings);

sensor_msgs::msg::JointState createMitJointStateMsg(const std::vector<rm2_ecat::mit::Reading>& readings);

sensor_msgs::msg::Imu createImuMsg(standard::CanBus bus, const rm2_ecat::standard::Reading& reading);

std::vector<sensor_msgs::msg::Imu> createImuMsgs(const std::vector<rm2_ecat::standard::Reading>& readings);

rm2_msgs::msg::DbusData createDbusData(const rm2_ecat::standard::Reading& reading);

std::vector<rm2_msgs::msg::DbusData> createDbusDatas(const std::vector<rm2_ecat::standard::Reading>& readings);

rm2_msgs::msg::GpioData createRmGpioData(const rm2_ecat::standard::Reading& reading);

std::vector<rm2_msgs::msg::GpioData> createRmGpioDatas(const std::vector<rm2_ecat::standard::Reading>& readings);

rm2_msgs::msg::GpioData createMitGpioData(const rm2_ecat::mit::Reading& reading);

std::vector<rm2_msgs::msg::GpioData> createMitGpioDatas(const std::vector<rm2_ecat::mit::Reading>& readings);

sensor_msgs::msg::TimeReference createTimeReferenceMsg(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint);

}  // namespace rm2_ecat
