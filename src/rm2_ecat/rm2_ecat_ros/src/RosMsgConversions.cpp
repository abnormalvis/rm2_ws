//
// Created by qiayuan on 23-4-13.
//

#include "rm2_ecat_ros/RosMsgConversions.h"

namespace rm2_ecat {

rclcpp::Time createRosTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint) {
  return rclcpp::Time(static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(timePoint.time_since_epoch()).count()),
          static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint.time_since_epoch()).count() % 1000000000));
}

rm2_ecat_msgs::msg::RmEcatStandardSlaveReading createRmSlaveReadingMsg(const rm2_ecat::standard::Reading& reading) {
  rm2_ecat_msgs::msg::RmEcatStandardSlaveReading readingMsg;
  readingMsg.stamp = createRosTime(reading.getStamp());
  readingMsg.statusword = reading.getStatusword().getRaw();

  auto ids = reading.getEnabledMotorIds(standard::CanBus::CAN0);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(standard::CanBus::CAN0, id));
    readingMsg.is_online.push_back(reading.getStatusword().isOnline(standard::CanBus::CAN0, id));
    //    readingMsg.is_over_temperature.push_back(reading.getStatusword().isOverTemperature(CanBus::CAN0, id));
    readingMsg.position.push_back(reading.getPosition(standard::CanBus::CAN0, id));
    readingMsg.velocity.push_back(reading.getVelocity(standard::CanBus::CAN0, id));
    readingMsg.torque.push_back(reading.getTorque(standard::CanBus::CAN0, id));
    readingMsg.temperature.push_back(reading.getTemperature(standard::CanBus::CAN0, id));
  }
  ids = reading.getEnabledMotorIds(standard::CanBus::CAN1);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(standard::CanBus::CAN1, id));
    readingMsg.is_online.push_back(reading.getStatusword().isOnline(standard::CanBus::CAN1, id));
    //    readingMsg.is_over_temperature.push_back(reading.getStatusword().isOverTemperature(CanBus::CAN1, id));
    readingMsg.position.push_back(reading.getPosition(standard::CanBus::CAN1, id));
    readingMsg.velocity.push_back(reading.getVelocity(standard::CanBus::CAN1, id));
    readingMsg.torque.push_back(reading.getTorque(standard::CanBus::CAN1, id));
    readingMsg.temperature.push_back(reading.getTemperature(standard::CanBus::CAN1, id));
  }

  return readingMsg;
}

rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings createRmSlaveReadingsMsg(const std::vector<std::string>& names,
                                                                   const std::vector<rm2_ecat::standard::Reading>& readings) {
  rm2_ecat_msgs::msg::RmEcatStandardSlaveReadings readingsMsg;
  for (size_t i = 0; i < names.size(); i++) {
    readingsMsg.names.push_back(names[i]);
    readingsMsg.readings.push_back(createRmSlaveReadingMsg(readings[i]));
  }

  return readingsMsg;
}

sensor_msgs::msg::JointState createRmJointStateMsg(const std::vector<rm2_ecat::standard::Reading>& readings) {
  sensor_msgs::msg::JointState jointState;
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(standard::CanBus::CAN0);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(standard::CanBus::CAN0, id));
      jointState.position.push_back(reading.getPosition(standard::CanBus::CAN0, id));
      jointState.velocity.push_back(reading.getVelocity(standard::CanBus::CAN0, id));
      jointState.effort.push_back(reading.getTorque(standard::CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(standard::CanBus::CAN1);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(standard::CanBus::CAN1, id));
      jointState.position.push_back(reading.getPosition(standard::CanBus::CAN1, id));
      jointState.velocity.push_back(reading.getVelocity(standard::CanBus::CAN1, id));
      jointState.effort.push_back(reading.getTorque(standard::CanBus::CAN1, id));
    }
  }

  return jointState;
}

sensor_msgs::msg::JointState createMitJointStateMsg(const std::vector<rm2_ecat::mit::Reading>& readings) {
  sensor_msgs::msg::JointState jointState;
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(mit::CanBus::CAN0);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(mit::CanBus::CAN0, id));
      jointState.position.push_back(reading.getPosition(mit::CanBus::CAN0, id));
      jointState.velocity.push_back(reading.getVelocity(mit::CanBus::CAN0, id));
      jointState.effort.push_back(reading.getTorque(mit::CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(mit::CanBus::CAN1);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(mit::CanBus::CAN1, id));
      jointState.position.push_back(reading.getPosition(mit::CanBus::CAN1, id));
      jointState.velocity.push_back(reading.getVelocity(mit::CanBus::CAN1, id));
      jointState.effort.push_back(reading.getTorque(mit::CanBus::CAN1, id));
    }
  }

  return jointState;
}

rm2_ecat_msgs::msg::RmEcatMitSlaveReading createMitSlaveReadingMsg(const rm2_ecat::mit::Reading& reading) {
  rm2_ecat_msgs::msg::RmEcatMitSlaveReading readingMsg;
  readingMsg.stamp = createRosTime(reading.getStamp());
  readingMsg.statusword = reading.getStatusword().getRaw();

  auto ids = reading.getEnabledMotorIds(mit::CanBus::CAN0);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(mit::CanBus::CAN0, id));
    readingMsg.is_online.push_back(reading.getStatusword().isOnline(mit::CanBus::CAN0, id));
    readingMsg.position.push_back(reading.getPosition(mit::CanBus::CAN0, id));
    readingMsg.velocity.push_back(reading.getVelocity(mit::CanBus::CAN0, id));
    readingMsg.torque.push_back(reading.getTorque(mit::CanBus::CAN0, id));
  }
  ids = reading.getEnabledMotorIds(mit::CanBus::CAN1);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(mit::CanBus::CAN1, id));
    readingMsg.is_online.push_back(reading.getStatusword().isOnline(mit::CanBus::CAN1, id));
    readingMsg.position.push_back(reading.getPosition(mit::CanBus::CAN1, id));
    readingMsg.velocity.push_back(reading.getVelocity(mit::CanBus::CAN1, id));
    readingMsg.torque.push_back(reading.getTorque(mit::CanBus::CAN1, id));
  }

  return readingMsg;
}

rm2_ecat_msgs::msg::RmEcatMitSlaveReadings createMitSlaveReadingsMsg(const std::vector<std::string>& names,
                                                               const std::vector<rm2_ecat::mit::Reading>& readings) {
  rm2_ecat_msgs::msg::RmEcatMitSlaveReadings readingsMsg;
  for (size_t i = 0; i < names.size(); i++) {
    readingsMsg.names.push_back(names[i]);
    readingsMsg.readings.push_back(createMitSlaveReadingMsg(readings[i]));
  }

  return readingsMsg;
}

sensor_msgs::msg::Imu createImuMsg(standard::CanBus bus, const rm2_ecat::standard::Reading& reading) {
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.stamp = createRosTime(reading.getStamp());
  imuMsg.header.frame_id = reading.getImuName(bus);
  reading.getOrientation(bus, imuMsg.orientation.w, imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z);
  reading.getLinearAcceleration(bus, imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
  reading.getAngularVelocity(bus, imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z);
  return imuMsg;
}

std::vector<sensor_msgs::msg::Imu> createImuMsgs(const std::vector<rm2_ecat::standard::Reading>& readings) {
  std::vector<sensor_msgs::msg::Imu> imuMsgs;
  for (const auto& reading : readings) {
    const auto imuBuss = reading.getEnabledImuBuss();
    for (const auto& bus : imuBuss) {
      imuMsgs.push_back(createImuMsg(bus, reading));
    }
  }
  return imuMsgs;
}

static rclcpp::Time lastDbusStamp;
rm2_msgs::msg::DbusData createDbusData(const rm2_ecat::standard::Reading& reading) {
  auto data = reading.getDbusData();
  if (reading.getDbusStatus()) {
    data.stamp = createRosTime(reading.getStamp());
    lastDbusStamp = data.stamp;
  }
  data.stamp = lastDbusStamp;
  return data;
}

std::vector<rm2_msgs::msg::DbusData> createDbusDatas(const std::vector<rm2_ecat::standard::Reading>& readings)
{
  std::vector<rm2_msgs::msg::DbusData> dbusDatas;
  for (const auto& reading : readings) {
    if(reading.getEnabledDbus())
    {
      dbusDatas.push_back(createDbusData(reading));
    }
  }
  return dbusDatas;
}

rm2_msgs::msg::GpioData createRmGpioData(const rm2_ecat::standard::Reading& reading) {
  rm2_msgs::msg::GpioData data;
  auto ids = reading.getEnabledDigitalInputIds();
  for (const auto& id : ids) {
    data.gpio_name.emplace_back(reading.getGpioName(id));
    data.gpio_state.push_back(reading.getDigitalInput(id));
    data.gpio_type.emplace_back("in");
  }
  data.header.stamp = createRosTime(reading.getStamp());
  return data;
}

rm2_msgs::msg::GpioData createMitGpioData(const rm2_ecat::mit::Reading& reading) {
  rm2_msgs::msg::GpioData data;
  auto ids = reading.getEnabledDigitalInputIds();
  for (const auto& id : ids) {
    data.gpio_name.emplace_back(reading.getGpioName(id));
    data.gpio_state.push_back(reading.getDigitalInput(id));
    data.gpio_type.emplace_back("in");
  }
  data.header.stamp = createRosTime(reading.getStamp());
  return data;
}

std::vector<rm2_msgs::msg::GpioData> createRmGpioDatas(const std::vector<rm2_ecat::standard::Reading>& readings) {
  std::vector<rm2_msgs::msg::GpioData> gpioDatas;
  gpioDatas.reserve(readings.size());
  for (const auto& reading : readings) {
      gpioDatas.push_back(createRmGpioData(reading));
  }
  return gpioDatas;
}

std::vector<rm2_msgs::msg::GpioData> createMitGpioDatas(const std::vector<rm2_ecat::mit::Reading>& readings) {
  std::vector<rm2_msgs::msg::GpioData> gpioDatas;
  gpioDatas.reserve(readings.size());
  for (const auto& reading : readings) {
      gpioDatas.push_back(createMitGpioData(reading));
  }
  return gpioDatas;
}

sensor_msgs::msg::TimeReference createTimeReferenceMsg(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint) {
  sensor_msgs::msg::TimeReference timeReferenceMsg;
  timeReferenceMsg.header.stamp = createRosTime(timePoint);
  timeReferenceMsg.time_ref = timeReferenceMsg.header.stamp;
  return timeReferenceMsg;
}

}  // namespace rm2_ecat
