//
// Created by ch on 2025/10/5.
//

#pragma once

#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace rm2_control
{
class ImuSensorHandle
{
public:
  struct Data
  {
    Data() {}
 
    std::string name;                                   
    std::string frame_id;                               
    double* orientation                    = {nullptr}; 
    double* orientation_covariance         = {nullptr}; 
    double* angular_velocity               = {nullptr}; 
    double* angular_velocity_covariance    = {nullptr}; 
    double* linear_acceleration            = {nullptr}; 
    double* linear_acceleration_covariance = {nullptr}; 
  };
 
  ImuSensorHandle(const Data& data = {})
    : name_(data.name),
      frame_id_(data.frame_id),
      orientation_(data.orientation),
      orientation_covariance_(data.orientation_covariance),
      angular_velocity_(data.angular_velocity),
      angular_velocity_covariance_(data.angular_velocity_covariance),
      linear_acceleration_(data.linear_acceleration),
      linear_acceleration_covariance_(data.linear_acceleration_covariance)
  {}
 
  ImuSensorHandle(
        const std::string& name,                      
        const std::string& frame_id,                  
        const double* orientation,                    
        const double* orientation_covariance,         
        const double* angular_velocity,               
        const double* angular_velocity_covariance,    
        const double* linear_acceleration,            
        const double* linear_acceleration_covariance  
    )
    : name_(name),
      frame_id_(frame_id),
      orientation_(orientation),
      orientation_covariance_(orientation_covariance),
      angular_velocity_(angular_velocity),
      angular_velocity_covariance_(angular_velocity_covariance),
      linear_acceleration_(linear_acceleration),
      linear_acceleration_covariance_(linear_acceleration_covariance)
  {}
 
  std::string getName()                           const {return name_;}
  std::string getFrameId()                        const {return frame_id_;}
  const double* getOrientation()                  const {return orientation_;}
  const double* getOrientationCovariance()        const {return orientation_covariance_;}
  const double* getAngularVelocity()              const {return angular_velocity_;}
  const double* getAngularVelocityCovariance()    const {return angular_velocity_covariance_;}
  const double* getLinearAcceleration()           const {return linear_acceleration_;}
  const double* getLinearAccelerationCovariance() const {return linear_acceleration_covariance_;}
 
private:
  std::string name_;
  std::string frame_id_;
 
  const double* orientation_;
  const double* orientation_covariance_;
  const double* angular_velocity_;
  const double* angular_velocity_covariance_;
  const double* linear_acceleration_;
  const double* linear_acceleration_covariance_;
};



class RmImuSensorHandle : public ImuSensorHandle
{
public:
  RmImuSensorHandle() = default;

  RmImuSensorHandle(const ImuSensorHandle& imu_sensor_handle, rclcpp::Time* time_stamp)
    : ImuSensorHandle(imu_sensor_handle), time_stamp_(time_stamp)
  {
    if (!time_stamp_)
    {
      throw std::runtime_error("Cannot create handle '" + imu_sensor_handle.getName() +
                                                           "'. Time stamp pointer is null");
    }
  }
  rclcpp::Time getTimeStamp()
  {
    assert(time_stamp_);
    return *time_stamp_;
  }

private:
  rclcpp::Time* time_stamp_ = { nullptr };
};

}  // namespace rm2_control
