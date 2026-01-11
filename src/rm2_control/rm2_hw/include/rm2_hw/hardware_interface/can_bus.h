//
// Created by ch on 2025/10/17.
//

#pragma once

#include "rm2_hw/hardware_interface/socketcan.h"
#include "rm2_hw/hardware_interface/types.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace rm2_hw
{
struct CanFrameStamp
{
  can_frame frame;
  rclcpp::Time stamp;
};

class CanBus
{
public:
  /** \brief
   * Initialize device at can_device, retry if fail. Set up header of CAN frame.
   *
   * \param bus_name Bus's name(example: can0).
   * \param data_ptr Pointer which point to CAN data.
   */
  CanBus(rclcpp::Node::SharedPtr node, const std::string& bus_name, CanDataPtr data_ptr, int thread_priority);
  /** \brief Read active data from read_buffer_ to data_ptr_, such as position, velocity, torque and so on. Clear
   * read_buffer_ after reading.
   *
   * \param time ROS time, but it doesn't be used.
   */
  void read(rclcpp::Time time);
  /** \brief Write commands to can bus.
   *
   */
  void write();

  void write(can_frame* frame);

  const std::string bus_name_;

private:
  /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
   *
   * @param frame The frame which socketcan receive.
   */
  void frameCallback(const can_frame& frame);

  rclcpp::Node::SharedPtr node_;
  can::SocketCAN socket_can_;
  CanDataPtr data_ptr_;
  std::vector<CanFrameStamp> read_buffer_;

  can_frame rm_frame0_{};  // for id 0x201~0x204
  can_frame rm_frame1_{};  // for id 0x205~0x208

  mutable std::mutex mutex_;
};

}  // namespace rm2_hw
