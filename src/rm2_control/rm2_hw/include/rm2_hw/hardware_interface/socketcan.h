//
// Created by ch on 2025/10/15.
//

#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
// Multi-threading
#include <pthread.h>
#include <boost/function.hpp>

namespace can
{
class SocketCAN
{
private:
  rclcpp::Node::SharedPtr node_;
  ifreq interface_request_{};
  sockaddr_can address_{};
  pthread_t receiver_thread_id_{};

public:
  /**
   * CAN socket file descriptor
   */
  int sock_fd_ = -1;
  /**
   * Request for the child thread to terminate
   */
  bool terminate_receiver_thread_ = false;
  bool receiver_thread_running_ = false;

  SocketCAN() = default;
  ~SocketCAN();

  /** \brief Open and bind socket.
   *
   * \param interface bus's name(example: can0).
   * \param handler Pointer to a function which shall be called when frames are being received from the CAN bus.
   *
   * \returns \c true if it successfully open and bind socket.
   */
  bool open(rclcpp::Node::SharedPtr node, const std::string& interface, boost::function<void(const can_frame& frame)> handler, int thread_priority);
  /** \brief Close and unbind socket.
   *
   */
  void close();
  /** \brief Returns whether the socket is open or closed.
   *
   * \returns \c True if socket has opened.
   */
  bool isOpen() const;
  /** \brief Sends the referenced frame to the bus.
   *
   * \param frame referenced frame which you want to send.
   */
  void write(can_frame* frame) const;
  /** \brief Starts a new thread, that will wait for socket events.
   *
   */
  bool startReceiverThread(int thread_priority);
  /**
   * Pointer to a function which shall be called
   * when frames are being received from the CAN bus
   */
  boost::function<void(const can_frame& frame)> reception_handler;
};

}  // namespace can
