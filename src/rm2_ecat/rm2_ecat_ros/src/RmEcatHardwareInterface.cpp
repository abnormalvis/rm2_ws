//
// Created by qiayuan on 23-4-16.
//

#include "rm2_ecat_ros/RmEcatHardwareInterface.h"
#include "rm2_ecat_ros/RosMsgConversions.h"
#include "rm2_ecat_ros/types.h"
#include "rm2_msgs/msg/bus_state.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_limits_urdf.hpp>
#include <memory>
#include <message_logger/log/log_messages.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <unordered_map>
#include <pluginlib/class_list_macros.hpp>

namespace rm2_ecat {
CallbackReturn RmEcatHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  node_ = get_node()->create_sub_node("rm2_ecat");
  std::string name = "rm_ecat_hw";
  rmStandardSlaveManager_ = std::make_shared<rm2_ecat::standard::RmStandardSlaveManagerRos>(false, false, 0.001, node_, name);
  rmMitSlaveManager_ = std::make_shared<rm2_ecat::mit::RmMitSlaveManagerRos>(false, false, 0.001, node_, name);
  std::string setupFile, desc;
  setupFile = node_->declare_parameter("setupFile", "");
  busManager_ = std::make_shared<ecat_manager::EcatBusManager>();
  busManager_->fromFile(setupFile, false);
  busManager_->startupCommunication();
  rmStandardSlaveManager_->setBusManager(busManager_);
  rmMitSlaveManager_->setBusManager(busManager_);

  busNames_ = busManager_->getAllBusNames();
  for (const auto& busName : busNames_) {
    busIsOk_.emplace(busName, true);
  }
  busStatesPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm2_msgs::msg::BusState>>(
      node_->create_publisher<rm2_msgs::msg::BusState>("bus_state", 10), 10, false);
  if (!rmStandardSlaveManager_->getDigitalOutputNames().empty()) {
    rmGpioOutputsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm2_msgs::msg::GpioData>>(
        node_->create_publisher<rm2_msgs::msg::GpioData>("gpio_expect_outputs/rm", 10), 20, false);
  }
  if (!rmMitSlaveManager_->getDigitalOutputNames().empty()) {
    mitGpioOutputsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm2_msgs::msg::GpioData>>(
        node_->create_publisher<rm2_msgs::msg::GpioData>("gpio_expect_outputs/mit", 10), 20, false);
  }

  if (!rmStandardSlaveManager_->startup()) {
    return CallbackReturn::ERROR;
  }
  if (!rmMitSlaveManager_->startup()) {
    return CallbackReturn::ERROR;
  }

  updateWorker_ = std::make_shared<any_worker::Worker>("updateWorker", rmStandardSlaveManager_->getTimeStep(),
                                                        [this](auto && PH1) { return updateWorkerCb(std::forward<decltype(PH1)>(PH1)); });
  publishWorker_ = std::make_shared<any_worker::Worker>("PublishWorker", 10 * rmStandardSlaveManager_->getTimeStep(),
                                                        [this](auto && PH1) { return publishWorkerCb(std::forward<decltype(PH1)>(PH1)); });
  signal_handler::SignalHandler::bindAll(&RmEcatHardwareInterface::handleSignal, this);

  ////////// ros-control //////////
  if (!loadUrdf(node_)) {
    MELO_ERROR("Error occurred while setting up urdf");
    return CallbackReturn::ERROR;
  }
  setupActuators();
  if (!setupTransmission()) {
    MELO_ERROR("Error occurred while setting up transmission");
    return CallbackReturn::ERROR;
  }
  if (!setupJointLimit()) {
    MELO_ERROR("Error occurred while setting up joint limit");
    return CallbackReturn::ERROR;
  }
  setupImus();
  setupGpios();

  updateWorker_->start(48);
  publishWorker_->start(20);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RmEcatHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  MELO_INFO("on_configure called");
  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmEcatHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  MELO_INFO("on_activate called");
  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmEcatHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  MELO_INFO("on_deactivate called");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RmEcatHardwareInterface::isRunning() {
  return rmStandardSlaveManager_->isRunning() && rmMitSlaveManager_->isRunning();
}

bool RmEcatHardwareInterface::updateWorkerCb(const any_worker::WorkerEvent& /*unused*/) {
  busManager_->readAllBuses();
  rmStandardSlaveManager_->updateProcessReadings();
  rmMitSlaveManager_->updateProcessReadings();
  rmStandardSlaveManager_->updateSendStagedCommands();
  rmMitSlaveManager_->updateSendStagedCommands();
  busManager_->writeToAllBuses();
  return true;
}

bool RmEcatHardwareInterface::publishWorkerCb(const any_worker::WorkerEvent& /*unused*/) {
  rmStandardSlaveManager_->sendRos();
  rmMitSlaveManager_->sendRos();
  if (busStatesMsgUpdated_) {
    busStatesMsgUpdated_ = false;
    if (busStatesPublisher_) {
      busStatesPublisher_->sendRos();
    }
  }
  if (rmGpioOutputsMsgUpdated_) {
    rmGpioOutputsMsgUpdated_ = false;
    if (rmGpioOutputsPublisher_) {
      rmGpioOutputsPublisher_->sendRos();
    }
  }
  if (mitGpioOutputsMsgUpdated_) {
    mitGpioOutputsMsgUpdated_ = false;
    if (mitGpioOutputsPublisher_) {
      mitGpioOutputsPublisher_->sendRos();
    }
  }
  return true;
}

void RmEcatHardwareInterface::handleSignal(int /*signum*/) {
  updateWorker_->stop();
  publishWorker_->stop();
  rmStandardSlaveManager_->shutdown();
  rmMitSlaveManager_->shutdown();
  busStatesPublisher_->shutdown();
  rmGpioOutputsPublisher_->shutdown();
  mitGpioOutputsPublisher_->shutdown();
  // TODO(ch): set hardware interface deactivate
}

bool RmEcatHardwareInterface::loadUrdf(std::shared_ptr<rclcpp::Node> node) {
  if (urdf_model_ == nullptr) {
    urdf_model_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  urdf_string_ = node->declare_parameter("robot_description", "");
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

void RmEcatHardwareInterface::setupActuators() {
  // rm standard slave
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto motorNames = rmStandardSlaveManager_->getMotorNames();
  auto motorNeedCalibrations = rmStandardSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    act_data_list_.push_back({0., 0., 0., 0., 0., 0., false, motorNeedCalibrations[i], false, false});
    act_state_map_.addMap(motorNames[i], &(act_data_list_.back().pos), 
                                                &(act_data_list_.back().vel), 
                                                &(act_data_list_.back().effort), 
                                   &(act_data_list_.back().commandUnlimited), 
                                            &(act_data_list_.back().command));
    act_extra_map_.addMap(motorNames[i], &(act_data_list_.back().halted), 
                                      &(act_data_list_.back().needCalibration), 
                                            &(act_data_list_.back().calibrated),
                                   &(act_data_list_.back().calibrationReading),
                                                &(act_data_list_.back().offset));
  }

  // mit slave
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  motorNames = rmMitSlaveManager_->getMotorNames();
  motorNeedCalibrations = rmMitSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    act_data_list_.push_back({0., 0., 0., 0., 0., 0., false, motorNeedCalibrations[i], false, false});
    act_state_map_.addMap(motorNames[i], &(act_data_list_.back().pos), 
                                                &(act_data_list_.back().vel), 
                                                &(act_data_list_.back().effort), 
                                   &(act_data_list_.back().commandUnlimited), 
                                            &(act_data_list_.back().command));
    act_extra_map_.addMap(motorNames[i], &(act_data_list_.back().halted), 
                                      &(act_data_list_.back().needCalibration), 
                                            &(act_data_list_.back().calibrated),
                                   &(act_data_list_.back().calibrationReading),
                                                &(act_data_list_.back().offset));
  }
}

bool RmEcatHardwareInterface::setupTransmission() {
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();
  for (const auto& transmission_info : info_.transmissions) {
    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException& e)
    {
      MELO_FATAL("Error while loading %s: %s", transmission_info.name.c_str(), e.what());
      return false;
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto& joint_info : transmission_info.joints) {
      const auto joint_interface = 
          joint_transmission_interfaces_.emplace_hint(joint_transmission_interfaces_.end(), std::make_pair(joint_info.name, TransmissionData()));
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_POSITION,
                                 &joint_interface->second.transmissionPassthrough[0]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_VELOCITY,
                                 &joint_interface->second.transmissionPassthrough[1]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_EFFORT,
                                 &joint_interface->second.transmissionPassthrough[2]);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto& actuator_info : transmission_info.actuators) {
      act_extra_map_.setOffset(actuator_info.name, actuator_info.offset);
      const auto actuator_interface =
          actuator_transmission_interfaces_.emplace_hint(actuator_transmission_interfaces_.end(), std::make_pair(actuator_info.name, TransmissionData()));
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_POSITION,
                                    &actuator_interface->second.transmissionPassthrough[0]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_VELOCITY,
                                    &actuator_interface->second.transmissionPassthrough[1]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_EFFORT,
                                    &actuator_interface->second.transmissionPassthrough[2]);
    }

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& e)
    {
      MELO_FATAL("Error while configuring %s: %s", transmission_info.name.c_str(), e.what());
      return false;
    }

    transmissions_.push_back(transmission);
  }

  return true;
}

bool RmEcatHardwareInterface::setupJointLimit() {
  return joint_limiters_.init(info_, 
    node_->get_node_parameters_interface(), 
  node_->get_node_logging_interface());
}

bool RmEcatHardwareInterface::enforceLimit(const rclcpp::Duration &period) {
  for (auto joint : info_.joints) {
    std::string joint_name = joint.name;
    
    joint_limits::JointControlInterfacesData current_joint_states, desired_joint_states;
    current_joint_states.joint_name = desired_joint_states.joint_name =joint_name;
    current_joint_states.position = get_state(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    current_joint_states.velocity = get_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    current_joint_states.effort = get_state(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    desired_joint_states.effort = get_command(joint_name + "/" + hardware_interface::HW_IF_EFFORT);

    if (joint_limiters_.enforceJoint(joint_name, current_joint_states, desired_joint_states, period)) {
      set_command(joint_name + "/" + hardware_interface::HW_IF_EFFORT, desired_joint_states.effort.value());
    } else {
      set_command(joint_name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
      return false;
    }
  }
  return true;
}

void RmEcatHardwareInterface::setupImus() {
  size_t num_imus = rmStandardSlaveManager_->getImuNames().size();
  auto imuNames = rmStandardSlaveManager_->getImuNames();
  for (size_t i = 0; i < num_imus; ++i) {
    imu_data_list_.push_back({});
    imu_sensor_map_.addMap(imuNames[i], &(imu_data_list_.back().orientation), 
                                                &(imu_data_list_.back().angularVel), 
                                               &(imu_data_list_.back().linearAccel), 
                                     &(imu_data_list_.back().orientationCovariance), 
                                 &(imu_data_list_.back().angularVelocityCovariance), 
                              &(imu_data_list_.back().linearAccelerationCovariance),
                                                     &(imu_data_list_.back().stamp));
  }
}

void RmEcatHardwareInterface::setupGpios() {
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputNames().size();
  auto DigitalInputNames = rmStandardSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    digital_input_list_.push_back({});
    gpio_state_map_.addMap(DigitalInputNames[i], GpioType::INPUT, &(digital_input_list_.back()));
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputNames().size();
  DigitalInputNames = rmMitSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    digital_input_list_.push_back({});
    gpio_state_map_.addMap(DigitalInputNames[i], GpioType::INPUT, &(digital_input_list_.back()));
  }

  size_t num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  auto DigitalOutputNames = rmStandardSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    digital_output_list_.push_back({});
    gpio_state_map_.addMap(DigitalOutputNames[i], GpioType::OUTPUT, &(digital_output_list_.back()));
  }
  num_outputs = rmMitSlaveManager_->getDigitalOutputNames().size();
  DigitalOutputNames = rmMitSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    digital_output_list_.push_back({});
    gpio_state_map_.addMap(DigitalOutputNames[i], GpioType::OUTPUT, &(digital_output_list_.back()));
  }
}

std::vector<hardware_interface::InterfaceDescription> RmEcatHardwareInterface::export_unlisted_state_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> ecat_unlisted_state_interfaces;

  std::unordered_map<std::string, std::string> act_state_interface_type{{"pos", "double"}, 
                                                                        {"vel", "double"},
                                                                        {"eff", "double"}}; 

  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    for (auto it : act_state_interface_type) {
      hardware_interface::InterfaceInfo act_state_interface;
      act_state_interface.name = "act_state/" + it.first;
      act_state_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, act_state_interface));
    }
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    for (auto it : act_state_interface_type) {
      hardware_interface::InterfaceInfo act_state_interface;
      act_state_interface.name = "act_state/" + it.first;
      act_state_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, act_state_interface));
    }
  }

  std::unordered_map<std::string, std::string> act_extra_interface_type{{"halted", "bool"}, 
                                                                        {"need_calibration", "bool"},
                                                                        {"calibrated", "bool"}, 
                                                                        {"calibration_reading", "bool"}, 
                                                                        {"offset", "double"}}; 
  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    for (auto it : act_extra_interface_type) {
      hardware_interface::InterfaceInfo act_extra_interface;
      act_extra_interface.name = "act_extra/" + it.first;
      act_extra_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, act_extra_interface));
    }
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    for (auto it : act_extra_interface_type) {
      hardware_interface::InterfaceInfo act_extra_interface;
      act_extra_interface.name = "act_extra/" + it.first;
      act_extra_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, act_extra_interface));
    }
  }

  std::unordered_map<std::string, std::string> imu_sensor_interface_type{{"orientation/x", "double"}, 
                                                                         {"orientation/y", "double"},
                                                                         {"orientation/z", "double"}, 
                                                                         {"orientation/w", "double"}, 
                                                                         {"angular_vel/x", "double"}, 
                                                                         {"angular_vel/y", "double"}, 
                                                                         {"angular_vel/z", "double"}, 
                                                                         {"linear_accel/x", "double"}, 
                                                                         {"linear_accel/y", "double"}, 
                                                                         {"linear_accel/z", "double"}, 
                                                                        {"orientation_covariance[1]", "double"}, 
                                                                        {"orientation_covariance[2]", "double"}, 
                                                                        {"orientation_covariance[3]", "double"}, 
                                                                        {"angular_velocity_covariance[1]", "double"}, 
                                                                        {"angular_velocity_covariance[2]", "double"}, 
                                                                        {"angular_velocity_covariance[3]", "double"}, 
                                                                        {"linear_acceleration_covariance[1]", "double"}, 
                                                                        {"linear_acceleration_covariance[2]", "double"}, 
                                                                        {"linear_acceleration_covariance[3]", "double"}}; 

  for (auto imu_name : rmStandardSlaveManager_->getImuNames()) {
    for (auto it : imu_sensor_interface_type) {
      hardware_interface::InterfaceInfo imu_sensor_interface;
      imu_sensor_interface.name = it.first;
      imu_sensor_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(imu_name, imu_sensor_interface));
    }
  }

  std::unordered_map<std::string, std::string> gpio_interface_type{{"type", "double"}, 
                                                                   {"value", "bool"}}; 
  
  for (auto digital_input_name : rmStandardSlaveManager_->getDigitalInputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_input_interface;
      gpio_input_interface.name = it.first;
      gpio_input_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(digital_input_name, gpio_input_interface));
    }
  }
  for (auto digital_input_name : rmMitSlaveManager_->getDigitalInputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_input_interface;
      gpio_input_interface.name = it.first;
      gpio_input_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(digital_input_name, gpio_input_interface));
    }
  }
  for (auto digital_output_name : rmStandardSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(digital_output_name, gpio_output_interface));
    }
  }
  for (auto digital_output_name : rmMitSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.push_back(hardware_interface::InterfaceDescription(digital_output_name, gpio_output_interface));
    }
  }

  return ecat_unlisted_state_interfaces;
}

std::vector<hardware_interface::InterfaceDescription> RmEcatHardwareInterface::export_unlisted_command_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> ecat_unlisted_command_interfaces;

  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    hardware_interface::InterfaceInfo actuator_calibrated_interface;
    actuator_calibrated_interface.name = "act_extra/calibrated";
    actuator_calibrated_interface.data_type = "bool";
    ecat_unlisted_command_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, actuator_calibrated_interface));
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    hardware_interface::InterfaceInfo actuator_calibrated_interface;
    actuator_calibrated_interface.name = "act_extra/calibrated";
    actuator_calibrated_interface.data_type = "bool";
    ecat_unlisted_command_interfaces.push_back(hardware_interface::InterfaceDescription(motor_name, actuator_calibrated_interface));
  }

  std::unordered_map<std::string, std::string> gpio_interface_type{{"type", "double"}, 
                                                                   {"value", "bool"}}; 

  for (auto digital_output_name : rmStandardSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.push_back(hardware_interface::InterfaceDescription(digital_output_name, gpio_output_interface));
    }
  }
  for (auto digital_output_name : rmMitSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.push_back(hardware_interface::InterfaceDescription(digital_output_name, gpio_output_interface));
    }
  }

  return ecat_unlisted_command_interfaces;
}

hardware_interface::return_type RmEcatHardwareInterface::read(const rclcpp::Time&  /*time*/, const rclcpp::Duration&  /*period*/) {
  // Motors
  const auto& rmMotorIsOnlines = rmStandardSlaveManager_->getMotorIsOnlines();
  const auto& rmMotorPositions = rmStandardSlaveManager_->getMotorPositions();
  const auto& rmMotorVelocities = rmStandardSlaveManager_->getMotorVelocities();
  const auto& rmMotorTorques = rmStandardSlaveManager_->getMotorTorque();

  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto slave_it = act_data_list_.begin();
  for (size_t i = 0; i < num_motors; ++i) {
    slave_it->halted = !rmMotorIsOnlines.at(i);  // TODO: add isOverTemperature
    slave_it->pos = rmMotorPositions.at(i) + slave_it->offset;
    slave_it->vel = rmMotorVelocities.at(i);
    slave_it->effort = rmMotorTorques.at(i);
    slave_it++;
  }

  rmMitSlaveManager_->checkMotorsIsonline();
  const auto& mitMotorIsOnlines = rmMitSlaveManager_->getMotorIsOnlines();
  const auto& mitMotorPositions = rmMitSlaveManager_->getMotorPositions();
  const auto& mitMotorVelocities = rmMitSlaveManager_->getMotorVelocities();
  const auto& mitMotorTorques = rmMitSlaveManager_->getMotorTorque();
  
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    slave_it->halted = !mitMotorIsOnlines.at(i);
    slave_it->pos = mitMotorPositions.at(i) + slave_it->offset;
    slave_it->vel = mitMotorVelocities.at(i);
    slave_it->effort = mitMotorTorques.at(i);
    slave_it++;
  }

  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    set_state(motor_name + "/act_state/pos", act_state_map_.getPos(motor_name));
    set_state(motor_name + "/act_state/vel", act_state_map_.getVel(motor_name));
    set_state(motor_name + "/act_state/eff", act_state_map_.getEff(motor_name));
    set_state(motor_name + "/act_extra/halted", act_extra_map_.getHalted(motor_name));
    set_state(motor_name + "/act_extra/need_calibration", act_extra_map_.getNeedCalibration(motor_name));
    set_state(motor_name + "/act_extra/offset", act_extra_map_.getOffset(motor_name));
    set_state(motor_name + "/act_extra/calibrated", act_extra_map_.getCalibrated(motor_name));
    set_state(motor_name + "/act_extra/calibration_reading", act_extra_map_.getCalibrationReading(motor_name));
  }

  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    set_state(motor_name + "/act_state/pos", act_state_map_.getPos(motor_name));
    set_state(motor_name + "/act_state/vel", act_state_map_.getVel(motor_name));
    set_state(motor_name + "/act_state/eff", act_state_map_.getEff(motor_name));
    set_state(motor_name + "/act_extra/halted", act_extra_map_.getHalted(motor_name));
    set_state(motor_name + "/act_extra/need_calibration", act_extra_map_.getNeedCalibration(motor_name));
    set_state(motor_name + "/act_extra/offset", act_extra_map_.getOffset(motor_name));
    set_state(motor_name + "/act_extra/calibrated", act_extra_map_.getCalibrated(motor_name));
    set_state(motor_name + "/act_extra/calibration_reading", act_extra_map_.getCalibrationReading(motor_name));
  }

  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& actuator_info : transmission_info.actuators) {
      actuator_transmission_interfaces_.at(actuator_info.name).state[0] = get_state(actuator_info.name + "/act_state/pos");
      actuator_transmission_interfaces_.at(actuator_info.name).state[1] = get_state(actuator_info.name + "/act_state/vel");
      actuator_transmission_interfaces_.at(actuator_info.name).state[2] = get_state(actuator_info.name + "/act_state/eff");
    }
  }
  
  // actuator: state -> transmission
  for(auto& actuator_transmission_interface : actuator_transmission_interfaces_) {
    actuator_transmission_interface.second.transmissionPassthrough = actuator_transmission_interface.second.state;
  }
  // transmission: actuator -> joint
  for(auto& transmission : transmissions_) {
    transmission->actuator_to_joint();
  }
  // joint: transmission -> state
  for(auto& joint_transimission_interface : joint_transmission_interfaces_) {
    joint_transimission_interface.second.state = joint_transimission_interface.second.transmissionPassthrough;
  }

  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& joint_info : transmission_info.joints) {
      set_state(joint_info.name + "/position", joint_transmission_interfaces_.at(joint_info.name).state[0]);
      set_state(joint_info.name + "/velocity", joint_transmission_interfaces_.at(joint_info.name).state[1]);
      set_state(joint_info.name + "/effort", joint_transmission_interfaces_.at(joint_info.name).state[2]);
    }
  }
  
  for (auto& joint : info_.joints) {
    set_command(joint.name + "/effort", 0.);  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  }
  // Imus
  const auto imuOrientations = rmStandardSlaveManager_->getImuOrientations();
  const auto imuAngularVelocities = rmStandardSlaveManager_->getImuAngularVelocities();
  const auto imuLinearAccelerations = rmStandardSlaveManager_->getImuLinearAccelerations();
  const auto readings = rmStandardSlaveManager_->getReadings<rm2_ecat::standard::Reading>();
  size_t i = 0;
  for (auto& imu : imu_data_list_) {
    imu.orientation[0] = imuOrientations.at(i * 4 + 0);
    imu.orientation[1] = imuOrientations.at(i * 4 + 1);
    imu.orientation[2] = imuOrientations.at(i * 4 + 2);
    imu.orientation[3] = imuOrientations.at(i * 4 + 3);
    imu.angularVel[0] = imuAngularVelocities.at(i * 3 + 0);
    imu.angularVel[1] = imuAngularVelocities.at(i * 3 + 1);
    imu.angularVel[2] = imuAngularVelocities.at(i * 3 + 2);
    imu.linearAccel[0] = imuLinearAccelerations.at(i * 3 + 0);
    imu.linearAccel[1] = imuLinearAccelerations.at(i * 3 + 1);
    imu.linearAccel[2] = imuLinearAccelerations.at(i * 3 + 2);
    imu.stamp = createRosTime(readings.at(0).getStamp()).seconds();  // TODO: correct readings index
    ++i;
  }

  for (auto imu_name : rmStandardSlaveManager_->getImuNames()) {
    set_state(imu_name + "/orientation/x", imu_sensor_map_.getOrientation(imu_name).at(0));
    set_state(imu_name + "/orientation/y", imu_sensor_map_.getOrientation(imu_name).at(1));
    set_state(imu_name + "/orientation/z", imu_sensor_map_.getOrientation(imu_name).at(2));
    set_state(imu_name + "/orientation/w", imu_sensor_map_.getOrientation(imu_name).at(3));
    set_state(imu_name + "/angular_vel/x", imu_sensor_map_.getAngularVel(imu_name).at(0));
    set_state(imu_name + "/angular_vel/y", imu_sensor_map_.getAngularVel(imu_name).at(1));
    set_state(imu_name + "/angular_vel/z", imu_sensor_map_.getAngularVel(imu_name).at(2));
    set_state(imu_name + "/linear_accel/x", imu_sensor_map_.getLinearAccel(imu_name).at(0));
    set_state(imu_name + "/linear_accel/y", imu_sensor_map_.getLinearAccel(imu_name).at(1));
    set_state(imu_name + "/linear_accel/z", imu_sensor_map_.getLinearAccel(imu_name).at(2));
  }

  // Digital Inputs
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputs().size();
  auto digitalInputs = rmStandardSlaveManager_->getDigitalInputs();
  auto input_it = digital_input_list_.begin();
  for (i = 0; i < num_inputs; ++i) {
    *input_it = digitalInputs.at(i);
    input_it++;
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputs().size();
  digitalInputs = rmMitSlaveManager_->getDigitalInputs();
  for (i = 0; i < num_inputs; ++i) {
    *input_it = digitalInputs.at(i);
    input_it++;
  }

  for (auto digital_input_name : rmStandardSlaveManager_->getDigitalInputNames()) {
    set_state(digital_input_name + "/type", gpio_state_map_.getType(digital_input_name));
    set_state(digital_input_name + "/value", gpio_state_map_.getValue(digital_input_name));
  }
  for (auto digital_input_name : rmMitSlaveManager_->getDigitalInputNames()) {
    set_state(digital_input_name + "/type", gpio_state_map_.getType(digital_input_name));
    set_state(digital_input_name + "/value", gpio_state_map_.getValue(digital_input_name));
  }
  for (auto digital_output_name : rmStandardSlaveManager_->getDigitalOutputNames()) {
    set_state(digital_output_name + "/type", gpio_state_map_.getType(digital_output_name));
    set_state(digital_output_name + "/value", gpio_state_map_.getValue(digital_output_name));
  }
  for (auto digital_output_name : rmStandardSlaveManager_->getDigitalOutputNames()) {
    set_state(digital_output_name + "/type", gpio_state_map_.getType(digital_output_name));
    set_state(digital_output_name + "/value", gpio_state_map_.getValue(digital_output_name));
  }  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmEcatHardwareInterface::write(const rclcpp::Time&  /*time*/, const rclcpp::Duration& period) {
  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    act_extra_map_.setCalibrated(motor_name, get_command<bool>(motor_name + "/act_extra/calibrated"));
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    act_extra_map_.setCalibrated(motor_name, get_command<bool>(motor_name + "/act_extra/calibrated"));
  }

  // Propagate without joint limits
  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& joint_info : transmission_info.joints) {
      joint_transmission_interfaces_.at(joint_info.name).command[2] = get_command(joint_info.name + "/" +hardware_interface::HW_IF_EFFORT);
    }
  }

  // joint: command -> transmission
  for(auto& joint_transmission_interface : joint_transmission_interfaces_) {
    joint_transmission_interface.second.transmissionPassthrough = joint_transmission_interface.second.command;
  }
  // transmission: joint -> actuator
  for(auto& transmission : transmissions_) {
    transmission->joint_to_actuator();
  }
  // actuator: transmission -> command
  for(auto& actuator_transimission_interface : actuator_transmission_interfaces_) {
    actuator_transimission_interface.second.command = actuator_transimission_interface.second.transmissionPassthrough;  
  }

  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& actuator_info : transmission_info.actuators) {
      act_state_map_.setCommand(actuator_info.name, actuator_transmission_interfaces_.at(actuator_info.name).command[2]);
    }
  }

  // Save command before enforceLimits
  for (auto& act_data : act_data_list_) {
    act_data.commandUnlimited = act_data.command;
  }

  // enforceLimits will limit cmd_effort into suitable value
  if (!enforceLimit(period)) {
    MELO_ERROR_STREAM("Error occurred when enforce joint limits");
    return hardware_interface::return_type::ERROR;
  }
  // Propagate with joint limits
  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& joint_info : transmission_info.joints) {
      joint_transmission_interfaces_.at(joint_info.name).command[2] = get_command(joint_info.name + "/" +hardware_interface::HW_IF_EFFORT);
    }
  }

  // joint: command -> transmission
  for(auto& joint_transmission_interface : joint_transmission_interfaces_) {
    joint_transmission_interface.second.transmissionPassthrough = joint_transmission_interface.second.command;
  }
  // transmission: joint -> actuator
  for(auto& transmission : transmissions_) {
    transmission->joint_to_actuator();
  }
  // actuator: transmission -> command
  for(auto& actuator_transimission_interface : actuator_transmission_interfaces_) {
    actuator_transimission_interface.second.command = actuator_transimission_interface.second.transmissionPassthrough;  
  }

  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& actuator_info : transmission_info.actuators) {
      act_state_map_.setCommand(actuator_info.name, actuator_transmission_interfaces_.at(actuator_info.name).command[2]);
    }
  }

  // Restore the command for the calibrating joint
  for (auto& act_data : act_data_list_) {
    if (act_data.needCalibration && !act_data.calibrated) {
      act_data.command = act_data.commandUnlimited;
    }
  }

  // Set command to motor
  std::vector<double> rm_slave_commands;
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto slave_it = act_data_list_.begin();
  for (size_t i = 0; i < num_motors; ++i) {
    rm_slave_commands.push_back(slave_it->command);
    slave_it++;
  }
  rmStandardSlaveManager_->stageMotorCommands(rm_slave_commands);

  std::vector<rm2_ecat::mit::target> mit_slave_commands;
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    mit::target target{0., 0., slave_it->command, 0., 0.};
    mit_slave_commands.push_back(target);
    slave_it++;
  }
  rmMitSlaveManager_->stageMotorCommands(mit_slave_commands);

  // Digital outputs
  std::vector<bool> rm_digital_outputs;
  rm2_msgs::msg::GpioData rm_gpio_datas;
  auto rm_output_names = rmStandardSlaveManager_->getDigitalOutputNames();
  size_t rm_num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  auto output_it = digital_output_list_.begin();
  for (size_t i = 0; i < rm_num_outputs; ++i) {
    rm_digital_outputs.push_back(*output_it);
    rm_gpio_datas.gpio_name.emplace_back(rm_output_names.at(i));
    rm_gpio_datas.gpio_state.push_back(*output_it);
    rm_gpio_datas.gpio_type.emplace_back("output");
    output_it++;
  }
  rm_gpio_datas.header.stamp = node_->now();
  if (rmGpioOutputsPublisher_) {
    rmGpioOutputsPublisher_->publish(rm_gpio_datas);
  }
  rmGpioOutputsMsgUpdated_ = true;
  rmStandardSlaveManager_->stageDigitalOutputs(rm_digital_outputs);

  std::vector<bool> mit_digital_outputs;
  rm2_msgs::msg::GpioData mit_gpio_datas;
  auto mit_output_names = rmMitSlaveManager_->getDigitalOutputNames();
  size_t mit_num_outputs = rmMitSlaveManager_->getDigitalOutputNames().size();
  for (size_t i = 0; i < mit_num_outputs; ++i) {
    mit_digital_outputs.push_back(*output_it);
    mit_gpio_datas.gpio_name.emplace_back(mit_output_names.at(i));
    mit_gpio_datas.gpio_state.push_back(*output_it);
    mit_gpio_datas.gpio_type.emplace_back("output");
    output_it++;
  }
  mit_gpio_datas.header.stamp = node_->now();
  if (mitGpioOutputsPublisher_) {
    mitGpioOutputsPublisher_->publish(mit_gpio_datas);
  }
  mitGpioOutputsMsgUpdated_ = true;
  rmMitSlaveManager_->stageDigitalOutputs(mit_digital_outputs);

  // bus monitoring
  if (busDiagDecimationCount_ > 100) {
    rm2_msgs::msg::BusState busStatesMsg_;
    for (const auto& bus : busNames_) {
      if (!busIsOk_.at(bus)) {
        if (busManager_->onActivate(bus)) {
          busIsOk_.at(bus) = true;
        }
      }
      if (!busManager_->busMonitoring(bus) && busIsOk_.at(bus)) {
        MELO_ERROR("Bus is not ok");
        busManager_->onDeactivate(bus);
        busIsOk_.at(bus) = false;
      }
      busStatesMsg_.name.push_back(bus);
      busStatesMsg_.is_online.push_back(busIsOk_.at(bus));
      busStatesMsg_.stamp = node_->now();
    }
    if (busStatesPublisher_) {
      busStatesPublisher_->publish(busStatesMsg_);
    }
    busStatesMsgUpdated_ = true;
    busDiagDecimationCount_ = 0;
  }
  busDiagDecimationCount_++;
  return hardware_interface::return_type::OK;
}

}  // namespace rm2_ecat

PLUGINLIB_EXPORT_CLASS(rm2_ecat::RmEcatHardwareInterface, hardware_interface::SystemInterface)