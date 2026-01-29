//
// Created by qiayuan on 23-4-16.
//

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "rm2_ecat_ros/RmEcatHardwareInterface.h"
#include "rm2_ecat_ros/RosMsgConversions.h"

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

  updateWorker_ = std::make_shared<any_worker::Worker>("updateWorker", rmStandardSlaveManager_->getTimeStep(),
                                                        [this](auto && PH1) { return updateWorkerCb(std::forward<decltype(PH1)>(PH1)); });
  publishWorker_ = std::make_shared<any_worker::Worker>("PublishWorker", 10 * rmStandardSlaveManager_->getTimeStep(),
                                                        [this](auto && PH1) { return publishWorkerCb(std::forward<decltype(PH1)>(PH1)); });
  signal_handler::SignalHandler::bindAll(&RmEcatHardwareInterface::handleSignal, this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RmEcatHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

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

  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmEcatHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (manualShutdown_) {
    busManager_->startupCommunication();
    manualShutdown_ = false;
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
  updateWorker_->start(48);
  publishWorker_->start(20);  

  return hardware_interface::CallbackReturn::SUCCESS;
}
    
CallbackReturn RmEcatHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SystemInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  manualShutdown_ = true;
  updateWorker_->stop(true);
  publishWorker_->stop(true);
  rmStandardSlaveManager_->shutdown();
  rmMitSlaveManager_->shutdown();
  busStatesPublisher_->shutdown();
  if (rmGpioOutputsPublisher_) {
    rmGpioOutputsPublisher_->shutdown();
  }
  if (mitGpioOutputsPublisher_) {
    mitGpioOutputsPublisher_->shutdown();
  }
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
  manualShutdown_ = true;  
  updateWorker_->stop(true);
  publishWorker_->stop(true);
  rmStandardSlaveManager_->shutdown();
  rmMitSlaveManager_->shutdown();
  busStatesPublisher_->shutdown();
  if (rmGpioOutputsPublisher_) {
    rmGpioOutputsPublisher_->shutdown();
  }
  if (mitGpioOutputsPublisher_) {
    mitGpioOutputsPublisher_->shutdown();
  }
  rclcpp::shutdown();
}

bool RmEcatHardwareInterface::loadUrdf(std::shared_ptr<rclcpp::Node> node) {
  if (urdf_model_ == nullptr) {
    urdf_model_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  if (node_->has_parameter("robot_description")) {
    urdf_string_ = node->get_parameter("robot_description").as_string();
  } else {
    urdf_string_ = node->declare_parameter("robot_description", "");
  }
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

void RmEcatHardwareInterface::setupActuators() {
  // rm standard slave
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto motorNames = rmStandardSlaveManager_->getMotorNames();
  auto motorNeedCalibrations = rmStandardSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    set_state(motorNames.at(i) + "/act_state/pos", 0.);
    set_state(motorNames.at(i) + "/act_state/vel", 0.);
    set_state(motorNames.at(i) + "/act_state/eff", 0.);
    set_command(motorNames.at(i) + "/act_extra/offset", 0.);    
    set_command(motorNames.at(i) + "/act_extra/halted", false);
    set_command(motorNames.at(i) + "/act_extra/need_calibration", static_cast<bool>(motorNeedCalibrations.at(i)));
    set_command(motorNames.at(i) + "/act_extra/calibrated", false);
    set_command(motorNames.at(i) + "/act_extra/calibration_reading", false);
    set_command(motorNames.at(i) + "/act_command/command", 0.);
    set_command(motorNames.at(i) + "/act_command/command_unlimited", 0.);
  }

  // mit slave
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  motorNames = rmMitSlaveManager_->getMotorNames();
  motorNeedCalibrations = rmMitSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    set_state(motorNames.at(i) + "/act_state/pos", 0.);
    set_state(motorNames.at(i) + "/act_state/vel", 0.);
    set_state(motorNames.at(i) + "/act_state/eff", 0.);
    set_command(motorNames.at(i) + "/act_extra/offset", 0.);    
    set_command(motorNames.at(i) + "/act_extra/halted", false);
    set_command(motorNames.at(i) + "/act_extra/need_calibration", static_cast<bool>(motorNeedCalibrations.at(i)));
    set_command(motorNames.at(i) + "/act_extra/calibrated", false);
    set_command(motorNames.at(i) + "/act_extra/calibration_reading", false);
    set_command(motorNames.at(i) + "/act_command/command", 0.);
    set_command(motorNames.at(i) + "/act_command/command_unlimited", 0.);
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
      set_command(actuator_info.name + "/act_extra/offset", actuator_info.offset);
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
    set_state(imuNames.at(i) + "/orientation/x", 0.);
    set_state(imuNames.at(i) + "/orientation/y", 0.);
    set_state(imuNames.at(i) + "/orientation/z", 0.);
    set_state(imuNames.at(i) + "/orientation/w", 0.);
    set_state(imuNames.at(i) + "/angular_vel/x", 0.);
    set_state(imuNames.at(i) + "/angular_vel/y", 0.);
    set_state(imuNames.at(i) + "/angular_vel/z", 0.);
    set_state(imuNames.at(i) + "/linear_accel/x", 0.);
    set_state(imuNames.at(i) + "/linear_accel/y", 0.);
    set_state(imuNames.at(i) + "/linear_accel/z", 0.);
    set_state(imuNames.at(i) + "/stamp", 0.);
  }
}

void RmEcatHardwareInterface::setupGpios() {
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputNames().size();
  auto digitalInputNames = rmStandardSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    set_state(digitalInputNames.at(i) + "/type", static_cast<bool>(GpioType::INPUT));
    set_state(digitalInputNames.at(i) + "/value", false);
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputNames().size();
  digitalInputNames = rmMitSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    set_state(digitalInputNames.at(i) + "/type", static_cast<bool>(GpioType::INPUT));
    set_state(digitalInputNames.at(i) + "/value", false);
  }

  size_t num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  auto digitalOutputNames = rmStandardSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    set_command(digitalOutputNames.at(i) + "/type", static_cast<bool>(GpioType::OUTPUT));
    set_command(digitalOutputNames.at(i) + "/value", false);
  }
  num_outputs = rmMitSlaveManager_->getDigitalOutputNames().size();
  digitalOutputNames = rmMitSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    set_command(digitalOutputNames.at(i) + "/type", static_cast<bool>(GpioType::OUTPUT));
    set_command(digitalOutputNames.at(i) + "/value", false);
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
      ecat_unlisted_state_interfaces.emplace_back(motor_name, act_state_interface);
    }
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    for (auto it : act_state_interface_type) {
      hardware_interface::InterfaceInfo act_state_interface;
      act_state_interface.name = "act_state/" + it.first;
      act_state_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.emplace_back(motor_name, act_state_interface);
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
                                                                        {"linear_acceleration_covariance[3]", "double"},
                                                                        {"stamp", "double"}}; 

  for (auto imu_name : rmStandardSlaveManager_->getImuNames()) {
    for (auto it : imu_sensor_interface_type) {
      hardware_interface::InterfaceInfo imu_sensor_interface;
      imu_sensor_interface.name = it.first;
      imu_sensor_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.emplace_back(imu_name, imu_sensor_interface);
    }
  }

  std::unordered_map<std::string, std::string> gpio_interface_type{{"type", "double"}, 
                                                                   {"value", "bool"}}; 
  
  for (auto digital_input_name : rmStandardSlaveManager_->getDigitalInputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_input_interface;
      gpio_input_interface.name = it.first;
      gpio_input_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.emplace_back(digital_input_name, gpio_input_interface);
    }
  }
  for (auto digital_input_name : rmMitSlaveManager_->getDigitalInputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_input_interface;
      gpio_input_interface.name = it.first;
      gpio_input_interface.data_type = it.second;
      ecat_unlisted_state_interfaces.emplace_back(digital_input_name, gpio_input_interface);
    }
  }

  return ecat_unlisted_state_interfaces;
}

std::vector<hardware_interface::InterfaceDescription> RmEcatHardwareInterface::export_unlisted_command_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> ecat_unlisted_command_interfaces;

  std::unordered_map<std::string, std::string> act_command_interface_type{{"command", "double"}, 
                                                                          {"command_unlimited", "double"}}; 
  std::unordered_map<std::string, std::string> act_extra_interface_type{{"offset", "double"},
                                                                        {"halted", "bool"}, 
                                                                        {"need_calibration", "bool"},
                                                                        {"calibrated", "bool"}, 
                                                                        {"calibration_reading", "bool"}}; 
  for (auto motor_name : rmStandardSlaveManager_->getMotorNames()) {
    for (auto it : act_command_interface_type) {
      hardware_interface::InterfaceInfo act_command_interface;
      act_command_interface.name = "act_command/" + it.first;
      act_command_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(motor_name, act_command_interface);
    }
    for (auto it : act_extra_interface_type) {
      hardware_interface::InterfaceInfo act_extra_interface;
      act_extra_interface.name = "act_extra/" + it.first;
      act_extra_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(motor_name, act_extra_interface);
    }    
  }
  for (auto motor_name : rmMitSlaveManager_->getMotorNames()) {
    for (auto it : act_command_interface_type) {
      hardware_interface::InterfaceInfo act_command_interface;
      act_command_interface.name = "act_command/" + it.first;
      act_command_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(motor_name, act_command_interface);
    }
    for (auto it : act_extra_interface_type) {
      hardware_interface::InterfaceInfo act_extra_interface;
      act_extra_interface.name = "act_extra/" + it.first;
      act_extra_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(motor_name, act_extra_interface);
    }    
  }

  std::unordered_map<std::string, std::string> gpio_interface_type{{"type", "double"}, 
                                                                   {"value", "bool"}}; 

  for (auto digital_output_name : rmStandardSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(digital_output_name, gpio_output_interface);
    }
  }
  for (auto digital_output_name : rmMitSlaveManager_->getDigitalOutputNames()) {
    for (auto it : gpio_interface_type) {
      hardware_interface::InterfaceInfo gpio_output_interface;
      gpio_output_interface.name = it.first;
      gpio_output_interface.data_type = it.second;
      ecat_unlisted_command_interfaces.emplace_back(digital_output_name, gpio_output_interface);
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
  const auto rmMotorNames = rmStandardSlaveManager_->getMotorNames();

  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    set_command(rmMotorNames.at(i) + "/act_extra/halted", !rmMotorIsOnlines.at(i));
    set_state(rmMotorNames.at(i) + "/act_state/pos", rmMotorPositions.at(i) + get_command(rmMotorNames.at(i) + "/act_extra/offset"));
    set_state(rmMotorNames.at(i) + "/act_state/vel", rmMotorVelocities.at(i));
    set_state(rmMotorNames.at(i) + "/act_state/eff", rmMotorTorques.at(i));
  }

  rmMitSlaveManager_->checkMotorsIsonline();
  const auto& mitMotorIsOnlines = rmMitSlaveManager_->getMotorIsOnlines();
  const auto& mitMotorPositions = rmMitSlaveManager_->getMotorPositions();
  const auto& mitMotorVelocities = rmMitSlaveManager_->getMotorVelocities();
  const auto& mitMotorTorques = rmMitSlaveManager_->getMotorTorque();
  const auto mitMotorNames = rmMitSlaveManager_->getMotorNames();
  
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    set_command(mitMotorNames.at(i) + "/act_extra/halted", !mitMotorIsOnlines.at(i));
    set_state(mitMotorNames.at(i) + "/act_state/pos", mitMotorPositions.at(i) + get_command(mitMotorNames.at(i) + "/act_extra/offset"));
    set_state(mitMotorNames.at(i) + "/act_state/vel", mitMotorVelocities.at(i));
    set_state(mitMotorNames.at(i) + "/act_state/eff", mitMotorTorques.at(i));
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
      set_state(joint_info.name + "/" + hardware_interface::HW_IF_POSITION, joint_transmission_interfaces_.at(joint_info.name).state[0]);
      set_state(joint_info.name + "/" + hardware_interface::HW_IF_VELOCITY, joint_transmission_interfaces_.at(joint_info.name).state[1]);
      set_state(joint_info.name + "/" + hardware_interface::HW_IF_EFFORT, joint_transmission_interfaces_.at(joint_info.name).state[2]);
    }
  }
  
  for (auto& joint : info_.joints) {
    set_command(joint.name + "/" + hardware_interface::HW_IF_EFFORT, 0.);  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  }
  // Imus
  const auto imuOrientations = rmStandardSlaveManager_->getImuOrientations();
  const auto imuAngularVelocities = rmStandardSlaveManager_->getImuAngularVelocities();
  const auto imuLinearAccelerations = rmStandardSlaveManager_->getImuLinearAccelerations();
  const auto imuNames = rmStandardSlaveManager_->getImuNames();
  const auto readings = rmStandardSlaveManager_->getReadings<rm2_ecat::standard::Reading>();

  size_t num_imus = rmStandardSlaveManager_->getImuNames().size();
  for (size_t i = 0; i < num_imus; ++i) {
    set_state(imuNames.at(i) + "/orientation/x", imuOrientations.at(i * 4 + 0));
    set_state(imuNames.at(i) + "/orientation/y", imuOrientations.at(i * 4 + 1));
    set_state(imuNames.at(i) + "/orientation/z", imuOrientations.at(i * 4 + 2));
    set_state(imuNames.at(i) + "/orientation/w", imuOrientations.at(i * 4 + 3));
    set_state(imuNames.at(i) + "/angular_vel/x", imuAngularVelocities.at(i * 3 + 0));
    set_state(imuNames.at(i) + "/angular_vel/y", imuAngularVelocities.at(i * 3 + 1));
    set_state(imuNames.at(i) + "/angular_vel/z", imuAngularVelocities.at(i * 3 + 2));
    set_state(imuNames.at(i) + "/linear_accel/x", imuLinearAccelerations.at(i * 3 + 0));
    set_state(imuNames.at(i) + "/linear_accel/y", imuLinearAccelerations.at(i * 3 + 1));
    set_state(imuNames.at(i) + "/linear_accel/z", imuLinearAccelerations.at(i * 3 + 2));
    set_state(imuNames.at(i) + "/stamp", createRosTime(readings.at(0).getStamp()).seconds());
  }

  // Digital Inputs
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputs().size();
  auto digitalInputs = rmStandardSlaveManager_->getDigitalInputs();
  auto digitalInputNames = rmStandardSlaveManager_->getDigitalInputNames();

  for (size_t i = 0; i < num_inputs; ++i) {
    set_state(digitalInputNames.at(i) + "/value", static_cast<bool>(digitalInputs.at(i)));
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputs().size();
  digitalInputs = rmMitSlaveManager_->getDigitalInputs();
  digitalInputNames = rmMitSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    set_state(digitalInputNames.at(i) + "/value", static_cast<bool>(digitalInputs.at(i)));
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmEcatHardwareInterface::write(const rclcpp::Time&  /*time*/, const rclcpp::Duration& period) {
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
  // Save command before enforceLimits
  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& actuator_info : transmission_info.actuators) {
      set_command(actuator_info.name + "/act_command/command", actuator_transmission_interfaces_.at(actuator_info.name).command[2]);
      set_command(actuator_info.name + "/act_command/command_unlimited", actuator_transmission_interfaces_.at(actuator_info.name).command[2]);
    }
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
  // Restore the command for the calibrating joint
  for (const auto& transmission_info : info_.transmissions) {
    for (const auto& actuator_info : transmission_info.actuators) {
      set_command(actuator_info.name + "/act_command/command", actuator_transmission_interfaces_.at(actuator_info.name).command[2]);
      if (get_command<bool>(actuator_info.name + "/act_extra/need_calibration") && !get_command<bool>(actuator_info.name + "act_extra/calibrated")) {
        set_command(actuator_info.name + "/act_command/command", get_command(actuator_info.name + "/act_command/command_unlimited"));
      }
    }
  }

  // Set command to motor
  std::vector<double> rm_slave_commands;
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  const auto rmMotorNames = rmStandardSlaveManager_->getMotorNames();

  for (size_t i = 0; i < num_motors; ++i) {
    rm_slave_commands.push_back(get_command(rmMotorNames.at(i) + "/act_command/command"));
  }
  rmStandardSlaveManager_->stageMotorCommands(rm_slave_commands);

  std::vector<rm2_ecat::mit::target> mit_slave_commands;
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  const auto mitMotorNames = rmMitSlaveManager_->getMotorNames();  
  for (size_t i = 0; i < num_motors; ++i) {
    mit::target target{0., 0., get_command(mitMotorNames.at(i) + "/act_command/command"), 0., 0.};
    mit_slave_commands.push_back(target);
  }
  rmMitSlaveManager_->stageMotorCommands(mit_slave_commands);

  // Digital outputs
  std::vector<bool> rm_digital_outputs;
  rm2_msgs::msg::GpioData rm_gpio_datas;
  auto rm_output_names = rmStandardSlaveManager_->getDigitalOutputNames();
  size_t rm_num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  for (size_t i = 0; i < rm_num_outputs; ++i) {
    rm_digital_outputs.push_back(get_command<bool>(rm_output_names.at(i) + "/value"));
    rm_gpio_datas.gpio_name.emplace_back(rm_output_names.at(i));
    rm_gpio_datas.gpio_state.push_back(get_command<bool>(rm_output_names.at(i) + "/value"));
    rm_gpio_datas.gpio_type.emplace_back("output");
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
    mit_digital_outputs.push_back(get_command<bool>(mit_output_names.at(i) + "/value"));
    mit_gpio_datas.gpio_name.emplace_back(mit_output_names.at(i));
    mit_gpio_datas.gpio_state.push_back(get_command<bool>(mit_output_names.at(i) + "/value"));
    mit_gpio_datas.gpio_type.emplace_back("output");
  }
  mit_gpio_datas.header.stamp = node_->now();
  if (mitGpioOutputsPublisher_) {
    mitGpioOutputsPublisher_->publish(mit_gpio_datas);
  }
  mitGpioOutputsMsgUpdated_ = true;
  rmMitSlaveManager_->stageDigitalOutputs(mit_digital_outputs);

  // bus monitoring
  if (!manualShutdown_) {
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
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rm2_ecat

PLUGINLIB_EXPORT_CLASS(rm2_ecat::RmEcatHardwareInterface, hardware_interface::SystemInterface)