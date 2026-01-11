cmake_minimum_required(VERSION 3.8)
project(rm2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(msg_detection_files
        "msg/detection/PriorityArray.msg"
        "msg/detection/TargetDetection.msg"
        "msg/detection/TargetDetectionArray.msg"
        "msg/detection/TrackData.msg"
)

set(msg_referee_files
        "msg/referee/CapacityData.msg"
        "msg/referee/GameRobotStatus.msg"
        "msg/referee/GameStatus.msg"
        "msg/referee/StateCmd.msg"
        "msg/referee/EngineerUi.msg"
        "msg/referee/GameRobotHp.msg"
        "msg/referee/PowerHeatData.msg"
        "msg/referee/EventData.msg"
        "msg/referee/DartStatus.msg"
        "msg/referee/IcraBuffDebuffZoneStatus.msg"
        "msg/referee/SupplyProjectileAction.msg"
        "msg/referee/DartInfo.msg"
        "msg/referee/RobotHurt.msg"
        "msg/referee/SentryAttackingTarget.msg"
        "msg/referee/ShootData.msg"
        "msg/referee/BulletAllowance.msg"
        "msg/referee/RfidStatus.msg"
        "msg/referee/DartClientCmd.msg"
        "msg/referee/ManualToReferee.msg"
        "msg/referee/RadarData.msg"
        "msg/referee/RadarMarkData.msg"
        "msg/referee/RobotsPositionData.msg"
        "msg/referee/ClientMapSendData.msg"
        "msg/referee/ClientMapReceiveData.msg"
        "msg/referee/MapSentryData.msg"
        "msg/referee/GameRobotPosData.msg"
        "msg/referee/SentryInfo.msg"
        "msg/referee/RadarInfo.msg"
        "msg/referee/Buff.msg"
        "msg/referee/RadarToSentry.msg"
        "msg/referee/SentryCmd.msg"
        "msg/referee/VisualizeStateData.msg"
        "msg/referee/PowerManagementSampleAndStatusData.msg"
        "msg/referee/PowerManagementInitializationExceptionData.msg"
        "msg/referee/PowerManagementProcessStackOverflowData.msg"
        "msg/referee/PowerManagementSystemExceptionData.msg"
        "msg/referee/PowerManagementUnknownExceptionData.msg"
)

set(msg_files
        ${msg_detection_files}
        ${msg_referee_files}
        "msg/ActuatorState.msg"
        "msg/BalanceState.msg"
        "msg/BusState.msg"
        "msg/DbusData.msg"
        "msg/ExchangerMsg.msg"
        "msg/ChassisCmd.msg"
        "msg/ShootCmd.msg"
        "msg/ShootState.msg"
        "msg/ShootBeforehandCmd.msg"
        "msg/GimbalCmd.msg"
        "msg/GimbalDesError.msg"
        "msg/GimbalPosState.msg"
        "msg/LegCmd.msg"
        "msg/LpData.msg"
        "msg/LocalHeatState.msg"
        "msg/KalmanData.msg"
        "msg/MovingAverageData.msg"
        "msg/GpioData.msg"
        "msg/TofRadarData.msg"
        "msg/MultiDofCmd.msg"
        "msg/TagMsg.msg"
        "msg/TagMsgArray.msg"
        "msg/SentryDeviate.msg"
        "msg/CustomControllerData.msg"
        "msg/VTKeyboardMouseData.msg"
        "msg/VTReceiverControlData.msg"
)

set(srv_files
        "srv/ContinousDetectorSwitch.srv"
        "srv/StatusChange.srv"
        "srv/SetLimitVel.srv"
        "srv/CameraStatus.srv"
        "srv/EnableImuTrigger.srv"
        "srv/EnableGyro.srv"
)

set(action_files
        "action/Engineer.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        ${action_files}
        DEPENDENCIES std_msgs geometry_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
