cmake_minimum_required(VERSION 3.8)
project(rm2_gimbal_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pid_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rm2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pal_statistics REQUIRED)

set(DEPENDENCIES
    rclcpp
    rcl_interfaces
    pid_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    control_toolbox
    tf2_eigen
    visualization_msgs
    angles
    tf2_geometry_msgs
    tf2_ros
    realtime_tools
    rm2_msgs
    control_msgs
    geometry_msgs
    pal_statistics
)

add_library(${PROJECT_NAME} SHARED
  src/gimbal_base.cpp
  src/joint_controller.cpp
  src/pid_debug_publisher.cpp
  src/bullet_solver.cpp
  src/gimbal_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${DEPENDENCIES}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
    DESTINATION include
)

pluginlib_export_plugin_description_file(controller_interface rm2_gimbal_controllers_plugins.xml)

ament_package()