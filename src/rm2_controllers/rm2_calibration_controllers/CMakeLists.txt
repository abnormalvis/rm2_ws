cmake_minimum_required(VERSION 3.8)
project(rm2_calibration_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm2_common REQUIRED)
find_package(pid_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_toolbox REQUIRED)

set(DEPENDENCIES
    rclcpp
    rm2_common
    pid_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    control_toolbox
)

add_library(${PROJECT_NAME} SHARED
    src/calibration_base.cpp
    src/mechanical_calibration_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

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

install(DIRECTORY include/
    DESTINATION include
)

pluginlib_export_plugin_description_file(controller_interface rm2_calibration_controllers.xml)

ament_package()
