cmake_minimum_required(VERSION 3.8)
project(rm2_dynamic_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)

add_executable(rm2_dynamic_config_gui
  include/rm2_dynamic_config/main_window.hpp
  src/main.cpp
  src/main_window.cpp
)

target_include_directories(rm2_dynamic_config_gui PRIVATE
  include
)

target_compile_definitions(rm2_dynamic_config_gui PRIVATE
  RM2_DYNAMIC_CONFIG_DEFAULT_SCHEMA="${CMAKE_CURRENT_SOURCE_DIR}/config/gimbal_controller.schema.yaml"
)

ament_target_dependencies(rm2_dynamic_config_gui
  rclcpp
  rcl_interfaces
)

target_link_libraries(rm2_dynamic_config_gui
  Qt5::Widgets
  yaml-cpp
)

install(TARGETS rm2_dynamic_config_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
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
