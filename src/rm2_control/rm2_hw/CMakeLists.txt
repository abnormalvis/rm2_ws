cmake_minimum_required(VERSION 3.8)
project(rm2_hw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(DEPEND_PACKAGES
        rclcpp
        rm2_common
        rm2_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits
        controller_manager
        urdf
        pluginlib
        realtime_tools 
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${DEPEND_PACKAGES})
  find_package(${Dependency} REQUIRED)
endforeach ()

file(GLOB HARDWARE_INTERFACE "include/rm2_hw/hardware_interface/*")
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/hardware_interface/*.cpp")
# add_library(${PROJECT_NAME} SHARED
#        ${HARDWARE_INTERFACE}
#        ${SOURCES}
# )

add_library(${PROJECT_NAME}_plugin SHARED
        src/hardware_interface/hardware_system.cpp
        src/hardware_interface/parse.cpp
        src/hardware_interface/socketcan.cpp
        src/hardware_interface/can_bus.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC ${DEPEND_PACKAGES})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/control_loop.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_plugin)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPEND_PACKAGES})

# INSTALL
install(DIRECTORY include/
        DESTINATION include
)
install(TARGETS ${PROJECT_NAME}_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
install(FILES rm2_hw_plugin.xml
        DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ${DEPEND_PACKAGES}
)

pluginlib_export_plugin_description_file(hardware_interface rm2_hw_plugin.xml)

ament_package()
