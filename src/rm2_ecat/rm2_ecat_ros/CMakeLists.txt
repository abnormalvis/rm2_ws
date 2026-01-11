cmake_minimum_required(VERSION 3.8)
project(rm2_ecat_ros)

add_compile_options(-Wall -Werror)

set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rm2_ecat_manager REQUIRED)
find_package(rm2_ecat_msgs REQUIRED)
find_package(any_node REQUIRED)
find_package(controller_manager REQUIRED)
find_package(urdf REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(rm2_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

set(DEPENDENCIES
    rclcpp
    rm2_ecat_manager
    rm2_ecat_msgs
    any_node
    controller_manager
    urdf
    transmission_interface
    joint_limits
    rm2_common
    yaml-cpp
    pluginlib
    realtime_tools
)

add_definitions(-DROS2_BUILD)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_plugin SHARED
    src/RosMsgConversions.cpp
    src/RmEcatHardwareInterface.cpp
    src/RmStandardSlaveManagerRos.cpp
    src/RmMitSlaveManagerRos.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC
    ${DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME}_plugin 
    PUBLIC yaml-cpp
)

set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(${DEPENDENCIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_plugin
    EXPORT ${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
# install(FILES rm2_ecat_ros_plugin.xml
#     DESTINATION share/${PROJECT_NAME}
# )

pluginlib_export_plugin_description_file(hardware_interface rm2_ecat_ros_plugin.xml)

ament_package()