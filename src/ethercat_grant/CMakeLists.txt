cmake_minimum_required(VERSION 3.8)
project(ethercat_grant)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

add_executable(ethercat_grant src/ethercat_grant.cpp)
target_link_libraries(ethercat_grant cap)

install(TARGETS ethercat_grant
    DESTINATION lib/${PROJECT_NAME}
)
