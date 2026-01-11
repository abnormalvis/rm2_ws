cmake_minimum_required(VERSION 3.8)
project(rm2_ecat_standard_slave)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall)
add_definitions(-DMELO_USE_COUT)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
    soem_interface_rsl
    rm2_msgs
)

find_package(ament_cmake REQUIRED)

foreach(dep IN LISTS PACKAGE_DEPENDENCIES)
    find_package(${dep} REQUIRED)
endforeach()

find_package(yaml-cpp REQUIRED)

###########
## Build ##
###########

file(GLOB SOURCES src/*.cpp)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    ${PACKAGE_DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME}
    yaml-cpp
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()