cmake_minimum_required(VERSION 3.8)
project(ecat_manager)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Werror -Wextra)

set(BASE_DEPENDENCIES
    soem_interface_rsl
    any_worker
    signal_handler
)

find_package(ament_cmake REQUIRED)

foreach(dep IN LISTS BASE_DEPENDENCIES)
    find_package(${dep} REQUIRED)
endforeach()

find_package(yaml-cpp REQUIRED)

set(OPTIONAL_DEPENDENCIES)
set(OPTIONAL_DEFINITIONS)

find_package(cleardrive_foc QUIET)
find_package(cleardrive_imu QUIET)
find_package(rm2_ecat_mit QUIET)
find_package(rm2_ecat_standard_slave QUIET)

if(cleardrive_foc_FOUND)
    list(APPEND OPTIONAL_DEFINITIONS -D_CLEAR_FOC_FOUND_)
    list(APPEND OPTIONAL_DEPENDENCIES cleardrive_foc)
endif()

if(cleardrive_imu_FOUND)
    list(APPEND OPTIONAL_DEFINITIONS -D_CLEAR_IMU_FOUND_)
    list(APPEND OPTIONAL_DEPENDENCIES cleardrive_imu)
endif()

if(rm2_ecat_mit_FOUND)
    list(APPEND OPTIONAL_DEFINITIONS -D_MIT_FOUND_)
    list(APPEND OPTIONAL_DEPENDENCIES rm2_ecat_mit)
endif()

if(rm2_ecat_standard_slave_FOUND)
    list(APPEND OPTIONAL_DEFINITIONS -D_RM_FOUND_)
    list(APPEND OPTIONAL_DEPENDENCIES rm2_ecat_standard_slave)
endif()

foreach(sdk IN LISTS OPTIONAL_DEPENDENCIES)
    message("Found EtherCAT slave: ${sdk}")
endforeach()

set(ALL_DEPENDENCIES
    ${BASE_DEPENDENCIES}
    ${OPTIONAL_DEPENDENCIES}
)

if(OPTIONAL_DEFINITIONS)
    add_compile_options(${OPTIONAL_DEFINITIONS})
endif()

add_definitions(-DROS2_BUILD)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
    src/EcatBusManager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    ${ALL_DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME}
    yaml-cpp
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${ALL_DEPENDENCIES})

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