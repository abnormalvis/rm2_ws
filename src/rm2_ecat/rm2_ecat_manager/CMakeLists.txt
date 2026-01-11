cmake_minimum_required(VERSION 3.8)
project(rm2_ecat_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

set(PACKAGE_DEPENDENCIES
    rm2_ecat_standard_slave
    rm2_ecat_mit
    ecat_manager
)

find_package(ament_cmake REQUIRED)

find_package(yaml-cpp REQUIRED)

foreach(dep IN LISTS PACKAGE_DEPENDENCIES)
    find_package(${dep} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(${PROJECT_NAME}
    src/RmEcatStandardSlaveManager.cpp
    src/RmEcatMitManager.cpp
)

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