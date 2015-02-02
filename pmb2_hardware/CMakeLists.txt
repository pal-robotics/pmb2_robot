cmake_minimum_required(VERSION 2.8.3)
project(ant_hardware)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    controller_manager
    pal_ros_control
    realtime_tools
    std_srvs)

# Set orocos target
set(OROCOS_TARGET_ENV $ENV{OROCOS_TARGET}) # Cache orocos target from environment
set(Xenomai_ROOT_DIR "/opt/xenomai")
find_package(Xenomai)
if(XENOMAI_FOUND)
  set(ENV{OROCOS_TARGET} xenomai)
else()
  set(ENV{OROCOS_TARGET} gnulinux)
endif()

find_package(orocos-rtt)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/ant_hardware.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

# Restore original orocos target, if any
if (OROCOS_TARGET_ENV)
  set(ENV{OROCOS_TARGET} ${OROCOS_TARGET_ENV})
endif()
