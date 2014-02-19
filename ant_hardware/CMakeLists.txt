cmake_minimum_required(VERSION 2.8.3)
project(ant_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp
    controller_manager
    pal_ros_control
    realtime_tools
    std_srvs)

find_package(orocos-rtt)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ant_hardware
  CATKIN_DEPENDS
    controller_manager
    pal_ros_control
    realtime_tools
    std_srvs
)

include_directories(include ${catkin_INCLUDE_DIRS})

orocos_component(ant_hardware src/ant_hardware.cpp)
target_link_libraries(ant_hardware ${catkin_LIBRARIES})

orocos_install_headers(include/ant_hardware/ant_hardware.h)
orocos_generate_package()

install(PROGRAMS scripts/hand_homing
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
