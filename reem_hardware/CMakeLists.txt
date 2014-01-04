cmake_minimum_required(VERSION 2.8.3)
project(reem_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp
    controller_manager
    pal_ros_control
    realtime_tools
    std_srvs)

find_package(orocos-rtt)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reem_hardware
  CATKIN_DEPENDS
    controller_manager
    pal_ros_control
    realtime_tools
    std_srvs
)

include_directories(include ${catkin_INCLUDE_DIRS})

orocos_component(reem_hardware src/reem_hardware.cpp)
target_link_libraries(reem_hardware ${catkin_LIBRARIES})

orocos_install_headers(include/reem_hardware/reem_hardware.h)
orocos_generate_package()

install(PROGRAMS scripts/hand_homing
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
