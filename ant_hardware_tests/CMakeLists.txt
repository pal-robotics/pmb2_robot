cmake_minimum_required(VERSION 2.8.3)
project(ant_hardware_tests)

find_package(catkin REQUIRED COMPONENTS ant_hardware)

find_package(orocos-rtt)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

orocos_component(ant_dummy_actuators_manager src/dummy_actuators_manager.cpp)
target_link_libraries(ant_dummy_actuators_manager ${catkin_LIBRARIES})

orocos_generate_package()
