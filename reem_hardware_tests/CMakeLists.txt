cmake_minimum_required(VERSION 2.8.3)
project(reem_hardware_tests)

find_package(catkin REQUIRED COMPONENTS reem_hardware)

find_package(orocos-rtt)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

orocos_component(reem_dummy_actuators_manager src/dummy_actuators_manager.cpp)
target_link_libraries(reem_dummy_actuators_manager ${catkin_LIBRARIES})

orocos_generate_package()
