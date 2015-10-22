cmake_minimum_required(VERSION 2.8.3)
project(pmb2_5_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
launch config
DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/pmb2_bringup
)

