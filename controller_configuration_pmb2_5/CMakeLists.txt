cmake_minimum_required(VERSION 2.8.3)
project(controller_configuration_pmb2_5)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch)
    install(DIRECTORY config
        DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/pmb2_controller_configuration)
endforeach()
