cmake_minimum_required(VERSION 2.8.3)
project(pmb2_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config gazebo Media meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()