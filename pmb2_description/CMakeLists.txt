cmake_minimum_required(VERSION 2.8.3)
project(pmb2_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config gazebo Media meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_pmb2_default.test)
  add_rostest(test/test_pmb2_sick551.test)
  add_rostest(test/test_pmb2_sick561.test)
  add_rostest(test/test_pmb2_sick571.test)
endif()
