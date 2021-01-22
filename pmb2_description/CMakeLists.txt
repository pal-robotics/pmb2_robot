cmake_minimum_required(VERSION 3.5)
project(pmb2_description)

find_package(ament_cmake_auto REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

ament_auto_find_build_dependencies()


#############
## Testing ##
#############

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  #  find_package(rostest REQUIRED)
  #  foreach(laser_model false hokuyo sick-551 sick-561 sick-571)
  #      foreach(rgbd_sensors false true)
  #        add_rostest(test/test_pmb2.test ARGS laser_model:=${laser_model} rgbd_sensors:=${rgbd_sensors})
  #      endforeach(rgbd_sensors)
  #  endforeach(laser_model)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config gazebo Media meshes robots urdf launch)
