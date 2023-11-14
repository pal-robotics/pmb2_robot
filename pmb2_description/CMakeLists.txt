cmake_minimum_required(VERSION 3.5)
project(pmb2_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/00_robot_state_publisher.yaml
  )
endif()

#############
## Testing ##
#############

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  foreach(laser_model no-laser hokuyo sick-551 sick-561 sick-571)
      foreach(courier_rgbd_sensors False True)
      add_launch_test(
        test/test_description.launch.py
        TARGET "pmb2_description_${laser_model}_${courier_rgbd_sensors}"
        ARGS "laser_model:=${laser_model}" "courier_rgbd_sensors:=${courier_rgbd_sensors}"
      )
      endforeach()
  endforeach()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config gazebo launch Media meshes robots ros2_control urdf)
