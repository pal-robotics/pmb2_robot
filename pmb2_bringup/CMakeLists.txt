cmake_minimum_required(VERSION 3.5)
project(pmb2_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/joystick.yaml
    module/twist_mux.yaml
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

if(pal_configuration_cmake_FOUND)
  pal_register_configs(
    CONFIGS
      params/joy_teleop/10_joy_teleop_pmb2.yaml
    RESOURCE_NAME
      joy_teleop
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_configuration_cmake package was not found. Configurations not registered.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
