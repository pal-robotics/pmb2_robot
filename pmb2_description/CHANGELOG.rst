^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.10.1 (2025-07-09)
-------------------
* fix use_sim_time
* Contributors: andreacapodacqua

5.10.0 (2025-07-08)
-------------------

5.9.0 (2025-06-17)
------------------
* add support to namespaces
* Contributors: antoniobrandi

5.8.3 (2025-06-06)
------------------
* Change path related to restructure path
* Change path from sensor courier to add_on cameras
* Delete sensors files
* Change path for hokuyo and courier
* Remove missing sensors.ros2_control.xacro
* Contributors: Aina, Noel Jimenez

5.8.2 (2025-03-28)
------------------
* Remove unused imu broadcaster and hardware interface
* Contributors: Noel Jimenez

5.8.1 (2025-03-25)
------------------

5.8.0 (2025-03-25)
------------------

5.7.0 (2025-02-24)
------------------

5.6.0 (2025-01-22)
------------------

5.5.0 (2025-01-22)
------------------
* support camera_model argument
* support add_on_module cobra
* use sensor qos profile
* support to structure cameras in simulation
* Contributors: antoniobrandi

5.4.0 (2024-11-05)
------------------
* Only run test_description test for the default config
* Test xacros with urdf_test
* Contributors: Mathias Lüdtke

5.3.1 (2024-10-15)
------------------

5.3.0 (2024-09-03)
------------------
* Merge branch 'man/feat/docking' into 'humble-devel'
  added docking link in urdf
  See merge request robots/pmb2_robot!145
* added docking link in urdf
* Merge branch 'fix/add_slash_to_nodes' into 'humble-devel'
  Add slash to node names on parameter files
  See merge request robots/pmb2_robot!146
* Add slash to node names on parameter files
* Contributors: Jordan Palacios, Noel Jimenez, josegarcia, martinaannicelli

5.2.0 (2024-08-29)
------------------

5.1.3 (2024-08-19)
------------------
* Merge branch 'tpe/fix/slipping' into 'humble-devel'
  Fix slippage gazebo issue happening on pmb2
  See merge request robots/pmb2_robot!144
* Fix slippage gazebo issue happening on pmb2
* Contributors: andreacapodacqua, thomas.peyrucain

5.1.2 (2024-08-07)
------------------

5.1.1 (2024-08-05)
------------------

5.1.0 (2024-08-02)
------------------

5.0.27 (2024-07-31)
-------------------
* Adapting gazebo parameters that was causing the base to move strangly
* Defauld add-on-module is no-add-on
* Contributors: David ter Kuile, thomas.peyrucain

5.0.26 (2024-07-09)
-------------------
* Add warning for pal_module_cmake not found
* Contributors: Noel Jimenez

5.0.25 (2024-06-28)
-------------------

5.0.24 (2024-06-27)
-------------------
* Merge branch 'dtk/rgbd-sensors' into 'humble-devel'
  Dtk/rgbd sensors
  See merge request robots/pmb2_robot!135
* Change courier sensors to add-on-module
* Change has_courier_rgbd_sensors to rgbd_sensors
* Merge branch 'fix/set_casters_suspension_fixed' into 'humble-devel'
  Set casters and suspension joints to fixed
  See merge request robots/pmb2_robot!134
* Set casters and suspension joints to fixed
* Contributors: David ter Kuile, Noel Jimenez, davidterkuile

5.0.23 (2024-06-26)
-------------------
* Merge branch 'dtk/move-robot-args' into 'humble-devel'
  Dtk/move robot args
  See merge request robots/pmb2_robot!133
* Change import for launch args
* Contributors: David ter Kuile, davidterkuile

5.0.22 (2024-06-25)
-------------------
* Merge branch 'omm/complete_std' into 'humble-devel'
  Complete std
  See merge request robots/pmb2_robot!129
* Move _link to top file xacro
* Remove colon from urdf
* Update xacro of the base
* Fix linters
* Add public sim
* URDF cleanup
* Fix tests
* Update copyright + move imu into sensors
* Cleanup + move deg_to_rad to robot urdf
* Bring back the parameters
* Rename file because of dependency error, will be handled in another task
* Unified style and namings
* Cleaning and proper main structure
* Launch standarization
* Contributors: David ter Kuile, Oscar, davidterkuile, oscarmartinez, thomas.peyrucain

5.0.21 (2024-06-20)
-------------------

5.0.20 (2024-06-03)
-------------------
* Merge branch 'fix/aca/reduced-laser-noise' into 'humble-devel'
  reduced laser_noise
  See merge request robots/pmb2_robot!130
* reduced laser_noise
* Merge branch 'omm/fix/imu_proper_urdf' into 'humble-devel'
  IMU proper placement
  See merge request robots/pmb2_robot!127
* Style and IMU proper placement
* Contributors: Oscar, andreacapodacqua, davidterkuile

5.0.19 (2024-04-11)
-------------------
* added public sim config mobile base controller and restored original laser noise
* enable dlo sim and reduce laser noise
* Contributors: andreacapodacqua

5.0.18 (2024-04-11)
-------------------

5.0.17 (2024-04-10)
-------------------

5.0.16 (2024-02-02)
-------------------

5.0.15 (2023-12-18)
-------------------
* Use pal_urdf_utils materials and deg_to_rad
* Contributors: Noel Jimenez

5.0.14 (2023-11-22)
-------------------

5.0.13 (2023-11-14)
-------------------
* Add website tag
* Rename description and controller modules
* Contributors: Noel Jimenez

5.0.12 (2023-11-13)
-------------------
* Set use_sim_time false as default
* Contributors: Noel Jimenez

5.0.11 (2023-11-07)
-------------------
* Split bringup module
* Contributors: Noel Jimenez

5.0.10 (2023-10-19)
-------------------
* fix laser_node name
* Contributors: andreacapodacqua

5.0.9 (2023-09-20)
------------------

5.0.8 (2023-09-04)
------------------
* Add use_sim_time argument to use it in the robot description
* Contributors: Noel Jimenez

5.0.7 (2023-07-11)
------------------
* Remove pal flags dependency
* Contributors: Noel Jimenez

5.0.6 (2023-06-13)
------------------
* remove namespace for base sonars
* Contributors: Noel Jimenez

5.0.5 (2023-05-15)
------------------

5.0.4 (2023-04-28)
------------------
* fix indentation
* fix odometry plugin tags
* Contributors: Noel Jimenez

5.0.3 (2023-04-17)
------------------
* rename motors to actuators
* Contributors: Noel Jimenez

5.0.2 (2023-03-06)
------------------

5.0.1 (2023-03-02)
------------------

5.0.0 (2023-02-08)
------------------
* Merge branch 'transmissions' into 'humble-devel'
  Update wheels transmissions
  See merge request robots/pmb2_robot!91
* update wheels transmissions
* Contributors: Jordan Palacios, Noel Jimenez

4.0.5 (2022-10-21)
------------------
* Merge branch 'cleanup' into 'humble-devel'
  update package.xml deps, indentation fix
  See merge request robots/pmb2_robot!85
* indentation fix
* update package.xml deps
* Merge branch 'fix_gz_physics' into 'humble-devel'
  Tune wheels physics to avoid slippage
  See merge request robots/pmb2_robot!84
* tune wheels physics to avoid slippage
* Merge branch 'update_copyright' into 'humble-devel'
  Update copyright
  See merge request robots/pmb2_robot!82
* update copyright
* Merge branch 'refactor_ld_population' into 'humble-devel'
  Refactor ld population
  See merge request robots/pmb2_robot!81
* refactor LaunchDescription population
* Merge branch 'update_maintainers' into 'humble-devel'
  update maintainers
  See merge request robots/pmb2_robot!80
* update maintainers
* Merge branch 'humble_fixes' into 'humble-devel'
  humble distro fixes
  See merge request robots/pmb2_robot!79
* linters
* launch files refactor
* material Black
* Merge branch 'pmb2_launcher' into 'galactic-devel'
  pmb2 launcher
  See merge request robots/pmb2_robot!76
* robot control plugin if no simulation
* Merge branch 'use-world-odometry-ros2' into 'foxy-devel'
  use our world odometry plugin
  See merge request robots/pmb2_robot!72
* use our world odometry plugin
* Contributors: Jordan Palacios, Noel Jimenez, Noel Jimenez Garcia, Victor Lopez, victor

4.0.4 (2021-10-19)
------------------
* Add missing exec dependency
* Contributors: Victor Lopez

4.0.3 (2021-10-19)
------------------
* Update to new transmission format
* Cleanup
* Contributors: Victor Lopez

4.0.2 (2021-07-15)
------------------
* Fix typo
* Contributors: Victor Lopez

4.0.1 (2021-07-15)
------------------
* Fix missing dependencies
* Contributors: Victor Lopez

4.0.0 (2021-07-12)
------------------
* Set back old version number before release
* Restructuring code and add description test
* No laser value is 'no-laser', change rgbd_sensors to courier_rgbd_sensors
* Tune a bit physics to avoid joint slippage, specially in TIAGo
* Correct physic property names for newer gazebo
* Add ROS2 control imu
* Add imu plugin
* Migrate rest of lasers to ROS2
* Renamed end_effector argument
* Format
* Fixed some tiago arguments
* Add linters to pmb2_description and apply fixes
* Move param utils to launch_pal
* Add show.launch.py
* Remove gazebo laser plugin namespace
* Merge branch 'single_ros2_control_system' into 'foxy-devel'
  Single ROS2 control system
  See merge request robots/pmb2_robot!65
* All joints now form part of a single ros2_control system
* Fixes to gazebo ros2 control param changes
* Update how ros2 gazebo plugin is loaded
* Add wheel ros2_control file
* First working version
* Remove comments to workaround https://github.com/ros2/launch_ros/issues/214
* First WIP of upload.py
* Contributors: Jordan Palacios, Victor Lopez, victor

3.0.14 (2021-01-18)
-------------------
* Merge branch 'fix_wheel_slippage' into 'erbium-devel'
  Fix wheel slippage
  See merge request robots/pmb2_robot!62
* Tuning mu1,mu2 parameters for reducing slippage during pure rotational speed cmds
* Uss sphere and tuned contacts for collision links
* test sphere collision mesh and contact parameters
* Contributors: Luca Marchionni, victor

3.0.13 (2020-07-30)
-------------------

3.0.12 (2020-07-16)
-------------------

3.0.11 (2020-07-10)
-------------------
* Merge branch 'elp-camera' into 'erbium-devel'
  Fix ELP rgb camera position and add its gazebo plugin
  See merge request robots/pmb2_robot!58
* Fix ELP rgb camera position and add its gazebo plugin
* Contributors: Sara Cooper, procopiostein

3.0.10 (2019-10-21)
-------------------

3.0.9 (2019-10-02)
------------------

3.0.8 (2019-09-27)
------------------

3.0.7 (2019-09-25)
------------------

3.0.6 (2019-09-20)
------------------
* scan_raw is the default laser topic
* Contributors: Procópio Stein

3.0.5 (2019-09-10)
------------------
* Melodic compatibility
* Contributors: Victor Lopez

3.0.4 (2019-07-17)
------------------
* Merge branch 'multi_pmb2' into 'erbium-devel'
  Changes for multi pmb2 simulation
  See merge request robots/pmb2_robot!44
* Changes for multi pmb2 simulation
* Contributors: Adria Roig, Victor Lopez

3.0.3 (2019-04-09)
------------------
* Merge branch 'enable_sonars' into 'erbium-devel'
  Add sonars argument to base_sensors
  See merge request robots/pmb2_robot!42
* Added sonars argument to base_sensors
* Contributors: Jordan Palacios, Victor Lopez

3.0.2 (2019-01-31)
------------------
* Merge branch 'fix-inertia' into 'erbium-devel'
  Fix inertial parameters of the caster wheels
  See merge request robots/pmb2_robot!41
* Fix inertial parameters of the caster wheels
  Also added friction and damping to improve behavior
* Contributors: Victor Lopez

3.0.1 (2018-12-20)
------------------
* Fix tests
* Contributors: Victor Lopez

3.0.0 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Remove upload_pmb2.launch
  See merge request robots/pmb2_robot!40
* Add rgbd sensors
* Change robot parameter name
* Parametrize urdf
* Remove upload_pmb2.launch
* Contributors: Victor Lopez

2.0.8 (2018-11-27)
------------------
* Merge branch 'remove-caster-friction' into 'erbium-devel'
  Remove caster friction so it doesn't push base around
  See merge request robots/pmb2_robot!34
* Remove caster friction so it doesn't push base around
* Contributors: Victor Lopez

2.0.7 (2018-07-30)
------------------
* Merge branch 'fix-xacro-warnings' into 'erbium-devel'
  prepend missing 'xacro' tag
  See merge request robots/pmb2_robot!33
* prepend missing 'xacro' tag
* Merge branch 'fix-warning-typo' into 'erbium-devel'
  fix typo
  See merge request robots/pmb2_robot!32
* fix typo
* Contributors: Hilario Tome, Jordi Pages, Victor Lopez

2.0.6 (2018-04-27)
------------------
* Merge branch 'fix_tf_depth_sensor' into 'erbium-devel'
  fixed the frame wrongly removed previously
  See merge request robots/pmb2_robot!31
* removed rgb frames that are not present in this sensor
* fixed the frame wrongly removed previously
* Contributors: Andrei Pasnicenco, Hilario Tome, Procópio Stein

2.0.5 (2018-04-17)
------------------
* Merge branch 'fix-tests-broken-due-to-stl' into 'erbium-devel'
  Revert "fixed warning when loading stl file"
  See merge request robots/pmb2_robot!29
* Revert "fixed warning when loading stl file"
  This reverts commit 49e84804a24372815b2b500159369f1d63d02857.
* Contributors: Hilario Tome, Procópio Stein

2.0.4 (2018-04-17)
------------------

2.0.3 (2018-04-17)
------------------
* Merge branch 'test-branch' into 'erbium-devel'
  Test branch
  See merge request robots/pmb2_robot!27
* Merge branch 'fix-stl' into test-branch
* Merge remote-tracking branch 'origin/fix_xacro_warning' into test-branch
* fixed warning when loading stl file
* fix missing xacro namespace
* Merge remote-tracking branch 'origin/fixing_sim' into test-branch
* Merge remote-tracking branch 'origin/deprecate_upload_pmb2' into test-branch
* Merge remote-tracking branch 'origin/fix_xacro_warning' into test-branch
* updated urdf file to get correct mesh and remove rgb related info
* added structure sensor mesh
* deprecate upload_pmb2
* normalize xmlns across xacro files
* fix xacro warning
  deprecated: xacro tags should be prepended with 'xacro' xml namespace.
  Use the following script to fix incorrect usage:
  find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
* rm usuless caster 1 collision mesh
* fix casters
* Contributors: Jeremie Deray, Procópio Stein

2.0.2 (2018-04-13)
------------------
* reduced sonars max range to avoid noise
* Contributors: Procópio Stein

2.0.1 (2018-03-29)
------------------
* delete transmission for passive joints
* Contributors: Andrei Pasnicenco

2.0.0 (2018-03-26)
------------------
* Merge branch 'collision_fix' into 'erbium-devel'
  caster wheels and base collision
  See merge request robots/pmb2_robot!19
* gtests passed on the flat surface
* linear move test passed
* revert testing torque value
* transmission caster
* sufficient for tests
* fix castor joints transmission
* rotate and climb with a virtual suspension system and use caster wheels
* virtual suspension and rolling caster wheels
* high-res mesh of the base for visual geometry
* Friction coeffs adjusted
* caster wheels and base collision
  Note: Frictin coeffs mu1, mu2 in caster.gazebo.xacro can me adjusted to make robot stopping immideately
* Contributors: Andrei Pasnicenco, Victor Lopez

1.1.14 (2018-01-30)
-------------------
* Merge branch 'test_urdf' into 'dubnium-devel'
  Add URDF tests
  See merge request robots/pmb2_robot!17
* Remove exec dependencies for pal_gazebo_plugins
* Add URDF tests
* Contributors: Adria Roig, Adrià Roig, davidfernandez

1.1.13 (2017-09-27)
-------------------
* renamed depth sensor
* add rgbd related files and config to description
* Contributors: Procópio Stein

1.1.12 (2017-06-30)
-------------------

1.1.11 (2017-06-30)
-------------------

1.1.10 (2017-06-29)
-------------------

1.1.9 (2017-06-28)
------------------
* upgraded packages format, maintainers and license
* Contributors: Procópio Stein

1.1.8 (2017-04-11)
------------------

1.1.7 (2017-02-23)
------------------

1.1.6 (2016-11-07)
------------------
* invert sonars 1 and 3
* Contributors: Jordi Pages

1.1.5 (2016-10-24)
------------------
* Now launch files are more like those for TIAGo
* add tiago_support as maintainer
* Contributors: Jordan Palacios, Jordi Pages

1.1.4 (2016-07-04)
------------------
* corrected imu frame, z always point upwards
  this is because the imu 6050 zeros itself (at least wrt pitch)
* Contributors: Procópio Stein

1.1.3 (2016-06-15)
------------------
* update sonars min/max range
* Contributors: Jeremie Deray

1.1.2 (2016-06-03)
------------------
* sonar ID two digit
* Add imu controller to launch
* Add imu gazebo plugin config
* 1.1.1
* Update changelog
* Updated to new generic pal hardware gazebo plugin
* Simplified base collision
  Now the base_link has a mesh that touches with the ground
* Contributors: Jeremie Deray, Sam Pfeiffer

1.1.1 (2016-04-15)
------------------
* Updated to new generic pal hardware gazebo plugin
* Simplified base collision
  Now the base_link has a mesh that touches with the ground
* Contributors: Sam Pfeiffer

1.1.0 (2016-03-15)
------------------
* urdf use macro param default value
* fix urdf laser
* Contributors: Jeremie Deray

1.0.6 (2016-03-03)
------------------

1.0.5 (2016-02-09)
------------------
* update gazebo sick 561 571 with proper params
* rename base_default to base_sensors
* remove base_full.urdf.xacro
* add gazebo draft sick 561 & 571
* pmb2 desscription upload default
* rm full urdf
* base_default now holds all sensors with option
* pmb2 urdf diff Sick
* Contributors: Jeremie Deray

1.0.4 (2015-10-26)
------------------

1.0.3 (2015-10-06)
------------------

1.0.2 (2015-10-05)
------------------

1.0.1 (2015-10-01)
------------------
* 1.0.0
* Add changelog
* Add changelog
* Merging metal base branch
* urdf full calls default & add sonar/micro
* urdf default calls base & add laser
* urdf base contains basics e.g. wheels
* add full_sick urdf
* add base_default urdf
* renamed base -> base_full
* Update maintainer
* Replace caster collision with spheres, fix spinning
* Remove spanish character nonvalid to xacro
* Update placement and name of base imu
* Add collision to antenna
* Update caster locations
* Add microphone locations
* Added sonars with proper colors
* Add color to gazebo
* Add antennas
* New meshes
* Remove references to xtion
* Remove robot model scripts
* Add inertial params to main body
* Remove bumpers
* Remove rear cover
* More battery removed
* Remove charger
* Remove battery
* Remove base_rgbd
* Fix color of wheels in gazebo
* Add new cover and orange ring around body
* Contributors: Bence Magyar, Jeremie Deray, Luca Marchionni

1.0.0 (2015-09-29)
------------------
* Add changelog
* Merging metal base branch
* urdf full calls default & add sonar/micro
* urdf default calls base & add laser
* urdf base contains basics e.g. wheels
* add full_sick urdf
* add base_default urdf
* renamed base -> base_full
* Update maintainer
* Replace caster collision with spheres, fix spinning
* Remove spanish character nonvalid to xacro
* Update placement and name of base imu
* Add collision to antenna
* Update caster locations
* Add microphone locations
* Added sonars with proper colors
* Add color to gazebo
* Add antennas
* New meshes
* Remove references to xtion
* Remove robot model scripts
* Add inertial params to main body
* Remove bumpers
* Remove rear cover
* More battery removed
* Remove charger
* Remove battery
* Remove base_rgbd
* Fix color of wheels in gazebo
* Add new cover and orange ring around body
* Contributors: Bence Magyar, Jeremie Deray, Luca Marchionni

0.10.0 (2015-07-14)
-------------------

0.9.10 (2015-02-27)
-------------------
* Merge from REEM-C params
* Fix and add link names in macro
* Contributors: Bence Magyar

0.9.9 (2015-02-18)
------------------

0.9.8 (2015-02-18)
------------------
* Add inertial block to xtion pro live
* Add inertial block to range sensor
* Add conditional for base rgbd sensor
* Chop off frontal antennas
* Use ${name} for imu
* Put sonars with its rear cover
* Make rgbd camera fixed
* Add microphones
* Add bumper
* Update meshes
* Use base_footprint_link
* Update meshes
* Add comment to show Joint, Child, Parent
* Remove sensors not needed
* Use 0.27m for footprint radius
* Add kinematics and stl files (except for the base)
* Add kinematics xlsx to URDF converter/helper
* Contributors: Bence Magyar, Enrique Fernandez

0.9.7 (2015-02-02)
------------------
* Update URDF (only locations)
* Replace ant -> pmb2
* Rename files
* Contributors: Enrique Fernandez
