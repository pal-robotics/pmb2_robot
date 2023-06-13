^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.0.6 (2023-06-13)
------------------

5.0.5 (2023-05-15)
------------------
* Merge branch 'fix/controller' into 'humble-devel'
  safer controller params
  See merge request robots/pmb2_robot!97
* safer controller params
* Contributors: antoniobrandi

5.0.4 (2023-04-28)
------------------
* set sim time for gazebo controller_manager
* Contributors: Noel Jimenez

5.0.3 (2023-04-17)
------------------

5.0.2 (2023-03-06)
------------------
* Merge branch 'rm_use_sim_time' into 'humble-devel'
  remove use_sim_time parameter
  See merge request robots/pmb2_robot!94
* remove use_sim_time parameter
* Contributors: Jordan Palacios, Noel Jimenez

5.0.1 (2023-03-02)
------------------
* Merge branch 'fix_controllers_config' into 'humble-devel'
  Remove initial / from controllers config
  See merge request robots/pmb2_robot!93
* remove initial / from controllers config
* Contributors: Jordan Palacios, Noel Jimenez

5.0.0 (2023-02-08)
------------------

4.0.5 (2022-10-21)
------------------
* Merge branch 'cleanup' into 'humble-devel'
  update package.xml deps, indentation fix
  See merge request robots/pmb2_robot!85
* update package.xml deps
* Merge branch 'update_copyright' into 'humble-devel'
  Update copyright
  See merge request robots/pmb2_robot!82
* update copyright
* Merge branch 'cleanup' into 'humble-devel'
  Cleanup
  See merge request robots/pmb2_robot!83
* cleanup
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
* update mobile_base_controller params
* Merge branch 'pmb2_launcher' into 'galactic-devel'
  pmb2 launcher
  See merge request robots/pmb2_robot!76
* add todo
* Contributors: Jordan Palacios, Noel Jimenez, Noel Jimenez Garcia

4.0.4 (2021-10-19)
------------------

4.0.3 (2021-10-19)
------------------
* Merge branch 'add_extra_joints' into 'foxy-devel'
  Add extra joints
  See merge request robots/pmb2_robot!74
* add extra joints to joint state
* add extra joints to joint state
* Contributors: cescfolch, victor

4.0.2 (2021-07-15)
------------------

4.0.1 (2021-07-15)
------------------
* Fix missing dependencies
* Contributors: Victor Lopez

4.0.0 (2021-07-12)
------------------
* Correct dependency name
* Using joint_state_broadcaster instead of controller
* Adapt to proper parameter naming
* Add linters to pmb2_bringup and apply fixes
* use_sim_time in controllers and cleanup
* Split default_controllers launch file
* Fixes to gazebo ros2 control param changes
* More fixes to default_controllers
* Add default_controllers.launch.py
* Update default_controllers.yaml
  Update gazebo controller name
* Add pmb2_controller_configuration
* First working version
* Contributors: Jordan Palacios, Victor Lopez

3.0.14 (2021-01-18)
-------------------

3.0.13 (2020-07-30)
-------------------
* Merge branch 'rename_tf_prefix' into 'erbium-devel'
  Rename tf_prefix to robot_namespace
  See merge request robots/pmb2_robot!60
* Rename tf_prefix to robot_namespace
* Contributors: davidfernandez, victor

3.0.12 (2020-07-16)
-------------------

3.0.11 (2020-07-10)
-------------------
* Merge branch 'fix-changelog' into 'erbium-devel'
  fixed changelog
  See merge request robots/pmb2_robot!55
* fixed changelog
* Contributors: Procópio Stein

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

3.0.5 (2019-09-10)
------------------

3.0.4 (2019-07-17)
------------------
* Merge branch 'multi_pmb2' into 'erbium-devel'
  Changes for multi pmb2 simulation
  See merge request robots/pmb2_robot!44
* Changes for multi pmb2 simulation
* Contributors: Adria Roig, Victor Lopez

3.0.3 (2019-04-09)
------------------

3.0.2 (2019-01-31)
------------------

3.0.1 (2018-12-20)
------------------

3.0.0 (2018-12-19)
------------------

2.0.8 (2018-11-27)
------------------

2.0.7 (2018-07-30)
------------------

2.0.6 (2018-04-27)
------------------

2.0.5 (2018-04-17)
------------------

2.0.4 (2018-04-17)
------------------
* Merge branch 'fixed_extra_joints' into 'erbium-devel'
  fixed extra joint param for caster wheels in joint_state_controller
  See merge request robots/pmb2_robot!28
* fixed extra joint param for caster wheels in joint_state_controller
* Contributors: Hilario Tome

2.0.3 (2018-04-17)
------------------

2.0.2 (2018-04-13)
------------------

2.0.1 (2018-03-29)
------------------
* Merge branch 'publish_cmd_true' into 'dubnium-devel'
  Publish_cmd to true, needed since kinetic version of mobile_base_controller
  See merge request robots/pmb2_robot!18
  (cherry picked from commit 7e311803a38db071956acaa3550893bdcac967f2)
  20fad179 Publish_cmd to true, needed since kinetic version of mobile_base_controller
* Contributors: Procópio Stein

2.0.0 (2018-03-26)
------------------

1.1.14 (2018-01-30)
-------------------

1.1.13 (2017-09-27)
-------------------

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

1.1.5 (2016-10-24)
------------------
* Now launch files are more like those for TIAGo
* add tiago_support as maintainer
* Contributors: Jordan Palacios, Jordi Pages

1.1.4 (2016-07-04)
------------------

1.1.3 (2016-06-15)
------------------

1.1.2 (2016-06-03)
------------------
* Add imu controller to launch
* 1.1.1
* Update changelog
* Contributors: Sam Pfeiffer

1.1.0 (2016-03-15)
------------------

1.0.6 (2016-03-03)
------------------

1.0.5 (2016-02-09)
------------------

1.0.4 (2015-10-26)
------------------
* adding new config package for pmb2-5
* Publish wheel cmd for leds feedback
* Contributors: Luca Marchionni

1.0.3 (2015-10-06)
------------------

1.0.2 (2015-10-05)
------------------

1.0.1 (2015-10-01)
------------------
* 1.0.0
* Add changelog
* Add changelog
* Remove imu because on pmb2 it will be published outside ros_control
* Merging metal base branch
* Add missing dependency
* Update maintainer
* Update placement and name of base imu
* Contributors: Bence Magyar, Luca Marchionni

0.10.0 (2015-07-14)
-------------------
* Use generic pal_ros_control component
  - Load configuration for generic pal_ros_control component.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.10 (2015-02-27)
-------------------
* Add publish_cmd and publish_wheel_data params
* Reduce yaw covariance (it's in radians)
* Contributors: Enrique Fernandez

0.9.9 (2015-02-18)
------------------
* Put very low cov for z, pitch, roll
* Contributors: Enrique Fernandez

0.9.8 (2015-02-18)
------------------
* Add params for pose covariance
* Update meshes
* Use base_footprint_link
* Contributors: Enrique Fernandez

0.9.7 (2015-02-02)
------------------
* Replace ant -> pmb2
* Rename files
* Contributors: Enrique Fernandez
