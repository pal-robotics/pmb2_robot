^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.31 (2023-05-09)
-------------------

3.0.30 (2023-05-04)
-------------------

3.0.29 (2023-04-24)
-------------------

3.0.28 (2023-04-18)
-------------------

3.0.27 (2023-03-23)
-------------------

3.0.26 (2023-03-13)
-------------------

3.0.25 (2023-02-07)
-------------------

3.0.24 (2022-10-24)
-------------------
* Merge branch 'feat/robust-odometry-integration' into 'erbium-devel'
  Disable odom tf publication
  See merge request robots/pmb2_robot!86
* Update mobile_base_controller.yaml
* Contributors: josegarcia

3.0.23 (2022-08-17)
-------------------

3.0.22 (2022-08-10)
-------------------

3.0.21 (2022-03-16)
-------------------
* Merge branch 'use-tf2-convention' into 'erbium-devel'
  Use tf2 convention
  See merge request robots/pmb2_robot!75
* added changes to use tf2 convention
* Contributors: Jordan Palacios, josegarcia

3.0.20 (2021-09-01)
-------------------

3.0.19 (2021-08-11)
-------------------

3.0.18 (2021-07-19)
-------------------

3.0.17 (2021-06-23)
-------------------

3.0.16 (2021-02-15)
-------------------

3.0.15 (2021-01-28)
-------------------

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
