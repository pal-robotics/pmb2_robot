^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
