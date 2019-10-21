^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.10 (2019-10-21)
-------------------
* Merge branch 'fix-twist' into 'erbium-devel'
  removed slash from out topi
  See merge request robots/pmb2_robot!54
* removed slash from out topi
* Merge branch 'remove-sonar-cloud' into 'erbium-devel'
  removed sonar cloud
  See merge request robots/pmb2_robot!50
* removed dep and maint
* removed sonar cloud
* Contributors: Procópio Stein

3.0.9 (2019-10-02)
------------------
* Merge branch 'fix-twist-default' into 'erbium-devel'
  output uses default instead of value
  See merge request robots/pmb2_robot!49
* output uses default instead of value
* Contributors: Procópio Stein

3.0.8 (2019-09-27)
------------------
* depends on speed-limit-node
* Contributors: Procópio Stein

3.0.7 (2019-09-25)
------------------
* Merge branch 'remove-speed-limit' into 'erbium-devel'
  removed speed limit
  See merge request robots/pmb2_robot!48
* removed speed limit
* Contributors: Procópio Stein

3.0.6 (2019-09-20)
------------------

3.0.5 (2019-09-10)
------------------

3.0.4 (2019-07-17)
------------------

3.0.3 (2019-04-09)
------------------

3.0.2 (2019-01-31)
------------------

3.0.1 (2018-12-20)
------------------

3.0.0 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Remove upload_pmb2.launch
  See merge request robots/pmb2_robot!40
* Add rgbd sensors
* Change robot parameter name
* Contributors: Victor Lopez

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

2.0.3 (2018-04-17)
------------------
* Merge branch 'test-branch' into 'erbium-devel'
  Test branch
  See merge request robots/pmb2_robot!27
* Merge remote-tracking branch 'origin/deprecate_upload_pmb2' into test-branch
* deprecate upload_pmb2
* Contributors: Jeremie Deray, Procópio Stein

2.0.2 (2018-04-13)
------------------

2.0.1 (2018-03-29)
------------------

2.0.0 (2018-03-26)
------------------

1.1.14 (2018-01-30)
-------------------

1.1.13 (2017-09-27)
-------------------
* removed commented and unused sensors
* Contributors: Procópio Stein

1.1.12 (2017-06-30)
-------------------
* speed limit starts disabled
* Contributors: Procópio Stein

1.1.11 (2017-06-30)
-------------------
* added robot pose dep
* Contributors: Procópio Stein

1.1.10 (2017-06-29)
-------------------
* added launch for robot pose publisher
* updated robot state publisher name and activated static tf
* Contributors: Procópio Stein

1.1.9 (2017-06-28)
------------------
* upgraded packages format, maintainers and license
* Contributors: Procópio Stein

1.1.8 (2017-04-11)
------------------
* added servoing_cmd_vel to twist_mux
* Contributors: Procópio Stein

1.1.7 (2017-02-23)
------------------
* added rviz_joy_vel to twist_mux
* refs #14797. Add required param for public sim
* Contributors: Jordi Pages, Procópio Stein

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
* 1.1.1
* Update changelog
* Contributors: Sam Pfeiffer

1.1.0 (2016-03-15)
------------------

1.0.6 (2016-03-03)
------------------

1.0.5 (2016-02-09)
------------------
* bringup default robot
* Contributors: Jeremie Deray

1.0.4 (2015-10-26)
------------------

1.0.3 (2015-10-06)
------------------
* mv sonar_to_cloud to pmb2_bringup.launch
* Contributors: Jeremie Deray

1.0.2 (2015-10-05)
------------------
* enable sonar after revert commit
* Revert "launch sonar_to_cloud from pmb2_bringup.launch"
  This reverts commit 2da0a9261b75d88a42d50102923d6f121329f2c2.
* Contributors: Jeremie Deray

1.0.1 (2015-10-01)
------------------
* rm double param load
* launch sonar_to_cloud from pmb2_bringup.launch
* rm rebujito.launch
* 1.0.0
* Add changelog
* sonar related launch call moved to pmb2.launch for easier overload
* Fixed error during ros_control starting on pmb2
* Merging metal base branch
* add pmb2_hardware.yaml !
* speed_limit add padding and sonar
* Update maintainer
* Remove rgbd layer
* Remove references to xtion
* Contributors: Bence Magyar, Jeremie Deray, Luca Marchionni

0.10.0 (2015-07-14)
-------------------
* Use generic pal_ros_control component
  - Load configuration for generic pal_ros_control component.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.10 (2015-02-27)
-------------------

0.9.9 (2015-02-18)
------------------

0.9.8 (2015-02-18)
------------------

0.9.7 (2015-02-02)
------------------
* Replace ant -> pmb2
* Rename files
* Contributors: Enrique Fernandez
