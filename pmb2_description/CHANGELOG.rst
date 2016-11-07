^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.1.1 (2016-04-15)
------------------
* Updated to new generic pal hardware gazebo plugin
* Simplified base collision
  Now the base_link has a mesh that touches with the ground
* Contributors: Sam Pfeiffer

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
