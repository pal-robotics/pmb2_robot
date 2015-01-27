^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ant_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.9.5 (2014-12-04)
------------------

0.9.4 (2014-11-25)
------------------

0.9.3 (2014-11-21)
------------------
* Add pal-xenomai dependency
* Contributors: Enrique Fernandez

0.9.2 (2014-11-21)
------------------
* Don't install headers (same reason as library)
* Don't export library
  NOTE this is a workaround because the xenomai target makes the library
  to be installed in a non-standard lib subfolder
* Build against xenomai target
* Contributors: Enrique Fernandez

0.9.1 (2014-11-17)
------------------
* adds dummy rgbd sensors
  this is needed because the base_rgbd_camera_link is revolute,
  which is needed to have a correct gazebo simulation when pitch != 0.0
  and roll != 0.0
* Accesor -> Accessor typo fix
* pal_ros_control upgrade
  pal_ros_control:
  - Allow a robot to provide read/write access to position, velocity and effort,
  plus write access to actuator current limit (up to seven values).
  - The above are all optional, so one can choose what a robot has. Before,
  position read/write was enabled and hardcoded.
  - Rework mechanism for specifying actuator port names, which became a bit
  cumbersome now that there are up to six values to spepcify (R/W pos, vel, eff)
  - Add a hardware interface for actuator current limits.
  <robot>_hardware:
  - Adapt to the new actuator port names spacification.
  - REEM, REEMC and ULNA expose an actuator current limiting interface.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Enrique Fernandez, Victor Lopez
