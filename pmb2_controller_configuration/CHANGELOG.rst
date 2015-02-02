^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.6 (2015-01-27)
------------------
* Add preserve turning radius and jerk limit params
  Note jerk limits are disabled to avoid any kind of oscillation they
  might produce
* Set less aggressive acceleration limits
  With the previous values, with large accelerations (specially when
  starting to move from 0) the odometry wasn't good enough; that could
  be related with the rolling mean done in the diff_drive_controller
* Re-calibrate odometry
  Note that with acceleration there are problems, probably because of the
  mean filter in the diff_drive_controller
* Contributors: Enrique Fernandez

0.9.5 (2014-12-04)
------------------

0.9.4 (2014-11-25)
------------------

0.9.2 (2014-11-21)
------------------

0.9.1 (2014-11-17)
------------------
* updates multipliers (odometry calibration)
* sets max velocity to 1.0m/s
  NOTE with the current control 2.0m/s is not feasible
  backwards speed also reduced
* Contributors: Enrique Fernandez
