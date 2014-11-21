^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ant_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2014-11-21)
------------------
* Update rgbd camera extrinsic calibration
* refs #10350 : fixes base_rgbd_y collision (y, yaw)
  also sets the yaw to 7.1º (current value on the real robot)
* Contributors: Enrique Fernandez

0.9.1 (2014-11-17)
------------------
* sets range for long range [0.45, 10.0]m
* sets QVGA@6Hz like in the real sensor
* adds xtion pro live for base v2
* adds depth and rgb frames
* adds rangeMax
* cosmetics
* renames gazebo macro
* adds inertia tensor
  without this the simulation was crashing
* updates joint name
* replaces fixed by revolute joint for softkinetic
* replace fixed by revolute joint
  Fixed joints are not fully supported by Gazebo. The joint specifying the pose of the xtion camera wrt base_link has been replaced by a 0 limits revolute joint. Fixes #10160
* use base_v2 rather than base urdf
* Adds vertical Xtion camera on top of mobile base
  The camera is also tilted so that it points mainly towards the floor to detect low profile obstacles as well as frontal ones
* decimates meshes
* renames TiM511 to TiM551 (the right name)
* updates base collision STL (now FOV 220º is OK)
* renames hokuyo STL
* adds base collision mesh and reduces laser FOV
  with the theoretic 2*110deg FOV the laser measurements touches
  the collision mesh, so it's been reduced to 2*108deg
* adds SICK TiM511 laser to URDF
* Merge branch 'wood_mobile_base_v2_urdf' into 'master'
  Wood Mobile Base v2.0 URDF
  Creates the URDF for the Wood Mobile Base v2.0, following the kinematics specification of:
  https://docs.google.com/a/pal-robotics.com/spreadsheet/ccc?key=0Al5pObG8WxjBdHFqd0kwa3BFTkhrSmVnbDhpa1NiclE&usp=drive_web#gid=5
  The STL files are still copied from the previous/original mobile base.
  Interestingly, the wheel URDF has different sign for the left and right wheel. It looks OK, but maybe it's wrong.
* removes cylinder for the base collision
  and also related params; we keep this commit to have the approximate
  values of them
* uses the base v2.0 STL for the collision
  A cylinder close the laser window!
* uses caster 1&2 v2.0 and base v2.0 STL
* uses wheel v2.0 STL
* removes not needed surface element
* removes visual and collision definitions not used
* updates copyright year to 2014
* removes old playerstage xmlns items
* sets same name as softkinetic_ds311 rgbd camera
* fixes wheel inertias for 1st mobile base
* urdf for wood mobile base v2.0
  see:
  https://docs.google.com/a/pal-robotics.com/spreadsheet/ccc?key=0Al5pObG8WxjBdHFqd0kwa3BFTkhrSmVnbDhpa1NiclE&usp=drive_web#gid=5
* uses vertical (with no pitch/roll) rgbd pose
* refs #9791 : single urdf for softkinetic_ds311 with several poses
* fixes origin (must be origin twice, not optical_origin)
* renames rgbd to xtion_pro_live
* fixes non-supported used of properties to defined another property (suspicious)
* adds antennas and move them inside base
* adds softkinetic with hfov (rotated)
* updates antenna urdf files (more modular)
  They will be also used for the ant with softkinetic
* adds antennas to Rebujito (Stockbot)
* adds rgb part (needs 2 'sensor' items) and increases far clip
* fixes sensor orientation (rotate 90 around z)
* uses xtion_pro_live gazebo.xacro and removes openni one (so generic and not used)
* fixes softkinetic DS311 URDF
  NOTE now it uses the same topic names as the real driver
* refs #9791 : fixes softkinetic URDF (missed include)
* fixes dae file to point to softkinetic.png
* sets base_rgbd_camera_link as softkinetic frame name
* fixes base_rebujito
* adds softkinetic mesh (copied from xtion for now)
* Fix rebujito URDF casters
* adds softkinetic
* adds RGBD sensor with laser
  NOTE with robot:=rgbd_only we have only the RGBD
  (and we can use the depthimage_to_laserscan to simulate a laser)
* adds xtion gazebo simulation config; not used yet!
* adds rgbd (with xtion) urdf
* fixes rplidar dimensions (still a cylinder, but it's enough)
* sets approx. laser z
* adds rplidar model
* increases footprint to 0.3m radius
* removes not used numpy import
* enables the footprint
  NOTE that the laser to base offset is not yet considered,
  but even though it works acceptably now
* disables footprint
* adds circular footprint.yaml
  NOTE generated with:
  scripts/build_footprint.py 0.2 70 urdf/base/footprint.yaml
* adds circular footprint builder
* puts more similar colors
* reduces wheel torque from 50 to 6Nm (tested: works)
* refs #8219 : sets inertias
* refs #8219 : sets CoM and mass
* updates caster and wheel position, angle and STL
* fixes base STL and updates links positions
  NOTE mass and CoM, and inertias not set yet
* adds caster STL (not used yet)
* adds base stl (note the final one)
* adds friction to the caster and wheels
  NOTE by default mu = mu2 = 1.0; but with 0.01 the behavior is almost the same in gazebo
* refs #8415 : fixes inclinometer and ir receivers position (z-axis)
* refs #8415 : fixes base_footprint position and casters
* updates ir_receivers
* refs #8415 : disables gazebo emitter part
  NOTE the plugin is NOT available yet
* refs #8415 : adds dock URDF model
* refs #8415 : adds IR receiver visual
* refs #8415 : removes not needed code SDF param
  NOTE also removes horizontal part of scan/ray used
  as ir_receiver sensor, and set the samples dependant on
  the FOV
* Contributors: Enrique Fernandez, Jordi Pages, Paul Mathieu
