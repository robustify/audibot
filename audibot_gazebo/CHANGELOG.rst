^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audibot_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2022-11-01)
------------------
* Add nav_msgs dependency to audibot_gazebo
* Publish odometry and steering angle feedback
* Remove tf_prefixer.py that is no longer needed in ROS Noetic
* Contributors: Micho Radovnikovich, Tor Børve Rasmussen

0.2.1 (2020-07-29)
------------------
* Bump minimum CMake version to 3.0.2 as recommended for ROS Noetic
* Adds dependency on tf2_geometry_msgs
* Contributors: Micho Radovnikovich

0.2.0 (2020-07-25)
------------------
* Implements tf_prefix functionality that was removed from robot_state_publisher
* Migrates from tf to tf2
* Prevents publishing TF frame transforms with the same timestamp
* Changes names of joints to be different from links
* Contributors: Micho Radovnikovich

0.1.1 (2020-07-25)
------------------
* Increases max braking torque to 8000 Nm
* Adds simulation of Drive and Reverse gears
* Contributors: Micho Radovnikovich

0.1.0 (2018-09-30)
------------------
* First release
* Contributors: Micho Radovnikovich
