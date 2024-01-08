# Audibot Simulator

This repository contains a Gazebo simulation model of an Audi R8. It is meant to be a very flexible simulation platform that supports single and multiple vehicle simulations.

To customize the model, include `audibot.urdf.xacro` in the `audibot_description` package in another URDF file and add sensors, plugins, etc.

`single_vehicle_example.launch.py` in the `audibot_gazebo` package shows how a single vehicle can be simulated in the root namespace with no TF prefix.

`two_vehicle_example.launch.py` shows how multiple vehicles can be simulated at the same time, with each in its own namespace and with a unique TF prefix.

To control the vehicle, publish the following topics:

- **steering_cmd** - `std_msgs/msg/Float64` topic containing the desired steering wheel angle in radians
- **brake_cmd** - `std_msgs/msg/Float64` topic containing the desired brake torque in Newton-meters (Nm)
- **throttle_cmd** - `std_msgs/msg/Float64` topic containing the desired throttle percentage (range 0 to 1)
- **gear_cmd** - `std_msgs/msg/UInt8` topic containing the desired gear (`DRIVE` = 0, `REVERSE` = 1)

Ground truth speed and yaw rate feedback are provided on the **twist** topic, which is of type `geometry_msgs/TwistStamped`.

Current gear state is provided on the **gear_state** topic, which is of type `std_msgs/msg/UInt8`. The gear state starts in `DRIVE` by default.

The current steering wheel angle is provided on the **steering_state** topic, which is of type `std_msgs/msg/Float64`.

Position, orientation and twist is provided on the **odom** topic, which is of type `nav_msgs/msg/Odometry`.

Twist is provided on the **twist** topic, which is of type `geometry_msgs/msg/TwistStamped`.

Some useful kinematics parameters:

- Gear ratio between steering wheel and equivalent bicycle steer angle = 17.3 : 1
- Wheelbase = 2.65 meters
- Track width = 1.638 meters
- Wheel radius = 0.36 meters

To reset the world (stop car and return to origin) `ros2 service call /reset_world std_srvs/srv/Empty`
