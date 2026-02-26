# Audibot

A ROS 2 / Gazebo simulation of an Audi R8-based autonomous vehicle. The package provides a realistic vehicle dynamics model with steering geometry, speed control, and a full sensor suite.

## Packages

| Package | Description |
|---------|-------------|
| `audibot` | Metapackage |
| `audibot_description` | Meshes and SDF robot descriptions |
| `audibot_gazebo` | Gazebo plugin, launch files, and world files |

---

## Launching the Simulator

Two ready-to-use launch files are provided, one for each vehicle color variant.

**Blue Audibot:**
```bash
ros2 launch audibot_gazebo blue_audibot.launch.xml
```

**Orange Audibot:**
```bash
ros2 launch audibot_gazebo orange_audibot.launch.xml
```

Both launch files start:
- Gazebo with an empty world
- The ROS ↔ Gazebo topic bridge
- `robot_state_publisher` for TF transforms
- `rqt_publisher` for manually sending control commands
- RViz2 with a pre-configured display

### Launch Arguments

The underlying `gazebo_bringup.launch.py` accepts the following arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_sdf_file` | *(required)* | Full path to the robot SDF file |
| `world_sdf_file` | *(required)* | Full path to the world SDF file |
| `gz_bridge_file` | *(required)* | Full path to the ROS/GZ bridge YAML config |
| `verbose` | `false` | Enable verbose Gazebo output |
| `start_paused` | `false` | Start simulation paused |
| `headless` | `false` | Run server only (no GUI) |

---

## Including Audibot in Your Own Launch File

### XML launch file

```xml
<?xml version="1.0"?>
<launch>

    <!-- Include the blue audibot simulator -->
    <include file="$(find-pkg-share audibot_gazebo)/launch/blue_audibot.launch.xml" />

    <!-- Your nodes here -->
    <node pkg="my_package" exec="my_node" name="my_node">
        <param name="use_sim_time" value="true" />
    </node>

</launch>
```

### Python launch file

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    audibot_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('audibot_gazebo'),
                'launch', 'blue_audibot.launch.xml'
            )
        )
    )

    my_node = Node(
        package='my_package',
        executable='my_node',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([audibot_launch, my_node])
```

> **Note:** Always set `use_sim_time: true` on your nodes so they use the `/clock` topic from Gazebo instead of wall-clock time.

---

## ROS Topics

### Control Inputs (subscribe to these to drive the robot)

| Topic | Type | Description |
|-------|------|-------------|
| `/audibot/steering_cmd` | `std_msgs/msg/Float64` | Steering wheel angle in radians. Range: approximately ±8.8 rad (±3.2 turns lock-to-lock). Internally maps to a front wheel angle of ±0.375 rad. |
| `/audibot/throttle_cmd` | `std_msgs/msg/Float64` | Throttle percentage. Range: `0.0` (no throttle) to `1.0` (full throttle). |
| `/audibot/brake_cmd` | `std_msgs/msg/Float64` | Brake torque in N·m. Range: `0.0` (no braking) to `5000.0` (full braking). Brakes take priority over throttle. |
| `/audibot/speed_cmd` | `std_msgs/msg/Float64` | Target speed in m/s. Range: `0.0` to `~36.1` m/s (130 km/h). When published, this overrides `throttle_cmd` and `brake_cmd` and uses an internal PI controller to reach the target speed. |
| `/audibot/gear_cmd` | `std_msgs/msg/UInt32` | Gear selection. `0` = Drive (forward), `1` = Reverse. |

> **Tip:** Use either `throttle_cmd` + `brake_cmd` for low-level control, or `speed_cmd` for higher-level speed tracking. Do not mix them — `speed_cmd` overrides throttle and brake when active.
>
> **Command timeouts:** All control commands time out after **0.1 seconds**. Commands must be published continuously (e.g., at 10 Hz or faster) to keep the vehicle moving.

### Sensor Outputs (subscribe to these to perceive the environment)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/audibot/imu` | `sensor_msgs/msg/Imu` | 100 Hz | IMU data: linear acceleration, angular velocity, and orientation. Frame: `chassis`. |
| `/audibot/twist` | `geometry_msgs/msg/TwistStamped` | 100 Hz | Vehicle velocity: linear (m/s) and angular (rad/s) in the body frame. |
| `/audibot/fix` | `sensor_msgs/msg/NavSatFix` | 1 Hz | GPS position fix. Default origin: 45.0° N, 81.0° W. |
| `/audibot/gnss_heading` | `std_msgs/msg/Float64` | 50 Hz | Vehicle heading derived from GNSS, in radians. |
| `/audibot/camera/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | Forward-facing camera image. Resolution: 640×480 RGB. Mounted 2.0 m forward and 1.5 m above ground, pitched down 0.25 rad. |
| `/audibot/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | Camera intrinsic parameters for `/audibot/camera/image_raw`. |
| `/joint_states` | `sensor_msgs/msg/JointState` | 100 Hz | Positions and velocities of all wheel and steering joints. |
| `/tf` | `tf2_msgs/msg/TFMessage` | 100 Hz | Transform tree: `world` → `base_footprint` → `chassis` → wheels and steering links. |
| `/clock` | `rosgraph_msgs/msg/Clock` | — | Simulation time. Subscribe via `use_sim_time: true` rather than directly. |

---

## Vehicle Parameters

| Parameter | Value |
|-----------|-------|
| Wheelbase | 2.67 m |
| Track width | 1.638 m |
| Max front wheel steer angle | ±0.375 rad (±21.5°) |
| Steering ratio | 17.3 |
| Max speed | 130 km/h (≈36.1 m/s) |
| Max brake torque | 5000 N·m |
| Vehicle mass | 1700 kg |
| Wheel radius | 0.36 m |
