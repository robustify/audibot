import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(package='audibot_description').find('audibot_description')
    urdf_file = os.path.join(pkg_share, 'urdf/audibot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0', description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0', description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.0', description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('tf_freq', default_value='100.0', description='Rate at which to broadcast TF frames'),
        DeclareLaunchArgument('pub_tf', default_value='false', description='Whether to broadcast TF frames'),
        DeclareLaunchArgument('blue', default_value='false', description='Car is blue if true, orange if false'),
        DeclareLaunchArgument('robot_name', default_value='', description='Name and prefix for this robot'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_prefix': [LaunchConfiguration('robot_name'), '/'],
                'publish_frequency': LaunchConfiguration('tf_freq'),
                'robot_description': Command([f'xacro {urdf_file} pub_tf:=', LaunchConfiguration('pub_tf'), ' blue:=', LaunchConfiguration('blue'), ' robot_name:=', LaunchConfiguration('robot_name')])
            }]),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot_name'),
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('start_x'),
                '-y', LaunchConfiguration('start_y'),
                '-z', LaunchConfiguration('start_z'),
                '-Y', LaunchConfiguration('start_yaw')
            ],
            output='screen')
    ])
