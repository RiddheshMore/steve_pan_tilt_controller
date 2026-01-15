#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('steve_pan_tilt_controller')
    config_file = os.path.join(pkg_share, 'config', 'dynamixel_motors.yaml')

    # Declare launch arguments
    pan_goal_position_arg = DeclareLaunchArgument(
        'pan_goal_position',
        default_value='75.0',
        description='Goal position for pan motor in degrees'
    )

    tilt_goal_position_arg = DeclareLaunchArgument(
        'tilt_goal_position',
        default_value='180.0',
        description='Goal position for tilt motor in degrees'
    )

    profile_velocity_arg = DeclareLaunchArgument(
        'profile_velocity',
        default_value='50',
        description='Profile velocity for smooth movement (rev/min, lower=slower)'
    )

    profile_acceleration_arg = DeclareLaunchArgument(
        'profile_acceleration',
        default_value='10',
        description='Profile acceleration for smooth ramp (rev/min^2, lower=smoother)'
    )

    # Create node
    pan_tilt_controller_node = Node(
        package='steve_pan_tilt_controller',
        executable='steve_pan_tilt_controller_node',
        name='steve_pan_tilt_controller',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'pan_goal_position': LaunchConfiguration('pan_goal_position'),
            'tilt_goal_position': LaunchConfiguration('tilt_goal_position'),
            'profile_velocity': LaunchConfiguration('profile_velocity'),
            'profile_acceleration': LaunchConfiguration('profile_acceleration'),
        }]
    )

    return LaunchDescription([
        pan_goal_position_arg,
        tilt_goal_position_arg,
        profile_velocity_arg,
        profile_acceleration_arg,
        pan_tilt_controller_node,
    ])
