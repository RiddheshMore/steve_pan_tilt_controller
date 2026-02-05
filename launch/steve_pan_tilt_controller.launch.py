#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
        default_value='180.0',
        description='Goal position for pan motor in degrees (Legacy: 180 is center)'
    )

    tilt_goal_position_arg = DeclareLaunchArgument(
        'tilt_goal_position',
        default_value='180.0',
        description='Goal position for tilt motor in degrees (Legacy: 180 is center)'
    )
    
    # New Sweep/Trajectory arguments
    pan_goals_arg = DeclareLaunchArgument(
        'pan_goals',
        default_value='[0.0]',
        description='Pan angles in degrees (0=center). One value=fixed, two values=sweep. '
                    'Example sweep: [-60.0, 60.0]. MUST respect hardware limits (typically ±90°)'
    )
    
    tilt_goals_arg = DeclareLaunchArgument(
        'tilt_goals',
        default_value='[0.0]',
        description='Tilt angles in degrees (0=center). One value=fixed, two values=sweep. '
                    'Example sweep: [-30.0, 30.0]. MUST respect hardware limits (typically ±90°)'
    )
    
    sweep_speed_arg = DeclareLaunchArgument(
        'sweep_speed',
        default_value='15.0',
        description='Sweep speed in degrees/second'
    )

    
    log_feedback_arg = DeclareLaunchArgument(
        'log_feedback',
        default_value='true',
        description='Enable feedback logging'
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

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to use simulation mode'
    )

    # Create node
    pan_tilt_controller_node = Node(
        package='steve_pan_tilt_controller',
        executable='steve_pan_tilt_controller_node',
        name='steve_pan_tilt_controller',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'use_sim': LaunchConfiguration('use_sim'),
            'pan_goal_position': LaunchConfiguration('pan_goal_position'),
            'tilt_goal_position': LaunchConfiguration('tilt_goal_position'),
            'pan_goals': LaunchConfiguration('pan_goals'),
            'tilt_goals': LaunchConfiguration('tilt_goals'),
            'sweep_speed': LaunchConfiguration('sweep_speed'),
            'log_feedback': LaunchConfiguration('log_feedback'),
            'profile_velocity': LaunchConfiguration('profile_velocity'),
            'profile_acceleration': LaunchConfiguration('profile_acceleration'),
        }]
    )

    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui',
        default_value='false',
        description='Whether to launch the slider GUI'
    )

    # Create GUI node
    pan_tilt_gui_node = Node(
        package='steve_pan_tilt_controller',
        executable='pan_tilt_gui',
        name='pan_tilt_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gui'))
    )

    return LaunchDescription([
        pan_goal_position_arg,
        tilt_goal_position_arg,
        pan_goals_arg,
        tilt_goals_arg,
        sweep_speed_arg,
        log_feedback_arg,
        profile_velocity_arg,
        profile_acceleration_arg,
        use_sim_arg,
        launch_gui_arg,
        pan_tilt_controller_node,
        pan_tilt_gui_node,
    ])
