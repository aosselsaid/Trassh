#!/usr/bin/env python3
"""
Main Bringup Launch File for Trash Bot
Launches all necessary nodes for autonomous operation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_trash_bot = get_package_share_directory('trash_bot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_trash_bot, 'maps', 'room_map.yaml'),
        description='Full path to map file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_trash_bot, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file for Nav2'
    )
    
    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trash_bot, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Serial Bridge Node
    serial_bridge_node = Node(
        package='trash_bot',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'wheel_separation': 0.21,
            'wheel_radius': 0.0325,
            'encoder_ticks_per_rev': 360
        }]
    )
    
    # Gesture Control Node
    gesture_control_node = Node(
        package='trash_bot',
        executable='gesture_control',
        name='gesture_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Mission Controller Node
    mission_controller_node = Node(
        package='trash_bot',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Nav2 Bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_trash_bot, 'config', 'trash_bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(serial_bridge_node)
    ld.add_action(gesture_control_node)
    ld.add_action(mission_controller_node)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_node)
    
    return ld
