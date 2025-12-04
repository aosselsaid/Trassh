#!/usr/bin/env python3
"""
SLAM Launch File for Trash Bot
Use this to create the map during Phase 1.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directories
    pkg_trash_bot = get_package_share_directory('trash_bot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_trash_bot, 'config', 'slam_params.yaml'),
        description='Full path to the ROS 2 parameters file for SLAM'
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
    
    # Lidar Node (RPLidar)
    # Note: If Lidar stops when RViz starts, it might be a USB bandwidth/power issue.
    # Try changing the serial port if needed (e.g. /dev/ttyUSB0 vs /dev/ttyUSB1)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen',
        respawn=True
    )
    
    # SLAM Toolbox (Online Async)
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file
        }.items()
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_trash_bot, 'config', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # Delay RViz to ensure Lidar and SLAM are ready
    delayed_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(serial_bridge_node)
    ld.add_action(lidar_node)
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(delayed_rviz_node)
    
    return ld
