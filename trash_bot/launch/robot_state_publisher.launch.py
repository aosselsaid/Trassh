#!/usr/bin/env python3
"""
Robot State Publisher Launch File
Publishes robot description to /robot_description topic and TF tree.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('trash_bot')
    
    # Path to SDF file
    sdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'trash_bot.sdf'])
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot State Publisher
    # Note: robot_state_publisher typically expects URDF. 
    # If using SDF, we might need to convert it or use a tool that supports SDF.
    # However, for Gazebo simulation, SDF is native.
    # For ROS 2 tools (RViz, Nav2), URDF is still the standard.
    # If you strictly want to use SDF for everything, you might face compatibility issues with standard ROS tools.
    # But assuming you want to use SDF for Gazebo and keep URDF for ROS tools, or try to load SDF content:
    
    with open(os.path.join(get_package_share_directory('trash_bot'), 'urdf', 'trash_bot.sdf'), 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Joint State Publisher (for manual testing)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    
    return ld
