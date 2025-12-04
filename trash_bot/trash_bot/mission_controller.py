#!/usr/bin/env python3
"""
Mission Controller Node for Trash Bot
Main state machine controlling navigation between desks using Nav2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Publishers and Subscribers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.desk_request_sub = self.create_subscription(
            Int8, '/desk_request', self.desk_request_callback, 10
        )
        
        # Navigation setup
        self.navigator = BasicNavigator()
        
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')
        
        # State variables
        self.current_status = "READY"
        self.current_desk = None
        
        # Waypoint definitions (x, y, yaw)
        # These are placeholders - update after mapping
        self.waypoints = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'desk_1': {'x': 1.5, 'y': 1.0, 'yaw': 0.0},
            'desk_2': {'x': 1.5, 'y': -1.0, 'yaw': 0.0},
            'desk_3': {'x': 3.0, 'y': 0.0, 'yaw': 0.0}
        }
        
        # Waiting time at desk (seconds)
        self.desk_wait_time = 15.0
        
        # Timer to check navigation status
        self.nav_check_timer = self.create_timer(0.5, self.check_navigation_status)
        
        # Publish initial status
        self.publish_status("READY")
        
        self.get_logger().info('Mission Controller initialized')
        self.get_logger().info('Waypoints can be updated in the waypoints dictionary')
    
    def publish_status(self, status):
        """Publish robot status."""
        self.current_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')
    
    def desk_request_callback(self, msg):
        """Handle desk request from gesture recognition."""
        desk_id = msg.data
        
        # Validate desk ID
        if desk_id not in [1, 2, 3]:
            self.get_logger().warn(f'Invalid desk ID: {desk_id}')
            return
        
        # Check if already busy
        if self.current_status != "READY":
            self.get_logger().warn(f'Robot busy ({self.current_status}), ignoring request')
            return
        
        self.get_logger().info(f'Received request for Desk {desk_id}')
        self.current_desk = desk_id
        
        # Start navigation to desk
        self.navigate_to_desk(desk_id)
    
    def create_pose_stamped(self, waypoint_name):
        """Create a PoseStamped message from waypoint data."""
        waypoint = self.waypoints[waypoint_name]
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = waypoint['yaw']
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(self.yaw_to_quaternion(yaw)[2])
        pose.pose.orientation.w = float(self.yaw_to_quaternion(yaw)[3])
        
        return pose
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (x, y, z, w)."""
        import math
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    
    def navigate_to_desk(self, desk_id):
        """Navigate to specified desk."""
        desk_name = f'desk_{desk_id}'
        
        self.publish_status("NAVIGATING")
        
        # Create goal pose
        goal_pose = self.create_pose_stamped(desk_name)
        
        self.get_logger().info(f'Navigating to {desk_name}...')
        self.navigator.goToPose(goal_pose)
    
    def navigate_to_home(self):
        """Navigate back to home position."""
        self.publish_status("NAVIGATING")
        
        # Create home pose
        home_pose = self.create_pose_stamped('home')
        
        self.get_logger().info('Navigating to home...')
        self.navigator.goToPose(home_pose)
    
    def check_navigation_status(self):
        """Check navigation status and handle state transitions."""
        if self.current_status != "NAVIGATING":
            return
        
        if not self.navigator.isTaskComplete():
            return
        
        # Navigation task completed
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            
            if self.current_desk is not None:
                # Reached desk, start waiting
                self.publish_status("BUSY")
                self.get_logger().info(f'At Desk {self.current_desk}, waiting {self.desk_wait_time} seconds...')
                
                # Create a one-shot timer for waiting
                self.wait_timer = self.create_timer(
                    self.desk_wait_time, 
                    self.finish_desk_wait, 
                    callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
                )
            else:
                # Reached home
                self.publish_status("READY")
                self.get_logger().info('Back at home, ready for next request')
        
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation was canceled')
            self.publish_status("READY")
            self.current_desk = None
        
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed')
            self.publish_status("READY")
            self.current_desk = None
    
    def finish_desk_wait(self):
        """Called after waiting at desk is complete."""
        self.wait_timer.cancel()
        self.get_logger().info('Wait complete, returning to home')
        
        # Clear desk tracking
        self.current_desk = None
        
        # Navigate back to home
        self.navigate_to_home()
    
    def update_waypoint(self, waypoint_name, x, y, yaw):
        """Update a waypoint's coordinates."""
        if waypoint_name in self.waypoints:
            self.waypoints[waypoint_name] = {'x': x, 'y': y, 'yaw': yaw}
            self.get_logger().info(f'Updated {waypoint_name}: x={x}, y={y}, yaw={yaw}')
        else:
            self.get_logger().warn(f'Unknown waypoint: {waypoint_name}')
    
    def destroy_node(self):
        """Cleanup resources."""
        self.navigator.lifecycleShutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
