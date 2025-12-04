#!/usr/bin/env python3
"""
Serial Bridge Node for Trash Bot
Communicates with ESP32 for motor control and odometry.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import time
from threading import Lock


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.21)  # meters
        self.declare_parameter('wheel_radius', 0.0325)    # meters
        self.declare_parameter('encoder_ticks_per_rev', 360)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        self.init_serial()
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Encoder tracking
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Timer for reading from serial
        self.serial_timer = self.create_timer(0.02, self.read_serial)  # 50 Hz
        
        self.get_logger().info('Serial Bridge Node initialized')
    
    def init_serial(self):
        """Initialize serial connection to ESP32."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial connection established on {self.serial_port}')
            time.sleep(2)  # Wait for ESP32 to reset
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_conn = None
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist command to wheel velocities and send to ESP32.
        
        Message format to ESP32:
        - Start byte: 0xFF
        - Command type: 0x01 (velocity command)
        - Left wheel speed: float (4 bytes)
        - Right wheel speed: float (4 bytes)
        - Checksum: uint8 (1 byte)
        """
        linear = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s
        
        # Convert to wheel velocities using differential drive kinematics
        left_vel = linear - (angular * self.wheel_separation / 2.0)
        right_vel = linear + (angular * self.wheel_separation / 2.0)
        
        # Send to ESP32
        self.send_velocity_command(left_vel, right_vel)
    
    def send_velocity_command(self, left_vel, right_vel):
        """Send velocity command to ESP32."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            with self.serial_lock:
                # Build message
                message = bytearray()
                message.append(0xFF)  # Start byte
                message.append(0x01)  # Command type: velocity
                
                # Pack velocities as floats
                message.extend(struct.pack('<f', left_vel))
                message.extend(struct.pack('<f', right_vel))
                
                # Calculate checksum (simple sum mod 256)
                checksum = sum(message[1:]) % 256
                message.append(checksum)
                
                # Send message
                self.serial_conn.write(message)
                
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_serial(self):
        """
        Read encoder data from ESP32 and update odometry.
        
        Expected message format from ESP32:
        - Start byte: 0xFF
        - Message type: 0x02 (encoder data)
        - Left encoder ticks: int32 (4 bytes)
        - Right encoder ticks: int32 (4 bytes)
        - Checksum: uint8 (1 byte)
        """
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            with self.serial_lock:
                # Check if data is available
                if self.serial_conn.in_waiting >= 11:  # Full message size
                    # Look for start byte
                    byte = self.serial_conn.read(1)
                    if byte[0] == 0xFF:
                        # Read message type
                        msg_type = self.serial_conn.read(1)[0]
                        
                        if msg_type == 0x02:  # Encoder data
                            # Read encoder ticks
                            data = self.serial_conn.read(9)  # 4 + 4 + 1 bytes
                            
                            left_ticks = struct.unpack('<i', data[0:4])[0]
                            right_ticks = struct.unpack('<i', data[4:8])[0]
                            checksum = data[8]
                            
                            # Verify checksum
                            expected_checksum = (msg_type + sum(data[0:8])) % 256
                            
                            if checksum == expected_checksum:
                                # Update odometry
                                self.update_odometry(left_ticks, right_ticks)
                            else:
                                self.get_logger().warn('Checksum mismatch')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
        except struct.error as e:
            self.get_logger().error(f'Data unpacking error: {e}')
    
    def update_odometry(self, left_ticks, right_ticks):
        """
        Update odometry based on encoder ticks using differential drive model.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate change in ticks
        d_left_ticks = left_ticks - self.last_left_ticks
        d_right_ticks = right_ticks - self.last_right_ticks
        
        # Convert ticks to distance
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = d_left_ticks * meters_per_tick
        d_right = d_right_ticks * meters_per_tick
        
        # Calculate displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation
        
        # Update pose
        if abs(d_theta) < 1e-6:
            # Straight line motion
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            # Arc motion
            radius = d_center / d_theta
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y += radius * (-math.cos(self.theta + d_theta) + math.cos(self.theta))
        
        self.theta += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        v_x = d_center / dt
        v_theta = d_theta / dt
        
        # Update tracking variables
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time
        
        # Publish odometry
        self.publish_odometry(current_time, v_x, v_theta)
    
    def publish_odometry(self, current_time, v_x, v_theta):
        """Publish odometry message and transform."""
        # Create quaternion from yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = v_theta
        
        # Set covariance (placeholder values)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # theta
        
        odom.twist.covariance[0] = 0.01  # v_x
        odom.twist.covariance[35] = 0.01 # v_theta
        
        self.odom_pub.publish(odom)
    
    def destroy_node(self):
        """Cleanup resources."""
        if self.serial_conn is not None and self.serial_conn.is_open:
            # Stop motors
            self.send_velocity_command(0.0, 0.0)
            time.sleep(0.1)
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
