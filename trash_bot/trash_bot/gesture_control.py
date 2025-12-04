#!/usr/bin/env python3
"""
Gesture Control Node for Trash Bot
Detects hand gestures (1, 2, or 3 fingers) using MediaPipe and publishes desk requests.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import cv2
import mediapipe as mp
import time


class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control')
        
        # Publishers and Subscribers
        self.desk_request_pub = self.create_publisher(Int8, '/desk_request', 10)
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10
        )
        
        # State variables
        self.robot_status = "READY"
        self.camera_active = True
        
        # Gesture detection parameters
        self.gesture_threshold = 1.0  # Seconds to hold gesture
        self.last_gesture = None
        self.gesture_start_time = None
        self.last_published_gesture = None
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Camera setup
        self.cap = None
        self.init_camera()
        
        # Timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 Hz
        
        self.get_logger().info('Gesture Control Node initialized')
    
    def init_camera(self):
        """Initialize the camera."""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera initialized successfully')
    
    def status_callback(self, msg):
        """Handle robot status updates."""
        self.robot_status = msg.data
        
        # Turn off camera processing when robot is busy
        if self.robot_status in ["NAVIGATING", "BUSY"]:
            if self.camera_active:
                self.get_logger().info('Robot busy, pausing gesture detection')
                self.camera_active = False
        else:
            if not self.camera_active:
                self.get_logger().info('Robot ready, resuming gesture detection')
                self.camera_active = True
                # Reset gesture tracking
                self.last_gesture = None
                self.gesture_start_time = None
    
    def count_fingers(self, hand_landmarks):
        """
        Count the number of extended fingers.
        Returns: Number of fingers (0-5)
        """
        if hand_landmarks is None:
            return 0
        
        # Finger tip and PIP landmark indices
        finger_tips = [
            self.mp_hands.HandLandmark.THUMB_TIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]
        
        finger_pips = [
            self.mp_hands.HandLandmark.THUMB_IP,
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]
        
        finger_count = 0
        
        # Check each finger
        for i in range(5):
            tip = hand_landmarks.landmark[finger_tips[i]]
            pip = hand_landmarks.landmark[finger_pips[i]]
            
            # For thumb, check x-coordinate (special case)
            if i == 0:
                # Get wrist and thumb MCP for reference
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC]
                
                # Check if thumb is extended (away from palm)
                if abs(tip.x - wrist.x) > abs(mcp.x - wrist.x):
                    finger_count += 1
            else:
                # For other fingers, check if tip is above PIP (extended)
                if tip.y < pip.y:
                    finger_count += 1
        
        return finger_count
    
    def process_frame(self):
        """Process camera frames and detect gestures."""
        if not self.camera_active or self.cap is None:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Flip frame for mirror effect
        frame = cv2.flip(frame, 1)
        
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        current_gesture = None
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks on frame
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                
                # Count fingers
                finger_count = self.count_fingers(hand_landmarks)
                
                # Only interested in 1, 2, or 3 fingers
                if finger_count in [1, 2, 3]:
                    current_gesture = finger_count
                
                # Display finger count on frame
                cv2.putText(
                    frame, f'Fingers: {finger_count}', 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 0), 2
                )
        
        # Display robot status on frame
        status_color = (0, 255, 0) if self.robot_status == "READY" else (0, 165, 255)
        cv2.putText(
            frame, f'Status: {self.robot_status}', 
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, status_color, 2
        )
        
        # Gesture debouncing logic
        current_time = time.time()
        
        if current_gesture is not None:
            if current_gesture == self.last_gesture:
                # Same gesture detected, check duration
                if self.gesture_start_time is None:
                    self.gesture_start_time = current_time
                else:
                    duration = current_time - self.gesture_start_time
                    
                    # Display progress bar
                    progress = min(duration / self.gesture_threshold, 1.0)
                    bar_length = int(progress * 200)
                    cv2.rectangle(frame, (10, 90), (10 + bar_length, 110), (0, 255, 0), -1)
                    cv2.rectangle(frame, (10, 90), (210, 110), (255, 255, 255), 2)
                    
                    if duration >= self.gesture_threshold:
                        # Gesture held long enough
                        if current_gesture != self.last_published_gesture:
                            self.publish_desk_request(current_gesture)
                            self.last_published_gesture = current_gesture
                        # Reset tracking
                        self.gesture_start_time = None
            else:
                # Different gesture detected, reset timer
                self.last_gesture = current_gesture
                self.gesture_start_time = current_time
        else:
            # No valid gesture detected, reset
            self.last_gesture = None
            self.gesture_start_time = None
        
        # Show the frame
        cv2.imshow('Trash Bot Gesture Control', frame)
        cv2.waitKey(1)
    
    def publish_desk_request(self, desk_id):
        """Publish a desk request."""
        msg = Int8()
        msg.data = desk_id
        self.desk_request_pub.publish(msg)
        self.get_logger().info(f'Published desk request: {desk_id}')
    
    def destroy_node(self):
        """Cleanup resources."""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.hands is not None:
            self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
