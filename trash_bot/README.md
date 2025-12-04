# Trash Bot - Autonomous Trash Bin Robot

ROS 2 Jazzy package for an autonomous mobile trash bin robot with gesture-based control and Navigation 2 integration.

## Project Overview

The **trash_bot** is an autonomous trash bin robot designed to navigate between desk locations in a room based on hand gesture commands. The robot uses:

- **Hardware**: Raspberry Pi 5 (main computer), ESP32 (motor controller), Lidar sensor, USB camera
- **Software**: ROS 2 Jazzy Jalisco, Navigation 2, OpenCV, MediaPipe
- **Sensors**: RPLidar (or similar), USB camera for gesture recognition
- **Actuators**: Differential drive with 2 DC motors

## System Architecture

### Phase 1: Setup & Mapping
1. Map the room using SLAM
2. Record waypoint coordinates (Desk 1, Desk 2, Desk 3, Home)

### Phase 2: Autonomous Operation
1. Robot starts at Home position
2. Camera detects hand gestures (1, 2, or 3 fingers)
3. Navigate to corresponding desk
4. Wait 15 seconds at desk
5. Return to Home
6. Repeat

## File Structure

```
trash_bot/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── nav2_params.yaml       # Navigation 2 configuration
│   └── slam_params.yaml        # SLAM Toolbox configuration
├── firmware/
│   ├── platformio.ini          # PlatformIO configuration
│   └── src/
│       └── main.cpp            # ESP32 firmware (PID, encoders, serial)
├── launch/
│   ├── bringup_launch.py       # Main launch file
│   ├── robot_state_publisher.launch.py
│   └── slam_launch.py          # SLAM for mapping
├── maps/
│   └── room_map.yaml           # Generated map (after SLAM)
├── scripts/
│   ├── gesture_control.py      # Hand gesture recognition node
│   ├── mission_controller.py   # Navigation state machine
│   └── serial_bridge.py        # ESP32 communication & odometry
├── trash_bot/
│   └── __init__.py
└── urdf/
    └── trash_bot.urdf.xacro    # Robot description
```

## Prerequisites

### Software Dependencies

```bash
# ROS 2 Jazzy (Ubuntu 24.04)
sudo apt update
sudo apt install ros-jazzy-desktop

# Navigation 2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# Robot State Publisher
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher

# Python dependencies
sudo apt install python3-pip
pip3 install opencv-python mediapipe pyserial numpy

# Development tools
sudo apt install python3-colcon-common-extensions
```

### Hardware Setup

1. **Raspberry Pi 5**: Install Ubuntu 24.04 and ROS 2 Jazzy
2. **ESP32**: Flash firmware using PlatformIO
3. **Lidar**: Connect to Raspberry Pi via USB
4. **Camera**: USB camera connected to Raspberry Pi
5. **Serial Connection**: ESP32 connected to Pi via USB (typically `/dev/ttyUSB0`)

## Building the Package

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/trash_bot_ws/src
cd ~/trash_bot_ws/src

# Clone or copy the trash_bot package here
# (Assuming package is already in workspace)

# Build
cd ~/trash_bot_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Flash ESP32 Firmware

```bash
cd ~/trash_bot_ws/src/trash_bot/firmware

# Using PlatformIO CLI
pio run --target upload

# Or using PlatformIO IDE in VS Code
```

### 2. Create Map (Phase 1)

```bash
# Terminal 1: Launch SLAM
ros2 launch trash_bot slam_launch.py

# Terminal 2: Teleoperate the robot to explore the room
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# After mapping, save the map
ros2 run nav2_map_server map_saver_cli -f ~/trash_bot_ws/src/trash_bot/maps/room_map
```

### 3. Record Waypoints

After creating the map, manually drive the robot to each desk location and record coordinates:

1. Drive to Desk 1, note pose from `/odom` or RViz
2. Repeat for Desk 2, Desk 3, and Home
3. Update waypoints in `scripts/mission_controller.py`:

```python
self.waypoints = {
    'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'desk_1': {'x': 1.5, 'y': 1.0, 'yaw': 0.0},
    'desk_2': {'x': 1.5, 'y': -1.0, 'yaw': 0.0},
    'desk_3': {'x': 3.0, 'y': 0.0, 'yaw': 0.0}
}
```

### 4. Run Autonomous Mode (Phase 2)

```bash
# Launch the complete system
ros2 launch trash_bot bringup_launch.py

# The robot will now:
# - Wait for gesture commands
# - Navigate to desks based on finger gestures
# - Return home after each mission
```

## Gesture Commands

- **1 Finger**: Navigate to Desk 1
- **2 Fingers**: Navigate to Desk 2
- **3 Fingers**: Navigate to Desk 3

Hold the gesture steady for 1 second to trigger navigation.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to robot |
| `/odom` | `nav_msgs/Odometry` | Odometry from wheel encoders |
| `/scan` | `sensor_msgs/LaserScan` | Lidar scan data |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/desk_request` | `std_msgs/Int8` | Desk ID from gesture |
| `/robot_status` | `std_msgs/String` | Robot state (READY/NAVIGATING/BUSY) |

## Nodes

### gesture_control
- Detects hand gestures using MediaPipe
- Publishes desk requests
- Pauses when robot is busy

### mission_controller
- Manages navigation state machine
- Uses Nav2 BasicNavigator
- Coordinates waypoint navigation

### serial_bridge
- Communicates with ESP32
- Converts `/cmd_vel` to wheel velocities
- Publishes odometry from encoders

## Configuration

### Robot Parameters
- Wheel separation: 0.21m
- Wheel radius: 0.0325m
- Encoder ticks per revolution: 360
- Max linear velocity: 0.3 m/s
- Max angular velocity: 1.0 rad/s

### ESP32 Pin Configuration

**Left Motor:**
- PWM: GPIO 25
- DIR1: GPIO 26
- DIR2: GPIO 27
- Encoder A: GPIO 32
- Encoder B: GPIO 33

**Right Motor:**
- PWM: GPIO 12
- DIR1: GPIO 13
- DIR2: GPIO 14
- Encoder A: GPIO 34
- Encoder B: GPIO 35

## Troubleshooting

### Serial Connection Issues
```bash
# Check if ESP32 is detected
ls /dev/ttyUSB*

# Give permission to serial port
sudo chmod 666 /dev/ttyUSB0
```

### Camera Not Detected
```bash
# List video devices
v4l2-ctl --list-devices

# Test camera
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
```

### Navigation Issues
- Ensure map is properly loaded
- Check that waypoints are within the mapped area
- Verify lidar is publishing on `/scan`
- Check transforms: `ros2 run tf2_tools view_frames`

## Development

### Adjusting PID Values (ESP32)
Edit `firmware/src/main.cpp`:
```cpp
PIDController left_pid(50.0, 20.0, 5.0, -255.0, 255.0);  // kp, ki, kd
```

### Changing Navigation Parameters
Edit `config/nav2_params.yaml` to tune:
- Costmap inflation radius
- DWB local planner parameters
- Goal tolerances

## License

Apache 2.0

## Author

Robotics Team

## Support

For issues and questions, please refer to:
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Navigation 2 Documentation](https://navigation.ros.org/)
- [MediaPipe Hand Tracking](https://google.github.io/mediapipe/solutions/hands.html)
