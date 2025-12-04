# Build Instructions for Trash Bot

## Quick Start Guide

### 1. System Setup (Raspberry Pi 5 - Ubuntu 24.04)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install dependencies
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    python3-pip \
    python3-colcon-common-extensions

# Install Python packages
pip3 install opencv-python mediapipe pyserial numpy
```

### 2. Build the ROS 2 Package

```bash
# Create workspace
mkdir -p ~/trash_bot_ws/src
cd ~/trash_bot_ws/src

# Copy trash_bot package to src directory
# (Assuming package is already here)

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build
cd ~/trash_bot_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/trash_bot_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Flash ESP32 Firmware

```bash
# Install PlatformIO
pip3 install platformio

# Navigate to firmware directory
cd ~/trash_bot_ws/src/trash_bot/firmware

# Build and upload
pio run --target upload

# Grant serial port access (if needed)
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

### 4. Hardware Connections

#### ESP32 to Motor Driver
Connect according to pin definitions in firmware/README.md

#### Raspberry Pi Connections
- **Lidar**: USB port
- **Camera**: USB port  
- **ESP32**: USB port (will appear as `/dev/ttyUSB0` or `/dev/ttyACM0`)

#### Power
- ESP32: USB power from Raspberry Pi
- Motors: External power supply (7-12V) connected to motor driver
- Raspberry Pi: Dedicated power supply (5V, 3A minimum)

### 5. Verify Installation

```bash
# Check if all nodes are available
ros2 pkg executables trash_bot

# Should show:
# trash_bot gesture_control
# trash_bot mission_controller
# trash_bot serial_bridge

# Check URDF
ros2 launch trash_bot robot_state_publisher.launch.py

# In another terminal, check topics
ros2 topic list

# View robot in RViz
ros2 run rviz2 rviz2
```

## Phase 1: Mapping

```bash
# 1. Launch SLAM
ros2 launch trash_bot slam_launch.py

# 2. In another terminal, teleoperate the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. Drive around the room to create a complete map
# 4. When satisfied with the map, save it:
cd ~/trash_bot_ws/src/trash_bot/maps
ros2 run nav2_map_server map_saver_cli -f room_map

# 5. This creates room_map.yaml and room_map.pgm
```

### Recording Waypoints

```bash
# 1. While SLAM is running, drive to Home position
# 2. In another terminal:
ros2 topic echo /odom

# 3. Note down the x, y coordinates and orientation
# 4. Repeat for Desk 1, Desk 2, and Desk 3
# 5. Update mission_controller.py with these coordinates
```

## Phase 2: Autonomous Operation

```bash
# Launch the complete system
ros2 launch trash_bot bringup_launch.py

# Optional: Launch without RViz (for better performance)
ros2 launch trash_bot bringup_launch.py use_rviz:=false

# Optional: Use saved parameters
ros2 launch trash_bot bringup_launch.py params_file:=/path/to/custom_params.yaml
```

## Testing Individual Components

### Test Serial Bridge Only
```bash
ros2 run trash_bot serial_bridge --ros-args -p serial_port:=/dev/ttyUSB0
```

### Test Gesture Control Only
```bash
ros2 run trash_bot gesture_control
```

### Test Mission Controller Only
```bash
# Requires Nav2 to be running
ros2 run trash_bot mission_controller
```

### Send Manual Desk Request
```bash
ros2 topic pub /desk_request std_msgs/Int8 "data: 1" --once
```

## Troubleshooting

### Issue: Serial port permission denied
```bash
sudo chmod 666 /dev/ttyUSB0
# Or permanently:
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Issue: Camera not found
```bash
# List cameras
v4l2-ctl --list-devices

# Test camera
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
```

### Issue: Nav2 not starting
```bash
# Check if map file exists
ls ~/trash_bot_ws/src/trash_bot/maps/

# Verify map file format
cat ~/trash_bot_ws/src/trash_bot/maps/room_map.yaml
```

### Issue: Transform errors
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# This creates frames.pdf showing the TF tree

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

### Issue: Robot not moving
```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Check if ESP32 is receiving commands (via Serial Monitor)
pio device monitor

# Test motors directly
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## Performance Optimization

### For Real-time Performance on Raspberry Pi:
```bash
# Reduce map update frequency in nav2_params.yaml
# Disable unnecessary visualizations
# Run without RViz on the robot (use remote RViz on another PC)
```

### Remote Visualization:
On your laptop (with ROS 2 installed):
```bash
export ROS_DOMAIN_ID=0  # Same as robot
ros2 run rviz2 rviz2
```

## Next Steps

1. **Calibrate Odometry**: Drive the robot in a square and measure drift
2. **Tune PID**: Adjust ESP32 PID values for smooth motion
3. **Tune Nav2**: Adjust costmap and planner parameters
4. **Add Safety Features**: Emergency stop, battery monitoring, etc.

## Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html)
