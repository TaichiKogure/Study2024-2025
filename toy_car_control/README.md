# Toy Car Control ROS2 Package

A simple ROS2 package for controlling a toy car with a Raspberry Pi.

## Overview

This package provides a ROS2-based control system for a toy car using a Raspberry Pi. It consists of two main nodes:

1. **Publisher Node**: Sends movement commands based on keyboard input
2. **Controller Node**: Receives commands and controls the motors via GPIO pins

## Hardware Requirements

- Raspberry Pi (any model with GPIO pins)
- Motor driver (e.g., L298N)
- DC motors (2x for differential drive)
- Power supply for motors
- Toy car chassis

## Wiring

Connect your motor driver to the Raspberry Pi GPIO pins as follows (adjust pin numbers in `subscriber_node.py` if needed):

- Left Motor Forward: GPIO 17
- Left Motor Backward: GPIO 18
- Right Motor Forward: GPIO 22
- Right Motor Backward: GPIO 23

## Software Setup

### 1. Install ROS2 Humble on Raspberry Pi

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base

# Install additional dependencies
sudo apt install -y python3-pip
pip3 install RPi.GPIO
```

### 2. Install this package

```bash
# Create a ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy this package to the workspace
cp -r /path/to/toy_car_control .

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Running the System

### Option 1: Run both nodes on the Raspberry Pi

```bash
# Source ROS2 and the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch both nodes
ros2 launch toy_car_control toy_car.launch.py
```

### Option 2: Run publisher on PC and controller on Raspberry Pi

On the Raspberry Pi:
```bash
# Source ROS2 and the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run only the controller node
ros2 run toy_car_control subscriber
```

On your PC:
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set ROS domain ID to match the Raspberry Pi
export ROS_DOMAIN_ID=0

# Run the publisher node
ros2 run toy_car_control publisher
```

## Using Docker (Optional)

You can also run ROS2 in a Docker container on the Raspberry Pi:

```bash
# Pull the ROS2 Humble image
docker pull ros:humble

# Run the container with access to GPIO
docker run -it --network=host -d -v=/dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --name humble -e DISPLAY=${DISPLAY} --restart=always ros:humble /bin/bash
```

## Controlling the Car

Use the following keyboard controls:
- W: Move forward
- S: Stop
- X: Move backward
- A: Turn left
- D: Turn right
- Q: Quit

## Customization

You can adjust the following parameters in the code:
- GPIO pin numbers in `subscriber_node.py`
- Motor speeds in `publisher_node.py`
- PWM frequency in `subscriber_node.py`

## Troubleshooting

- If the motors don't move, check your wiring and GPIO pin configuration
- Ensure the motor driver is properly powered
- Check ROS2 topic communication with `ros2 topic echo /cmd_vel`
- Verify GPIO permissions on the Raspberry Pi