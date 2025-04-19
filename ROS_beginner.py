"""
ROS2 Toy Car Control System

This file provides information about the ROS2 system for controlling a toy car with a Raspberry Pi.
The system is implemented in the 'toy_car_control' package in this directory.

Docker Command Explanation:
The command "docker run -it --network=host -d -v=/dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --name humble -e DISPLAY=${DISPLAY} --restart=always ros:humble /bin/bash"
is used to run ROS2 Humble in a Docker container on a Raspberry Pi with the following features:
- --network=host: Uses the host's network, allowing ROS2 nodes to communicate with other machines
- -v=/dev:/dev: Mounts the /dev directory, giving access to GPIO pins for motor control
- -v /tmp/.X11-unix:/tmp/.X11-unix and -e DISPLAY=${DISPLAY}: Enables GUI applications if needed
- --restart=always: Automatically restarts the container if it crashes or after reboot
- ros:humble: Uses the official ROS2 Humble Docker image

Usage Instructions:
1. Navigate to the toy_car_control package directory
2. Follow the instructions in the README.md file to set up and run the system

The system consists of two main nodes:
1. Publisher Node: Sends movement commands based on keyboard input
2. Controller Node: Receives commands and controls the motors via GPIO pins

For more details, see the README.md file in the toy_car_control directory.
"""

# Example of how to run the system
if __name__ == '__main__':
    print("This is an information file. To run the system, use the following commands:")
    print("1. Build the package:")
    print("   cd ~/ros2_ws")
    print("   colcon build --symlink-install --packages-select toy_car_control")
    print("2. Source the workspace:")
    print("   source ~/ros2_ws/install/setup.bash")
    print("3. Launch the system:")
    print("   ros2 launch toy_car_control toy_car.launch.py")