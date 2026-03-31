#!/bin/bash
sudo ip link set can0 up type can bitrate 500000
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch abd_package start_zigzag_launch.py

