#!/bin/bash

# Log everything to a file
exec &> /home/user/ros2_startup.log

echo "Starting CAN setup" >> /home/user/ros2_startup.log
sudo ip link set can0 up type can bitrate 500000
echo "CAN setup done" >> /home/user/ros2_startup.log
echo "Sourcing ROS 2" >> /home/user/ros2_startup.log
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
echo "ROS 2 sourced" >> /home/user/ros2_startup.log

echo "Launching ROS 2 follow_master_launch" >> /home/user/ros2_startup.log
ros2 launch abd_package follow_master_launch.py &  # Run in the background

#echo "Launching ROS 2 data_capture_launch" >> /home/user/ros2_startup.log
#ros2 launch vaughan_pilot data_capture_launch.py &  # Run in the background

echo "Both ROS 2 launch files started" >> /home/user/ros2_startup.log

