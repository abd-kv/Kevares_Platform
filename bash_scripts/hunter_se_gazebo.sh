#!/bin/bash

# Navigate to the workspace
cd ~/hunter_ws || exit

# Source the ROS 2 setup
source install/setup.bash

# Launch the robot in Gazebo
echo "Launching robot in Gazebo..."
ros2 launch hunter_gazebo spawn_robot_ros2.launch.xml &

# Wait for a few seconds to ensure the first launch completes
sleep 2

# Publish URDF to ROS 2
echo "Publishing URDF..."
ros2 launch hunter_description publish_urdf.launch.py &

# Wait for a few seconds to ensure the URDF is published
sleep 2

# Start the Gazebo world
echo "Starting Gazebo world..."
ros2 launch hunter_gazebo start_world.launch.py &

# Keep the script running to maintain ROS 2 launches
wait

