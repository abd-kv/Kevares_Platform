#!/bin/bash
cleanup() {
    echo "Cleaning up..."
    pkill -9 -f "/home/user/abd_ws_2/install/tests/lib/tests/"
    exit 0
}

trap cleanup SIGINT SIGTERM
# Log everything
exec &> /home/user/ros2_startup.log

echo "Starting CAN setup"
sudo ip link set can0 up type can bitrate 500000
echo "CAN setup done"

echo "Sourcing ROS 2"
source /opt/ros/humble/setup.bash
source /home/user/abd_ws_2/install/setup.bash
echo "ROS 2 sourced"

# Launch using setsid to create a new session
echo "Launching ROS 2 nodes..."
setsid ros2 launch tests start_robot_launch.py &
LAUNCH_PID=$!
echo "Started with PID $LAUNCH_PID"

# Launch WebSocket receiver
echo "Starting WebSocket receiver..."
setsid python3 /home/user/bobby/test/websocket_receiver.py &
PYTHON_PID=$!
echo "WebSocket receiver started with PID $PYTHON_PID"

# Wait for 'q' key press to terminate
echo "Press 'q' to shutdown nodes..."
while true; do
    read -n 1 -s key
    if [[ "$key" == "q" ]]; then
        break
    fi
done

# Kill all processes in the session created by setsid
echo "Shutting down..."

# Get all child PIDs of the launch process recursively
get_child_pids() {
    local pid=$1
    local children=$(pgrep -P $pid)
    for child in $children; do
        get_child_pids $child
    done
    echo "$children"
}

ALL_PIDS=""
for PID in $LAUNCH_PID $PYTHON_PID; do
    ALL_PIDS="$ALL_PIDS $PID $(get_child_pids $PID)"
done


# Send SIGINT to all
echo "Sending SIGINT to: $ALL_PIDS"
kill -SIGINT $ALL_PIDS
sleep 3

# Force kill any remaining
echo "Force killing any remaining..."
kill -9 $ALL_PIDS 2>/dev/null

echo "Shutdown complete."

