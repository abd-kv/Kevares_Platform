#!/bin/bash
# ============================================================
# Run Multi-Camera Collision Avoidance Launch Script
# Press 'q' at any time to stop all processes gracefully.
# ============================================================

set -e

# --- SETTINGS ---
WORKSPACE=~/abd_ws_2
#LAUNCH_FILE=tests/multi_cam_collision.launch.py

# --- STEP 1: Go to workspace ---
cd "$WORKSPACE" || {
    echo "❌ Directory $WORKSPACE not found!"
    exit 1
}

# --- STEP 2: Source ROS 2 environment ---
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "⚠️ ROS 2 Humble not found at /opt/ros/humble"
fi

# --- STEP 3: Source workspace ---
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "❌ install/setup.bash not found — please run 'colcon build' first."
    exit 1
fi

# --- STEP 4: Launch ROS2 in the background ---
echo "🚀 Launching multi-camera collision avoidance system..."
ros2 launch tests multi_cam_collision.launch.py  &
ROS_PID=$!

# --- STEP 5: Wait for user input to quit ---
echo ""
echo "==========================================="
echo " Press 'q' then [ENTER] to stop all nodes. "
echo "==========================================="
echo ""

# Wait for 'q' input
while true; do
    read -n 1 -s key
    if [[ $key == "q" ]]; then
        echo ""
        echo "🛑 Stopping ROS processes..."
        # Kill the ROS launch process and all its children
        pkill -P $ROS_PID 2>/dev/null || true
        kill $ROS_PID 2>/dev/null || true
        wait $ROS_PID 2>/dev/null || true
        echo "✅ All processes stopped."
        break
    fi
done

