#!/bin/bash
# ========================================================
# Courier Robot - Auto Build & Launch Script
# ========================================================

echo "============================================"
echo "  Courier Robot - Build & Full Mission Launch"
echo "============================================"
echo

# Navigate to workspace
cd ~/ros2_ws || { echo "[ERROR] ~/ros2_ws not found!"; exit 1; }

# ------------------------------
# Build ROS2 workspace if necessary
# ------------------------------
if [ ! -f "install/setup.bash" ]; then
    echo "[INFO] Building ROS2 workspace (colcon build)..."
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo "[ERROR] colcon build failed!"
        exit 1
    fi
else
    echo "[INFO] Workspace already built. Skipping build."
fi

# ------------------------------
# Source workspace
# ------------------------------
echo "[INFO] Sourcing ROS2 workspace..."
source install/setup.bash

# ------------------------------
# Launch Gazebo world + robot
# ------------------------------
echo "[INFO] Launching Gazebo world + robot + obstacles..."
ros2 launch courier_world world.launch.py &
GAZEBO_PID=$!
sleep 5  # wait for Gazebo to start

# ------------------------------
# Launch Nav2
# ------------------------------
echo "[INFO] Launching Nav2 stack..."
ros2 launch courier_navigation nav2.launch.py &
NAV2_PID=$!
sleep 5  # wait for Nav2 to initialize

# ------------------------------
# Launch BFS Planner
# ------------------------------
echo "[INFO] Launching BFS planner node..."
ros2 run courier_bfs_planner bfs_planner_node &
BFS_PID=$!
sleep 2

# ------------------------------
# Launch Behavior Tree mission
# ------------------------------
echo "[INFO] Launching Behavior Tree node..."
ros2 run courier_bt courier_bt_node &
BT_PID=$!

# ------------------------------
# Wait for all background processes
# ------------------------------
echo "[INFO] All nodes launched. Waiting for termination..."
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID
