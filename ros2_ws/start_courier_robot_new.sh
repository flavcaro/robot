#!/bin/bash
# Courier Robot - Auto Build & Launch Script

echo "============================================"
echo "  Courier Robot - Build & Full Mission Launch"
echo "============================================"
echo

cd ~/ros2_ws || { echo "[ERROR] ~/ros2_ws not found!"; exit 1; }

# Build ROS2 workspace
echo "[INFO] Building ROS2 workspace..."
if [ -d "build" ] && [ -d "install" ]; then
    colcon build --symlink-install --merge-install
else
    unset CMAKE_PREFIX_PATH
    colcon build --symlink-install --merge-install
fi
[ $? -ne 0 ] && { echo "[ERROR] Build failed!"; exit 1; }

# Source workspace
echo "[INFO] Sourcing workspace..."
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH="$HOME/ros2_ws/install:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$HOME/ros2_ws/install:$CMAKE_PREFIX_PATH"

# Verify packages
echo "[INFO] Verifying packages..."
for pkg in courier_world courier_navigation courier_bfs_planner courier_bt; do
    if ! ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo "[ERROR] Package $pkg not found!"
        exit 1
    fi
done
echo "[INFO] All packages verified!"

# Launch nodes
echo "[INFO] Launching Gazebo..."
bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export AMENT_PREFIX_PATH="$HOME/ros2_ws/install:$AMENT_PREFIX_PATH" && ros2 launch courier_world world.launch.py' &
GAZEBO_PID=$!
sleep 5

echo "[INFO] Launching Nav2..."
bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export AMENT_PREFIX_PATH="$HOME/ros2_ws/install:$AMENT_PREFIX_PATH" && ros2 launch courier_navigation nav2_bringup_launch.py' &
NAV2_PID=$!
sleep 5

echo "[INFO] Launching BFS planner..."
bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export AMENT_PREFIX_PATH="$HOME/ros2_ws/install:$AMENT_PREFIX_PATH" && ros2 run courier_bfs_planner bfs_node' &
BFS_PID=$!
sleep 2

echo "[INFO] Launching Behavior Tree..."
bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export AMENT_PREFIX_PATH="$HOME/ros2_ws/install:$AMENT_PREFIX_PATH" && ros2 run courier_bt courier_bt_node' &
BT_PID=$!

echo "[INFO] All nodes launched. Press Ctrl+C to stop."
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID
