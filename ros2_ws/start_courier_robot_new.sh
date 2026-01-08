#!/bin/bash
# ========================================================
# Courier Robot - Auto Build & Launch Script for ROS2 Jazzy
# ========================================================

echo "============================================"
echo "  Courier Robot - Full Build & Mission Launch"
echo "============================================"
echo

# Navigate to workspace
cd ~/ros2_ws || { echo "[ERROR] ~/ros2_ws not found!"; exit 1; }

# Check Python dependencies
echo "📦 Checking Python dependencies..."
for pkg in py_trees cv2; do
    python3 -c "import $pkg" 2>/dev/null || {
        echo "⚠️  $pkg not found. Installing..."
        if [ "$pkg" == "cv2" ]; then
            pip install --break-system-packages "numpy>=1.21.6,<1.28.0" opencv-python
        else
            pip install --break-system-packages "$pkg>=2.2.0"
        fi
    }
done

# Generate SDF from URDF
URDF_FILE="src/courier_robot_description/urdf/courier_robot.urdf.xacro"
SDF_FILE="src/courier_robot_description/urdf/courier_robot.sdf"

if [ ! -f "$URDF_FILE" ]; then
    echo "❌ URDF not found: $URDF_FILE"
    exit 1
fi

echo "🔧 Generating SDF from URDF..."
ros2 run xacro xacro "$URDF_FILE" > "$SDF_FILE"
echo "✅ SDF generated: $SDF_FILE"

# Clean previous build/install/log
echo "🧹 Cleaning build/install/log..."
rm -rf build install log

# Source ROS2 Jazzy underlay
echo "📡 Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Build workspace
echo "🔨 Building ROS2 workspace..."
colcon build --symlink-install
[ $? -ne 0 ] && { echo "[ERROR] colcon build failed!"; exit 1; }

# Source workspace AFTER build
source ~/ros2_ws/install/setup.bash

# Verify packages with better error handling (avoid broken pipe)
echo "🔍 Verifying packages..."
FAILED=0
PACKAGE_LIST=$(ros2 pkg list 2>/dev/null)
for pkg in courier_world courier_navigation courier_bfs_planner courier_bt courier_tag_localization courier_robot_description; do
    if echo "$PACKAGE_LIST" | grep -q "^$pkg$"; then
        echo "  ✅ $pkg"
    else
        echo "  ❌ $pkg"
        FAILED=1
    fi
done

if [ $FAILED -eq 1 ]; then
    echo "[ERROR] Some packages not found!"
    exit 1
fi
echo "✅ All packages verified!"

# Launch Gazebo simulation
WORLD_FILE="src/courier_world/worlds/grid_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi

echo "🌍 Launching Gazebo with world: $WORLD_FILE"
ros2 launch ros_gz_sim gz_sim.launch.py world:="$WORLD_FILE" &
GAZEBO_PID=$!

# Wait for Gazebo to start
echo "⏳ Waiting for Gazebo..."
sleep 5
until ros2 topic list 2>/dev/null | grep -q "/clock"; do
    sleep 1
    echo "  Still waiting for Gazebo..."
done
echo "✅ Gazebo running"

# Spawn robot into simulation
echo "🤖 Spawning robot..."
ros2 run ros_gz_sim create -world default -file "$SDF_FILE" -name courier_robot -x 0 -y 0 -z 0.1

# Launch Navigation2
echo "🚀 Launching Navigation2..."
ros2 launch courier_navigation bringup.launch.py use_sim_time:=true autostart:=true &
NAV2_PID=$!

# Wait for Nav2 to initialize
sleep 3

# Launch BFS planner
echo "🧭 Launching BFS planner..."
ros2 run courier_bfs_planner bfs_node &
BFS_PID=$!

# Launch Behavior Tree
echo "🌲 Launching Behavior Tree..."
ros2 run courier_bt courier_bt_node &
BT_PID=$!

# Wait for all nodes
echo "✅ All nodes launched. Press Ctrl+C to stop."
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID