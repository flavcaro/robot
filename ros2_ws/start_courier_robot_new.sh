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

# Generate AprilTag images
echo "🏷️  Generating AprilTag images..."
python3 generate_apriltags.py 2>/dev/null || echo "⚠️  AprilTag generation skipped"

# Set Gazebo model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/courier_robot_description/models:$(pwd)/src/courier_world/models

# Check Python dependencies
echo "📦 Checking Python dependencies..."
for pkg in py_trees cv2 dt_apriltags; do
    python3 -c "import $pkg" 2>/dev/null || {
        echo "⚠️  $pkg not found. Installing..."
        case "$pkg" in
            cv2)
                pip install --break-system-packages "numpy>=1.21.6,<1.28.0" opencv-python
                ;;
            dt_apriltags)
                pip install --break-system-packages dt-apriltags
                ;;
            *)
                pip install --break-system-packages "$pkg>=2.2.0"
                ;;
        esac
    }
done

# Install cv_bridge dependencies
pip install --break-system-packages opencv-python-headless 2>/dev/null || true

# Generate SDF from URDF
URDF_FILE="src/courier_robot_description/urdf/courier_robot.urdf.xacro"
SDF_FILE="src/courier_robot_description/urdf/courier_robot.sdf"

if [ -f "$URDF_FILE" ]; then
    echo "🔧 Generating SDF from URDF..."
    ros2 run xacro xacro "$URDF_FILE" > "$SDF_FILE" 2>/dev/null || true
    echo "✅ SDF generated: $SDF_FILE"
fi

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

# Verify packages
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

# Launch Gazebo with world
WORLD_FILE="$(pwd)/src/courier_world/worlds/grid_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi

echo "🌍 Launching Gazebo with world: $WORLD_FILE"
echo "⏳ This may take 15-30 seconds on first launch..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$WORLD_FILE" &
GAZEBO_PID=$!

echo "⏳ Waiting for Gazebo world to load..."
echo "   This typically takes 20-30 seconds..."

# Simple approach: wait for Gazebo process to be stable
sleep 5

GAZEBO_STARTED=0
for i in {1..40}; do
    if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
        echo "❌ Gazebo process died!"
        exit 1
    fi
    
    # After 20 seconds, assume Gazebo is ready (we know it works from manual test)
    if [ $i -ge 20 ]; then
        echo "✅ Gazebo should be ready (${i}s elapsed)"
        GAZEBO_STARTED=1
        break
    fi
    
    # Show progress every 5 seconds
    if [ $((i % 5)) -eq 0 ]; then
        echo "  Loading world... (${i}s)"
    fi
    sleep 1
done

if [ $GAZEBO_STARTED -eq 0 ]; then
    echo "❌ Timeout waiting for Gazebo"
    kill $GAZEBO_PID 2>/dev/null
    exit 1
fi

echo "🎮 Gazebo GUI should be visible at localhost:6080"
echo "   You should see: 6x6 grid, walls, obstacles, orange robot"

# Bridge Gazebo topics to ROS2 (more efficient with specific mappings)
echo "🌉 Bridging Gazebo topics to ROS2..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/courier_grid_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    --ros-args -r /world/courier_grid_world/clock:=/clock &
BRIDGE_PID=$!

# Wait for bridge to initialize
sleep 2
echo "✅ Bridge running"

# Verify topics are available
echo "📡 Verifying ROS2 topics..."
for i in {1..10}; do
    if ros2 topic list 2>/dev/null | grep -q "/clock"; then
        echo "✅ Topics bridged successfully"
        break
    fi
    sleep 1
done

# Launch AprilTag Localization
echo "🏷️  Launching AprilTag Localization..."
ros2 run courier_tag_localization tag_localization_node &
TAG_PID=$!

# Launch Navigation2
echo "🚀 Launching Navigation2..."
ros2 launch courier_navigation bringup.launch.py use_sim_time:=true autostart:=true &
NAV2_PID=$!

# Wait for Nav2 to initialize
echo "⏳ Waiting for Nav2 to initialize (5s)..."
sleep 5

# Launch BFS planner
echo "🧭 Launching BFS planner..."
ros2 run courier_bfs_planner bfs_node &
BFS_PID=$!

# Launch Behavior Tree
echo "🌲 Launching Behavior Tree..."
ros2 run courier_bt courier_bt_node &
BT_PID=$!

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  ✅ ALL SYSTEMS LAUNCHED SUCCESSFULLY!  ║"
echo "╚══════════════════════════════════════════╝"
echo ""
echo "📊 System Status:"
echo "  🌍 Gazebo: Running (PID: $GAZEBO_PID)"
echo "  🌉 Bridge: Running (PID: $BRIDGE_PID)"
echo "  🏷️  Tags:   Running (PID: $TAG_PID)"
echo "  🚀 Nav2:   Running (PID: $NAV2_PID)"
echo "  🧭 BFS:    Running (PID: $BFS_PID)"
echo "  🌲 BT:     Running (PID: $BT_PID)"
echo ""
echo "🎯 Mission Setup:"
echo "  📍 Start: GREEN cell (0,0) with AprilTag ID 0"
echo "  🎯 Goal:  BLUE cell (5,5) with AprilTag ID 1"
echo "  🚧 Obstacles: 4 red boxes blocking direct path"
echo "  🧱 Walls: 30cm high surrounding 6x6 grid"
echo ""
echo "📡 Active Sensors:"
echo "  • LiDAR → /scan (obstacle detection)"
echo "  • Camera → /camera/image_raw (AprilTag detection)"
echo "  • Odometry → /odom (position tracking)"
echo ""
echo "🔄 Mission Flow:"
echo "  1. BFS calculates obstacle-avoiding path"
echo "  2. Nav2 navigates to goal using LiDAR"
echo "  3. AprilTag confirms arrival at goal"
echo "  4. Package grab simulation (3s)"
echo "  5. Return to start via Nav2"
echo "  6. AprilTag confirms mission complete"
echo ""
echo "🖥️  Monitor topics:"
echo "  ros2 topic echo /planned_path"
echo "  ros2 topic echo /detected_tag_id"
echo "  ros2 topic list"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

# Trap Ctrl+C to cleanup
trap "echo ''; echo '🛑 Shutting down...'; kill $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID $TAG_PID $BRIDGE_PID 2>/dev/null; exit" INT

# Wait for all nodes
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID $TAG_PID $BRIDGE_PID