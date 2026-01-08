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
python3 generate_apriltags.py

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

# Launch Gazebo with world (robot included)
WORLD_FILE="$(pwd)/src/courier_world/worlds/grid_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi

echo "🌍 Launching Gazebo with world: $WORLD_FILE"
echo "🧱 World includes: LOW walls (30cm), green start cell, blue goal cell, obstacles, and robot"
echo "🏷️  AprilTags placed at start and goal positions"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$WORLD_FILE" &
GAZEBO_PID=$!

# Wait for Gazebo to start
echo "⏳ Waiting for Gazebo..."
sleep 8
until ros2 topic list 2>/dev/null | grep -q "/clock"; do
    sleep 1
    echo "  Still waiting for Gazebo..."
done
echo "✅ Gazebo running with robot spawned!"

# Bridge Gazebo topics to ROS2
echo "🌉 Bridging Gazebo topics to ROS2..."
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image &
BRIDGE_PID=$!
sleep 2

# Launch AprilTag Localization
echo "🏷️  Launching AprilTag Localization..."
ros2 run courier_tag_localization tag_localization_node &
TAG_PID=$!

# Launch Navigation2
echo "🚀 Launching Navigation2..."
ros2 launch courier_navigation bringup.launch.py use_sim_time:=true autostart:=true &
NAV2_PID=$!

# Wait for Nav2 to initialize
sleep 3

echo "📍 Robot starts at GREEN cell (0,0) with AprilTag ID 0"
echo "🎯 Goal is BLUE cell (5,5) with AprilTag ID 1"
echo "🚧 Obstacles block the direct path"
echo "🧱 LOW walls (30cm) surround the 6x6 grid"
echo "📷 Camera detects AprilTags for localization"
echo "🔦 LiDAR scans for obstacle avoidance"
echo ""
echo "Sensors:"
echo "  • LiDAR: /scan (for Nav2 costmap)"
echo "  • Camera: /camera/image_raw (for AprilTag detection)"
echo "  • Odometry: /odom (for Nav2 localization)"
echo ""
echo "Mission Flow:"
echo "  1. BFS planner calculates path avoiding obstacles"
echo "  2. Nav2 uses LiDAR to build costmap and avoid obstacles"
echo "  3. Robot detects AprilTags for position verification"
echo "  4. Behavior tree orchestrates: navigate → grab → return"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Wait for all nodes
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID $TAG_PID $BRIDGE_PID
echo ""
echo "Mission Flow:"
echo "  1. BFS planner calculates path avoiding obstacles"
echo "  2. Behavior tree waits for path"
echo "  3. Robot navigates to goal (blue cell)"
echo "  4. Robot grabs package"
echo "  5. Robot returns to start (green cell)"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Wait for all nodes
wait $GAZEBO_PID $NAV2_PID $BFS_PID $BT_PID