#!/bin/bash
# Quick system diagnostic script

echo "============================================"
echo "  Courier Robot - System Diagnostic"
echo "============================================"
echo

cd ~/ros2_ws || exit 1

# Check AprilTag images exist
echo "1. Checking AprilTag images..."
if [ -f "src/courier_world/models/apriltag_0/tag.png" ]; then
    echo "   ✅ apriltag_0/tag.png exists"
else
    echo "   ❌ apriltag_0/tag.png MISSING - Run: python3 generate_apriltags.py"
fi

if [ -f "src/courier_world/models/apriltag_1/tag.png" ]; then
    echo "   ✅ apriltag_1/tag.png exists"
else
    echo "   ❌ apriltag_1/tag.png MISSING - Run: python3 generate_apriltags.py"
fi

echo
echo "2. Checking Python dependencies..."
python3 -c "import py_trees" 2>/dev/null && echo "   ✅ py_trees" || echo "   ❌ py_trees"
python3 -c "import cv2" 2>/dev/null && echo "   ✅ cv2 (opencv)" || echo "   ❌ cv2"
python3 -c "import dt_apriltags" 2>/dev/null && echo "   ✅ dt_apriltags" || echo "   ❌ dt_apriltags"

echo
echo "3. Checking ROS2 packages..."
source install/setup.bash 2>/dev/null
for pkg in courier_world courier_navigation courier_bfs_planner courier_bt courier_tag_localization courier_robot_description; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo "   ✅ $pkg"
    else
        echo "   ❌ $pkg - Run: colcon build --symlink-install"
    fi
done

echo
echo "4. Checking model files..."
if [ -f "src/courier_robot_description/models/courier_robot/model.sdf" ]; then
    echo "   ✅ Robot model exists"
else
    echo "   ❌ Robot model MISSING"
fi

if [ -f "src/courier_world/worlds/grid_world.sdf" ]; then
    echo "   ✅ World file exists"
else
    echo "   ❌ World file MISSING"
fi

echo
echo "5. Checking GZ_SIM_RESOURCE_PATH..."
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/courier_robot_description/models:$(pwd)/src/courier_world/models
echo "   Set to: $GZ_SIM_RESOURCE_PATH"

echo
echo "============================================"
echo "  Diagnostic Complete"
echo "============================================"
echo
echo "If all checks pass, run: ./start_courier_robot_new.sh"
