#!/bin/bash
# Manual Gazebo test to see actual errors

echo "=========================================="
echo "  Manual Gazebo Test"
echo "=========================================="
echo

cd ~/ros2_ws || exit 1
source install/setup.bash

# Set model paths
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/courier_robot_description/models:$(pwd)/src/courier_world/models

echo "Model paths set:"
echo "  $GZ_SIM_RESOURCE_PATH"
echo

# Check world file exists
WORLD_FILE="$(pwd)/src/courier_world/worlds/grid_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi

echo "✅ World file exists: $WORLD_FILE"
echo

# Check robot model exists
ROBOT_MODEL="$(pwd)/src/courier_robot_description/models/courier_robot/model.sdf"
if [ ! -f "$ROBOT_MODEL" ]; then
    echo "❌ Robot model not found: $ROBOT_MODEL"
    exit 1
fi

echo "✅ Robot model exists: $ROBOT_MODEL"
echo

# Check AprilTag models exist
if [ -f "src/courier_world/models/apriltag_0/model.sdf" ]; then
    echo "✅ AprilTag 0 model exists"
else
    echo "❌ AprilTag 0 model missing"
fi

if [ -f "src/courier_world/models/apriltag_1/model.sdf" ]; then
    echo "✅ AprilTag 1 model exists"
else
    echo "❌ AprilTag 1 model missing"
fi

echo
echo "=========================================="
echo "Starting Gazebo with VISIBLE errors..."
echo "=========================================="
echo
echo "Watch for error messages below:"
echo

# Launch Gazebo WITHOUT hiding output so we can see errors
gz sim "$WORLD_FILE" -v 4
