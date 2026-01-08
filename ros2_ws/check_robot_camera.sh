#!/bin/bash
# Check if robot camera is working

echo "==================================="
echo "  Camera & Sensor Diagnostic"
echo "==================================="
echo

cd ~/ros2_ws || exit 1
source install/setup.bash 2>/dev/null

echo "1. Checking Gazebo process..."
if pgrep -f "gz sim" > /dev/null; then
    echo "✅ Gazebo process is running"
    GZ_PID=$(pgrep -f "gz sim")
    echo "   PID: $GZ_PID"
else
    echo "❌ Gazebo is NOT running!"
    echo "   Start with: ./start_courier_robot_new.sh"
    exit 1
fi

echo

echo "2. Checking Gazebo topics (gz)..."
if command -v gz > /dev/null 2>&1; then
    GZ_TOPICS=$(gz topic -l 2>/dev/null | head -10)
    if [ -n "$GZ_TOPICS" ]; then
        echo "✅ Gazebo internal topics detected:"
        echo "$GZ_TOPICS" | head -5
    else
        echo "⚠️  Cannot list Gazebo topics"
    fi
else
    echo "⚠️  'gz' command not found"
fi

echo

echo "3. Checking ROS2 topics..."
TOPICS=$(ros2 topic list 2>/dev/null)

if [ -z "$TOPICS" ] || [ "$(echo "$TOPICS" | wc -l)" -lt 3 ]; then
    echo "❌ Very few ROS2 topics found!"
    echo "   This means the ros_gz_bridge is not running or failed"
else
    echo "✅ ROS2 is active ($(echo "$TOPICS" | wc -l) topics)"
fi

echo

echo "4. Checking sensor topics..."
SENSORS_OK=0

if echo "$TOPICS" | grep -q "/camera/image_raw"; then
    echo "✅ Camera: /camera/image_raw"
    SENSORS_OK=$((SENSORS_OK + 1))
else
    echo "❌ Camera topic NOT found"
fi

if echo "$TOPICS" | grep -q "/scan"; then
    echo "✅ LiDAR: /scan"
    SENSORS_OK=$((SENSORS_OK + 1))
else
    echo "❌ LiDAR topic NOT found"
fi

if echo "$TOPICS" | grep -q "/odom"; then
    echo "✅ Odometry: /odom"
    SENSORS_OK=$((SENSORS_OK + 1))
else
    echo "❌ Odometry topic NOT found"
fi

if echo "$TOPICS" | grep -q "/cmd_vel"; then
    echo "✅ Control: /cmd_vel"
    SENSORS_OK=$((SENSORS_OK + 1))
else
    echo "❌ Control topic NOT found"
fi

echo

echo "5. Checking bridge process..."
if pgrep -f "parameter_bridge" > /dev/null; then
    echo "✅ ros_gz_bridge is running"
else
    echo "❌ ros_gz_bridge is NOT running!"
    echo "   The bridge connects Gazebo to ROS2"
fi

echo

if [ $SENSORS_OK -eq 0 ]; then
    echo "⚠️⚠️⚠️  PROBLEM DETECTED  ⚠️⚠️⚠️"
    echo
    echo "NO sensor topics found. Possible causes:"
    echo
    echo "1. Robot model didn't spawn in Gazebo"
    echo "   → Check Gazebo GUI (localhost:6080)"
    echo "   → Look for orange robot at (0.5, 0.5)"
    echo
    echo "2. ros_gz_bridge failed to start"
    echo "   → Check if bridge process is running"
    echo "   → Check GZ_SIM_RESOURCE_PATH is set"
    echo
    echo "3. World file has errors"
    echo "   → Check Gazebo terminal for error messages"
    echo
    echo "🔧 TROUBLESHOOTING:"
    echo "   1. Check Gazebo terminal for errors"
    echo "   2. Verify robot model exists:"
    echo "      ls src/courier_robot_description/models/courier_robot/"
    echo "   3. Try manual spawn:"
    echo "      export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$(pwd)/src/courier_robot_description/models"
    echo "      gz service -s /world/courier_grid_world/create --reqtype gz.msgs.EntityFactory --req 'sdf_filename: \"model://courier_robot\"'"
    echo
else
    echo "✅ Found $SENSORS_OK/4 sensor topics"
fi

echo
echo "==================================="
echo "  All ROS2 Topics:"
echo "==================================="
echo "$TOPICS"

echo
echo "💡 Useful commands:"
echo "   ros2 topic hz /camera/image_raw  # Check camera rate"
echo "   ros2 topic echo /scan --once     # Test LiDAR"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\" --once"
