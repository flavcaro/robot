# Courier Robot Mission System

## Overview
Complete autonomous delivery mission system with BFS pathfinding, behavior tree orchestration, and Nav2 navigation.

## System Architecture

### 1. **BFS Path Planner** (`courier_bfs_planner`)
- **File**: `ros2_ws/src/courier_bfs_planner/courier_bfs_planner/bfs_node.py`
- **Purpose**: Grid-based pathfinding using Breadth-First Search algorithm
- **Features**:
  - 6x6 grid world representation
  - Start position: (0, 0) - Green cell
  - Goal position: (5, 5) - Blue cell
  - Obstacles at: (2,1), (3,3), (1,3), (4,2)
  - Publishes path to `/planned_path` topic as `nav_msgs/Path`
  - Automatically converts grid coordinates to world coordinates
- **Run**: `ros2 run courier_bfs_planner bfs_node`

### 2. **Behavior Tree** (`courier_bt`)
- **File**: `ros2_ws/src/courier_bt/courier_bt/courier_bt_node.py`
- **Purpose**: Mission orchestration and decision making
- **Mission Sequence**:
  1. **WaitForPath**: Wait for BFS planner to publish path
  2. **NavigateToGoal**: Send goal (5,5) to Nav2, navigate to blue cell
  3. **GrabPackage**: Simulate package pickup (2 second delay)
  4. **NavigateToStart**: Return to start position (0,0)
- **Features**:
  - Uses py_trees behavior tree library
  - Integrates with Nav2 action server
  - Subscribes to BFS planner path
  - Clear logging with status indicators (✓, →, ←, 📦)
- **Run**: `ros2 run courier_bt courier_bt_node`

### 3. **Gazebo World** (`courier_world`)
- **File**: `ros2_ws/src/courier_world/worlds/grid_world.sdf`
- **Components**:
  - **Green Start Cell**: Visual marker at (0, 0)
  - **Blue Goal Cell**: Visual marker at (5, 5)
  - **4 Obstacles**: Gray boxes at grid positions
  - **Courier Robot**: Spawns at start position
  - **Physics & Lighting**: Configured for simulation

### 4. **Navigation** (`courier_navigation`)
- **Purpose**: Nav2 configuration and parameter files
- **Features**: AMCL localization, DWB controller, costmaps

### 5. **Robot Description** (`courier_robot_description`)
- **Purpose**: URDF/xacro files for robot model
- **Features**: Robot geometry, sensors, joints

## Mission Flow

```
START
  ↓
[BFS Planner] → Calculates path avoiding obstacles → Publishes to /planned_path
  ↓
[Behavior Tree] → Waits for path
  ↓
[Behavior Tree] → Sends goal (5,5) to Nav2
  ↓
[Nav2] → Robot navigates to blue cell
  ↓
[Behavior Tree] → Simulates package grab (2s delay)
  ↓
[Behavior Tree] → Sends goal (0,0) to Nav2
  ↓
[Nav2] → Robot returns to green cell
  ↓
MISSION COMPLETE
```

## Running the System

### Quick Start
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
./start_courier_robot_new.sh
```

### Manual Launch (for debugging)
```bash
# Terminal 1: Launch Gazebo world
cd ~/ros2_ws
source install/setup.bash
ros2 launch courier_world world.launch.py

# Terminal 2: Launch Nav2
ros2 launch courier_navigation navigation.launch.py

# Terminal 3: Launch BFS Planner
ros2 run courier_bfs_planner bfs_node

# Terminal 4: Launch Behavior Tree
ros2 run courier_bt courier_bt_node
```

## Topics

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/planned_path` | nav_msgs/Path | BFS calculated path |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Nav2 action server |
| `/odom` | nav_msgs/Odometry | Robot odometry |
| `/map` | nav_msgs/OccupancyGrid | Map for navigation |

## Dependencies

### System Requirements
- ROS2 Jazzy
- Gazebo Harmonic
- Nav2
- Python 3.10+

### Python Packages
- rclpy (ROS2 Python client)
- py_trees (Behavior tree library)
- nav_msgs (Path messages)
- nav2_msgs (Nav2 action messages)
- geometry_msgs (Pose messages)

### Package Dependencies
All dependencies are declared in package.xml files and installed automatically with colcon build.

## Grid Configuration

The grid is configured as a 6x6 layout where each cell is 1m x 1m:

```
[5] [ ] [ ] [ ] [ ] [G]  ← Goal (Blue)
[4] [ ] [X] [ ] [ ] [ ]
[3] [ ] [X] [ ] [X] [ ]
[2] [ ] [ ] [ ] [ ] [X]
[1] [ ] [ ] [X] [ ] [ ]
[0] [S] [ ] [ ] [ ] [ ]  ← Start (Green)
    [0] [1] [2] [3] [4] [5]

Legend:
[S] = Start (0,0) - Green cell
[G] = Goal (5,5) - Blue cell
[X] = Obstacle - Gray boxes
[ ] = Free space
```

## Key Features

1. **Obstacle Avoidance**: BFS automatically plans around static obstacles
2. **Behavior-Driven**: Mission logic cleanly separated in behavior tree
3. **Nav2 Integration**: Uses industry-standard navigation stack
4. **Visual Feedback**: Color-coded cells and clear logging
5. **Modular Design**: Each component (planning, decision, navigation) is independent

## Troubleshooting

### Robot doesn't move
- Check Nav2 is running: `ros2 node list | grep nav2`
- Verify map published: `ros2 topic echo /map --once`
- Check AMCL localization: `ros2 topic echo /amcl_pose`

### No path from BFS planner
- Verify planner running: `ros2 node list | grep bfs`
- Check path topic: `ros2 topic echo /planned_path`
- Review planner logs: Look for "Path found" messages

### Behavior tree not progressing
- Check BT logs for current state
- Verify path received: Should see "✓ Path received from BFS planner"
- Check Nav2 action server: `ros2 action list`

## Next Steps

- **Add dynamic obstacle avoidance**: Integrate sensor data
- **Multi-goal missions**: Extend behavior tree for multiple deliveries
- **Real robot deployment**: Test on physical courier robot
- **Performance tuning**: Optimize BFS grid resolution and Nav2 parameters
