# Simple Commander Launch Files

This package provides launch files to easily start TurtleBot4 simulation with all necessary components.

## Launch Files

### 1. turtlebot4_simple.launch.py
A simplified launch file that starts:
- Gazebo simulation with TurtleBot4
- ROS-Gazebo bridge for communication

**Usage:**
```bash
ros2 launch simple_commander turtlebot4_simple.launch.py
```

**Parameters:**
- `world`: World name (default: 'empty') - Options: empty, warehouse, maze, etc.
- `model`: TurtleBot4 model (default: 'standard') - Options: standard, lite
- `rviz`: Start RViz (default: 'false') - Options: true, false

**Examples:**
```bash
# Start with warehouse world
ros2 launch simple_commander turtlebot4_simple.launch.py world:=warehouse

# Start with RViz
ros2 launch simple_commander turtlebot4_simple.launch.py rviz:=true

# Start with lite model in empty world
ros2 launch simple_commander turtlebot4_simple.launch.py model:=lite world:=empty
```

### 2. turtlebot4_complete.launch.py
A comprehensive launch file that starts:
- Gazebo simulation with TurtleBot4
- ROS-Gazebo bridge for communication
- TurtleBot4 nodes
- Optional navigation, SLAM, and localization

**Usage:**
```bash
ros2 launch simple_commander turtlebot4_complete.launch.py
```

**Parameters:**
- `world`: World name (default: 'warehouse')
- `model`: TurtleBot4 model (default: 'standard')
- `rviz`: Start RViz (default: 'false')
- `namespace`: Robot namespace (default: '')
- `localization`: Enable localization (default: 'false')
- `slam`: Enable SLAM (default: 'false')
- `nav2`: Enable Nav2 navigation (default: 'false')
- `x`, `y`, `z`, `yaw`: Initial robot pose (default: '0.0')

**Examples:**
```bash
# Start with SLAM enabled
ros2 launch simple_commander turtlebot4_complete.launch.py slam:=true

# Start with navigation and RViz
ros2 launch simple_commander turtlebot4_complete.launch.py nav2:=true rviz:=true

# Start at specific position
ros2 launch simple_commander turtlebot4_complete.launch.py x:=1.0 y:=2.0 yaw:=1.57
```

## Prerequisites

Make sure you have built the workspace:
```bash
cd /workspace
colcon build --merge-install
source install/setup.bash
```

## What's Included

These launch files automatically start:

1. **Gazebo Simulation**: The Gazebo physics simulator with the selected world
2. **Robot Spawning**: TurtleBot4 robot model spawned in the simulation
3. **ROS-Gazebo Bridge**: Communication bridge between ROS2 and Gazebo including:
   - Lidar sensor data
   - Camera sensor data (RGB, depth, point cloud)
   - IMU data
   - Wheel odometry
   - Motor commands
   - LED control (for standard model)
   - HMI display and buttons (for standard model)

4. **TurtleBot4 Nodes** (complete launch only): Additional ROS2 nodes for robot functionality

## Available Worlds

- `empty`: Empty world with just a ground plane
- `warehouse`: Warehouse environment with obstacles
- `maze`: Maze environment for navigation testing
- `depot`: Depot environment
- `classroom`: Classroom environment

## Troubleshooting

1. If launch fails, ensure all dependencies are installed:
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot4-simulator
   ```

2. Make sure the workspace is properly sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

3. Check that Gazebo can find the worlds:
   ```bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix turtlebot4_gz_bringup)/share/turtlebot4_gz_bringup/worlds
   ```
