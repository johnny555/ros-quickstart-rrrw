# Simple TurtleBot4 Launch File

This launch file provides a minimal setup for TurtleBot4 simulation with Gazebo Harmonic.

## What it includes:

- **Gazebo Harmonic**: The simulation environment
- **Robot Description**: A simplified TurtleBot4-like differential drive robot
- **ROS2 Control**: Joint state publisher and differential drive controller
- **Basic ROS-Gazebo Bridges**: For cmd_vel, odometry, and joint states
- **Robot Spawning**: Spawns the robot based on the robot description

## Usage:

### Basic Launch:
```bash
# Source your workspace
source /workspace/install/setup.bash

# Launch with default settings (empty world, standard model)
ros2 launch simple_commander simple_turtlebot4.launch.py
```

### Launch with custom parameters:
```bash
# Launch with different world and position
ros2 launch simple_commander simple_turtlebot4.launch.py world:=maze x:=1.0 y:=2.0 yaw:=1.57

# Launch with lite model
ros2 launch simple_commander simple_turtlebot4.launch.py model:=lite

# Launch with custom robot name
ros2 launch simple_commander simple_turtlebot4.launch.py robot_name:=my_robot
```

## Available Parameters:

- `use_sim_time`: Use simulation time (default: true)
- `world`: Gazebo world file name without .sdf extension (default: empty)
- `model`: TurtleBot4 model type - 'standard' or 'lite' (default: standard)
- `x`: X position of robot spawn (default: 0.0)
- `y`: Y position of robot spawn (default: 0.0)
- `z`: Z position of robot spawn (default: 0.0)
- `yaw`: Yaw orientation of robot spawn in radians (default: 0.0)
- `robot_name`: Name of the robot in simulation (default: turtlebot4)

## Testing the Robot:

Once launched, you can test the robot by sending velocity commands:

```bash
# Test robot movement with direct topic publishing
ros2 topic pub /turtlebot4/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Or use the included simple teleop script (recommended)
ros2 run simple_commander simple_teleop.py
# Controls: w=forward, s=backward, a=left, d=right, space=stop, q=quit

# Check available topics
ros2 topic list

# Check robot transforms
ros2 run tf2_tools view_frames

# Monitor odometry
ros2 topic echo /turtlebot4/odom

# Monitor joint states
ros2 topic echo /joint_states
```

## What's Different from Full TurtleBot4 Launch:

This simplified version:
- ✅ Uses a basic robot description instead of the full TurtleBot4 URDF
- ✅ Includes only essential ROS-Gazebo bridges
- ✅ Doesn't include navigation stack, SLAM, or other advanced features
- ✅ Doesn't spawn the charging dock
- ✅ Minimal dependencies - easier to understand and modify
- ✅ Faster startup time

## Extending the Launch File:

You can easily extend this launch file by:
1. Adding more ROS-Gazebo bridges for sensors
2. Including the full TurtleBot4 description package
3. Adding navigation or SLAM capabilities
4. Including RViz for visualization

This serves as a clean starting point for TurtleBot4 simulation development.
