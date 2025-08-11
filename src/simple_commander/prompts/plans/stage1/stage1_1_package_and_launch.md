# Stage 1: Environment Setup and Basic Navigation

## Objectives
- Create a well-structured ROS 2 package for the explorer bot
- Integrate TurtleBot4 simulation with SLAM and Navigation2
- Set up RViz2 for visualization and monitoring
- Establish automated testing framework

## Step 1.1: Package Creation

### Task
Create the `explorer_bot` ROS 2 package with proper dependencies and structure.

### Implementation Details
- Use `ros2 pkg create` CLI tool with appropriate dependencies including:
  - Core ROS 2 packages: `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`
  - Navigation packages: `tf2`, `tf2_ros`, `nav2_msgs`, `action_msgs`
  - Detection packages: `apriltag_msgs`, `visualization_msgs`
  - Create it as an `ament-cmake` package
- Add additional dependencies to `package.xml`:
  - `nav2_bringup`
  - `slam_toolbox`
  - `apriltag_ros`
  - `turtlebot4_simulator`
  - `pointcloud_to_laserscan` (for RGBD camera to laser scan conversion)

### Directory Structure
Create the following directories in the package:
```
explorer_bot/
├── src/
├── include/explorer_bot/
├── launch/
├── config/
├── test/
└── rviz/
```

### Testing Criteria
- **Test 1**: Verify package creation using `colcon list | grep explorer_bot`
- **Test 2**: Verify package builds successfully using `colcon build --merge-install --symlink-install --packages-select explorer_bot`

## Step 1.2: Launch File Development

### Task
Create a comprehensive launch file that integrates all necessary components for autonomous exploration.

### Implementation Details
- **Launch File**: `launch/explorer_bot.launch.py`
- **Components to Launch**:
  1. TurtleBot4 Gazebo simulation (using the example `src/simple_commander/simple_turtlebot4.launch.py`)
  2. PointCloud to LaserScan conversion node (for RGBD camera data)
  3. SLAM Toolbox for mapping
  4. Navigation2 stack for path planning
  5. April tag detection node
  6. Explorer bot control node
  7. RViz2 with custom configuration

### Key Configuration Parameters
- **PointCloud to LaserScan**:
  - Input topic: `/points` (from RGBD camera)
  - Output topic: `/scan_converted`
  - Height range: `0.1` to `1.0` meters (for navigation)
  
- **SLAM Toolbox**:
  - Mode: `mapping`
  - Map resolution: `0.05` meters/pixel
  - Update frequency: `5` Hz
  - Scan topic: `/scan_converted` (converted from PointCloud2)
  
- **Navigation2**:
  - Global planner: `NavfnPlanner`
  - Local planner: `DWBLocalPlanner`
  - Recovery behaviors: `spin`, `backup`
  
- **April Tag Detection**:
  - Tag family: `36h11`
  - Tag size: `0.4` meters
  - Detection frequency: `10` Hz

### Launch File Template
The launch file should include:
- TurtleBot4 Gazebo simulation launch
- PointCloud to LaserScan conversion with proper topic remapping
- SLAM Toolbox online async launch with converted scan data
- Navigation2 bringup launch
- April tag detection node with proper parameter configuration
- All necessary topic remappings for camera data

### Testing Criteria
- **Test 1**: Verify all required nodes are running using `ros2 node list`
- **Test 2**: Verify all required topics are published using `ros2 topic list`
