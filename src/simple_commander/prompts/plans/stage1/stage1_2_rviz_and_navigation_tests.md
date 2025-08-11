# Stage 1: RViz2 Configuration and Basic Navigation Testing

## Step 1.3: RViz2 Configuration

### Task
Create a custom RViz2 configuration file optimized for exploration monitoring and April tag visualization.

### Implementation Details
- **Configuration File**: `rviz/explorer_bot.rviz`
- **Display Components**:
  1. Robot model with TF tree
  2. PointCloud2 display for RGBD camera data
  3. LaserScan display for converted scan data
  4. Map display with appropriate color scheme
  5. Path displays for global and local plans
  6. Marker display for April tags
  7. Goal pose tool for manual navigation testing

### Display Configuration Details
- **Map Display**:
  - Topic: `/map`
  - Color scheme: `map` (grayscale)
  - Alpha: `0.7`
  
- **PointCloud2 Display**:
  - Topic: `/points`
  - Size: `0.01`
  - Style: `Points`
  - Color: `Z-Axis` or `Intensity`
  
- **LaserScan Display**:
  - Topic: `/scan_converted`
  - Size: `0.03`
  - Style: `Points`
  - Color: `Intensity`

### Testing Criteria
- **Test 1**: Verify map topic contains data using `ros2 topic echo /map --once`
- **Test 2**: Verify RViz2 launches without errors using timeout command

## Step 1.4: Basic Navigation Testing

### Task
Implement automated tests to verify the navigation stack is functioning correctly with the TurtleBot4 simulation.

### Implementation Details
- **Test Node**: `test/test_navigation_basic.py`
- **Test Scenarios**:
  1. Send simple navigation goal and verify robot moves
  2. Check navigation feedback messages
  3. Verify obstacle avoidance behavior
  4. Test goal cancellation

### Navigation Test Implementation
Create a test node `test/test_navigation_basic.py` that:
- Creates an action client for NavigateToPose
- Sends a simple navigation goal (1 meter forward)
- Monitors goal acceptance and completion
- Verifies robot movement through odometry data
- Tests goal cancellation functionality

### Testing Criteria
- **Test 1**: Basic navigation goal acceptance
  - Send navigation goal and verify it's accepted
  - Expected: Goal accepted within 5 seconds

- **Test 2**: Robot movement verification
  - Monitor `/odom` topic for position changes
  - Expected: Position changes by > 0.1 meters within 30 seconds

- **Test 3**: Navigation completion
  - Verify navigation action completes successfully
  - Expected: Action result received within 60 seconds
