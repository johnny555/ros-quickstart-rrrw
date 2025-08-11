# Explorer Bot: Autonomous Maze Mapping Plan

This document outlines the step-by-step implementation plan for creating a TurtleBot4 control node that autonomously explores and maps a maze while detecting April tags.

## System Overview

- **Explorer Node**: Central control node orchestrating the autonomous exploration
- **NAV2 Stack**: Used for path planning and obstacle avoidance
- **SLAM Toolbox**: For simultaneous localization and mapping
- **April Tag Detection**: To identify and log April tag locations
- **Frontier Exploration**: Algorithm to efficiently explore unknown areas
- **RViz2 Visualization**: For monitoring exploration progress and April tag locations

## System Architecture

### Control Interface
- The exploration system will be implemented as a ROS 2 **Action Server** that provides:
  - Ability to start/stop exploration with specific goals
  - Feedback during exploration process
  - Final results upon completion
  - Cancellation capability
  
- The Action interface will define:
  - **Goal**: Target coverage percentage or time limit
  - **Feedback**: Current coverage percentage, tags found, exploration status
  - **Result**: Final map statistics, discovered tag locations, exploration success status

- An RViz2 panel will be developed to:
  - Provide GUI buttons to start/stop the exploration action
  - Display current exploration status and metrics
  - Allow parameter adjustment without restarting the node

### Component Interaction
- Explorer node will:
  - Subscribe to map updates from SLAM Toolbox
  - Process April tag detections
  - Identify frontier points
  - Send navigation goals to NAV2
  - Publish visualization markers for RViz2
  - Provide the action server interface

## Implementation Stages

### Stage 1: Environment Setup and Basic Navigation
1. Create the explorer_bot package
   - **Test**: Use `colcon list` to verify package existence and `colcon build` exit code to verify it builds correctly
   
2. Create custom launch file integrating:
   - TurtleBot4 simulation (reuse from simple_turtlebot4.launch.py)
   - SLAM Toolbox for mapping
   - Navigation2 stack for movement
   - **Test**: Use `ros2 node list` and `ros2 topic list` to verify all required nodes and topics are running

3. Set up RViz2 configuration with appropriate displays:
   - Robot model
   - Map
   - Path planning visualization
   - **Test**: Write a script that checks map topic is being published with non-empty data using `ros2 topic echo /map --once`

### Stage 2: April Tag Detection Integration
1. Create a node to detect April tags using the `apriltag_ros` package
   - **Implementation Detail**: Use the official ROS 2 Jazzy `apriltag_ros` package which provides:
     - Robust tag detection across various lighting conditions
     - Support for multiple tag families (36h11 required for our maze)
     - Accurate 6D pose estimation
     - Standard interfaces compatible with ROS 2 ecosystem
   - **Installation Note**: If not already installed, use apt to install the package:
     ```bash
     sudo apt update
     sudo apt install ros-jazzy-apriltag-ros
     ```
   - Configure the detector for our specific tags:
     - **Tag Family**: AprilTag 36h11 family
     - **Tag Size**: Exactly 0.4m x 0.4m (matching the physical tags in our maze)
     - **Tag Thickness**: Account for the 0.02m panel thickness in pose calculations
     - Configure detection thresholds for reliable detection at different distances
   - **Test**: Write a test node that publishes a simulated camera image containing an April tag, then verify detection messages are published

2. Implement a system to record April tag poses relative to the map
   - **Implementation Detail**: Use tf2 to transform tag poses from camera frame to map frame
   - Store tags in a persistent data structure with unique IDs
   - Handle duplicate tag detections by averaging or filtering based on detection confidence
   - **Test**: Create a test that detects a tag at a known position and verifies the recorded pose matches expected coordinates within tolerance

3. Add RViz2 markers for visualizing detected April tags
   - **Implementation Detail**: Generate unique markers for each tag with ID labels
   - Update marker positions when map corrections occur
   - **Test**: Write a script that counts visualization marker messages and verifies their properties match expected tag data

### Stage 3: Autonomous Exploration
1. Implement basic frontier detection algorithm
   - **Implementation Detail**: Identify frontiers by analyzing the occupancy grid map:
     - Define frontiers as transitions between free space (occupancy < 0.2) and unknown space (occupancy = -1)
     - Use a connected components algorithm to group adjacent frontier cells
     - Filter out frontier clusters that are too small (< threshold) or inaccessible
     - Extract the centroid or nearest accessible point for each frontier cluster
   - **Test**: Create unit tests with synthetic occupancy grid data to verify frontier point detection

2. Create exploration behavior:
   - **Implementation Detail**: Select best frontier point based on:
     - **Information Gain Calculation**:
       - Cast virtual rays from candidate point to estimate visible unknown area
       - Count number of unknown cells potentially visible from the frontier
       - Weight by distance from current position (gain/cost ratio)
     - Proximity to current position (to minimize travel time)
     - Estimated navigation difficulty (prefer easier paths)
     - Previous exploration attempts (avoid repeatedly failed destinations)
   - Navigate to the selected point using NAV2 action client
   - **Test**: Create a mock map with known frontiers and verify the algorithm selects the expected points; monitor goal messages to navigation stack

3. Implement recovery behaviors for when the robot gets stuck
   - **Implementation Detail**: Define "stuck" conditions:
     - No progress toward goal for X seconds
     - Oscillating movements with no net progress
     - Navigation server reports repeated failures
   - Recovery strategies:
     - Rotate in place to gather more sensor data
     - Attempt alternative paths with different planner parameters
     - Mark area as temporarily undesirable for exploration
     - Choose a different frontier if recovery fails
   - **Test**: Simulate a stuck condition (no progress for X seconds) and verify recovery actions are initiated via published topics

### Stage 4: Full System Integration and Optimization
1. Integrate all components into a cohesive system
   - **Test**: Create an automated test that verifies message flow between all components using `ros2 topic hz` and callback counters

2. Implement exploration completion detection
   - **Implementation Detail**: Define completion criteria:
     - Percentage of accessible area mapped (configurable parameter)
     - No remaining frontiers above minimum size threshold
     - All areas reachable within specified navigation cost
   - **Test**: Create a small known environment and verify the completion flag is set after expected coverage percentage is reached

3. Add performance metrics and logging
   - Track exploration efficiency, April tags found, etc.
   - **Test**: Create a benchmark environment and verify metrics are logged correctly in standard format

4. Optimize exploration strategy
   - Balance between covering new ground and revisiting uncertain areas
   - **Test**: Compare performance metrics before and after optimization in standardized test scenarios

## Testing Methodology

Our testing approach will focus on programmatic verification rather than manual observation:

1. **Unit Tests**:
   - Test individual components with mock data and inputs
   - Verify algorithmic correctness with predefined test cases
   - Use pytest framework for automated verification

2. **Integration Tests**:
   - Use ROS 2 launch testing framework to verify component interactions
   - Create test fixtures that monitor topic publications and service calls
   - Implement assertions for expected message sequences and content

3. **System Tests**:
   - Create reproducible test environments with known characteristics
   - Implement automated metrics collection for map coverage, accuracy, and completion time
   - Define quantitative success criteria for pass/fail determination

4. **Continuous Testing**:
   - Implement automated tests that can run without human supervision
   - Create test reports with quantitative results
   - Track performance trends across development iterations

## RViz2 Control Interface

We will implement an RViz2 panel plugin to control the exploration system:

1. **Panel Features**:
   - Start/Stop exploration buttons
   - Progress indicators (coverage percentage, elapsed time)
   - Tag detection count display
   - Parameter adjustment widgets (coverage goal, timeout)

2. **Implementation Approach**:
   - Create a custom RViz2 panel plugin (`explorer_bot_panel`)
   - Implement an action client to communicate with the exploration action server
   - Use Qt widgets for interactive elements
   - Update display elements based on action feedback

3. **Testing**:
   - Verify UI elements function correctly
   - Confirm action client properly communicates with server
   - Test that cancellation properly stops exploration process

## Evaluation Metrics

The explorer bot will be evaluated using these programmatically measurable metrics:

1. **Coverage**: 
   - Percentage of accessible maze area successfully mapped
   - Calculated by comparing known ground truth to generated map

2. **Accuracy**: 
   - Mean squared error between generated map and ground truth
   - Consistency of map across multiple runs

3. **Efficiency**: 
   - Time to reach X% coverage
   - Distance traveled to achieve complete coverage
   - Energy efficiency (simulated battery usage)

4. **Robustness**: 
   - Success rate across different maze configurations
   - Recovery effectiveness from simulated failures

5. **April Tag Detection**: 
   - Precision and recall metrics for tag detection
   - Accuracy of pose estimation (position error in meters)

## Development Workflow

1. Implement one stage at a time with test-driven development
2. Write automated tests before implementing features
3. Verify both functional correctness and performance metrics
4. Document test results and implementation decisions

## Next Steps

Begin with Stage 1, creating the basic package structure and launch files to integrate the necessary components for navigation and mapping. Focus on setting up automated testing infrastructure alongside initial development.
