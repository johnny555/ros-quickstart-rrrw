# Stage 4.1: System Integration

## Core System Integration

### Task
Combine all developed components (navigation, April tag detection, exploration) into a single, coordinated system.

### Implementation Details
- **Central Controller**: Create main `explorer_bot_node` that orchestrates all subsystems
- **Component Communication**: Establish proper message flow between all nodes
- **State Management**: Implement exploration state machine (IDLE, EXPLORING, PAUSED, COMPLETED)
- **Error Handling**: Comprehensive error handling and graceful degradation
- **Resource Management**: Efficient CPU and memory usage across all components

### System Architecture
- **Explorer Bot Node**: Central coordination and state management
- **Frontier Detector**: Continuous frontier identification and updates
- **Tag Manager**: April tag detection and pose recording
- **Navigation Controller**: Interface with Navigation2 stack
- **Recovery Manager**: Handle stuck conditions and navigation failures
- **Performance Monitor**: Track metrics and system health

### Inter-Component Communication
- **Topic Interfaces**: Standardized message types for component communication
- **Service Calls**: Synchronous operations for critical state changes
- **Parameter Server**: Shared configuration and tuning parameters
- **TF Integration**: Coordinate frame transformations across all components

### Testing Criteria
- **Test 1**: Verify all components start and communicate correctly
- **Test 2**: Test complete exploration workflow from start to finish
- **Test 3**: Confirm system handles component failures gracefully

---

# Stage 4.2: Action Server Implementation

## Action Server Implementation

### Task
Implement a ROS 2 Action Server interface that provides external control of the exploration system.

### Implementation Details
- **Action Definition**: Create custom action type `ExploreArea.action`
- **Goal Parameters**:
  - Target coverage percentage (default: 80%)
  - Maximum exploration time limit (seconds)
  - Priority areas or exclusion zones (optional)
- **Feedback Messages**:
  - Current coverage percentage
  - Number of April tags found
  - Current exploration status
  - Estimated time remaining
- **Result Data**:
  - Final coverage achieved
  - Total April tags discovered with poses
  - Exploration success status
  - Performance metrics summary

### Action Interface Design
- **Goal Validation**: Check parameters and system readiness
- **Preemption Support**: Allow goal cancellation and replacement
- **Feedback Frequency**: Regular status updates (1 Hz minimum)
- **Result Persistence**: Store results for post-exploration analysis

### State Management
- **Action States**: Map internal exploration states to action states
- **Transition Handling**: Smooth transitions between action goals
- **Cancellation**: Clean shutdown of exploration when action is cancelled
- **Error Recovery**: Handle action server failures and restarts

### Testing Criteria
- **Test 1**: Verify action server accepts and validates goals correctly
- **Test 2**: Test action cancellation and preemption scenarios
- **Test 3**: Confirm feedback and result messages contain accurate data
