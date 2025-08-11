# Stage 2.2: Tag Pose Recording & Visualization

This plan covers recording AprilTag poses in the map frame and visualizing them in RViz2.

## Objectives
- Record AprilTag poses relative to the map frame
- Visualize detected tags in RViz2 with markers
- Handle duplicate detections and pose averaging

## Implementation Details
### Tag Pose Recording
- **Node**: `tag_pose_recorder`
- **TF Integration**: Use tf2 for camera → base → map transforms
- **Data Storage**: Persistent structure with unique tag IDs
- **Duplicate Handling**: Average detections, filter by confidence, update on SLAM corrections
- **Data Structure**:
  - Tag ID
  - Pose in map frame
  - Confidence score
  - First/last detection timestamps
  - Number of detections

### Visualization in RViz2
- **Marker Publisher**: Node publishes CUBE markers with text labels
- **Dynamic Updates**: Update marker positions as tag poses are refined
- **Visual Design**:
  - Green for recent, yellow for older detections
  - Size: 0.4m
  - Text: Tag ID
- **Marker Config**:
  - Topic: `/april_tag_markers`
  - Frame: `map`
  - Namespace: `april_tags`
  - Lifetime: persistent
  - Colors: RGB for status
- **Interactive Features**:
  - Click selection in RViz2
  - Info display on selection
  - Update indicators

## Testing Criteria
- TF transforms work correctly
- Known tag position is accurately recorded
- Duplicate handling and averaging verified
- Markers appear and update in RViz2
- Marker persistence across restarts
