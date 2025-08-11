# Stage 2.1: AprilTag Detection Node Setup

This plan covers the integration of the `apriltag_ros` package for AprilTag detection in the maze environment.

## Objectives
- Set up AprilTag detection using the `apriltag_ros` package
- Configure detection parameters for the maze
- Remap topics for camera and detection output
- Test node startup and detection reliability

## Implementation Details
- **Package**: Use ROS 2 Jazzy `apriltag_ros`
- **Installation**: Install via apt if needed
- **Tag Family**: 36h11
- **Tag Size**: 0.4m x 0.4m
- **Tag Thickness**: 0.02m
- **Detection Parameters**:
  - `image_transport`: 'raw'
  - `family`: '36h11'
  - `size`: 0.4
  - `max_hamming`: 0
  - `detection_frequency`: 10 Hz

## Topic Remappings
- Input image: `/camera/image_raw`
- Camera info: `/camera/camera_info`
- Output detections: `/apriltag_detections`

## Testing Criteria
- Node subscribes to camera topics
- Simulated tag image produces detection messages
- Detection frequency meets 10 Hz target
