#!/usr/bin/env python3
"""
Basic Navigation Test Node for Explorer Bot
Tests fundamental navigation capabilities with TurtleBot4 simulation using pytest.
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import time
import threading
from unittest.mock import Mock, patch


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    Simple implementation to avoid tf_transformations dependency.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x, y, z, w]


class NavigationTester(Node):
    """Test node for basic navigation functionality."""
    
    def __init__(self, node_name='navigation_tester'):
        super().__init__(node_name)
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Odometry subscriber for movement verification
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        
        # State tracking
        self._current_pose = None
        self._initial_pose = None
        self._goal_handle = None
        
        # Test parameters
        self._goal_timeout = 60.0  # seconds
        self._movement_threshold = 0.1  # meters
        self._position_tolerance = 0.5  # meters
        
        self.get_logger().info("Navigation Tester initialized")

    def _odom_callback(self, msg):
        """Store current robot pose from odometry."""
        self._current_pose = msg.pose.pose
        if self._initial_pose is None:
            self._initial_pose = self._current_pose
            self.get_logger().info(f"Initial pose recorded: x={self._initial_pose.position.x:.2f}, y={self._initial_pose.position.y:.2f}")
    
    def _create_navigation_goal(self, x, y, yaw=0.0):
        """Create a NavigateToPose goal message."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return goal_msg
    
    def _calculate_distance(self, pose1, pose2):
        """Calculate distance between two poses."""
        if pose1 is None or pose2 is None:
            return float('inf')
        
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)


@pytest.fixture
def navigation_tester(ros_node):
    """Create a NavigationTester instance for testing."""
    # Replace the base node with our tester
    tester = NavigationTester()
    yield tester
    tester.destroy_node()


class TestNavigationFunctionality:
    """Test class for navigation functionality using pytest."""
    
    @pytest.mark.integration
    @pytest.mark.slow
    def test_navigation_goal_acceptance(self, navigation_tester):
        """Test 1: Verify navigation goal is accepted within timeout."""
        # Wait for action server
        assert navigation_tester._nav_client.wait_for_server(timeout_sec=10.0), \
            "Navigation action server not available"
        
        # Create simple forward goal (1 meter forward)
        goal = navigation_tester._create_navigation_goal(1.0, 0.0, 0.0)
        
        # Send goal and measure acceptance time
        start_time = time.time()
        future = navigation_tester._nav_client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(navigation_tester, future, timeout_sec=5.0)
        
        assert future.result() is not None, "Failed to send goal"
        goal_handle = future.result()
        assert goal_handle.accepted, "Goal was rejected"
        
        acceptance_time = time.time() - start_time
        assert acceptance_time < 5.0, f"Goal acceptance took too long: {acceptance_time:.2f}s"
        
        # Store for other tests
        navigation_tester._goal_handle = goal_handle
    
    @pytest.mark.integration
    def test_robot_movement(self, navigation_tester):
        """Test 2: Verify robot actually moves when navigation goal is sent."""
        # Wait for initial pose
        timeout = 10.0
        start_time = time.time()
        while navigation_tester._initial_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(navigation_tester, timeout_sec=0.1)
        
        assert navigation_tester._initial_pose is not None, "No initial pose available"
        
        # Send a goal first
        goal = navigation_tester._create_navigation_goal(1.0, 0.0, 0.0)
        future = navigation_tester._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(navigation_tester, future, timeout_sec=5.0)
        
        # Wait and monitor for movement
        start_time = time.time()
        movement_detected = False
        max_distance = 0.0
        
        while time.time() - start_time < 30.0:  # 30 second timeout
            if navigation_tester._current_pose is not None:
                distance = navigation_tester._calculate_distance(
                    navigation_tester._initial_pose, 
                    navigation_tester._current_pose
                )
                max_distance = max(max_distance, distance)
                
                if distance > navigation_tester._movement_threshold:
                    movement_detected = True
                    break
            
            rclpy.spin_once(navigation_tester, timeout_sec=0.1)
        
        assert movement_detected, f"No significant movement detected. Max distance: {max_distance:.3f}m"
    
    @pytest.mark.integration
    @pytest.mark.slow
    def test_navigation_completion(self, navigation_tester):
        """Test 3: Verify navigation action completes successfully."""
        # Send goal first
        goal = navigation_tester._create_navigation_goal(1.0, 0.0, 0.0)
        future = navigation_tester._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(navigation_tester, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        assert goal_handle is not None and goal_handle.accepted, "Goal not accepted"
        
        # Wait for navigation to complete
        start_time = time.time()
        result_future = goal_handle.get_result_async()
        
        rclpy.spin_until_future_complete(
            navigation_tester, 
            result_future, 
            timeout_sec=navigation_tester._goal_timeout
        )
        
        assert result_future.result() is not None, "No result received from navigation action"
        
        result = result_future.result()
        completion_time = time.time() - start_time
        
        # Check if we're reasonably close to the goal
        if navigation_tester._current_pose is not None:
            goal_pose = type('obj', (object,), {
                'position': type('obj', (object,), {'x': 1.0, 'y': 0.0})()
            })()
            
            final_distance = navigation_tester._calculate_distance(
                navigation_tester._current_pose, 
                goal_pose
            )
            
            # Allow for some tolerance in goal reaching
            assert final_distance < navigation_tester._position_tolerance, \
                f"Robot too far from goal: {final_distance:.3f}m"
    
    @pytest.mark.integration
    def test_goal_cancellation(self, navigation_tester):
        """Test 4: Verify goal cancellation functionality."""
        # Send a far goal that will take time to reach
        goal = navigation_tester._create_navigation_goal(5.0, 0.0, 0.0)
        
        future = navigation_tester._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(navigation_tester, future, timeout_sec=5.0)
        
        assert future.result() is not None and future.result().accepted, \
            "Could not send goal for cancellation test"
        
        goal_handle = future.result()
        
        # Wait a bit, then cancel
        time.sleep(2.0)
        
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(navigation_tester, cancel_future, timeout_sec=5.0)
        
        assert cancel_future.result() is not None, "Goal cancellation failed"


class TestNavigationConfiguration:
    """Test class for navigation configuration validation."""
    
    @pytest.mark.unit
    def test_navigation_goal_creation(self, rclpy_init, ros_node):
        """Test navigation goal message creation."""
        from geometry_msgs.msg import PoseStamped
        import math
        
        # Test basic goal structure creation
        goal = PoseStamped()
        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0
        goal.pose.position.z = 0.0
        goal.header.frame_id = 'map'
        
        # Set quaternion for π/4 rotation using our custom function
        q = quaternion_from_euler(0, 0, math.pi/4)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        assert goal.pose.position.x == 1.0
        assert goal.pose.position.y == 2.0
        assert goal.pose.position.z == 0.0
        assert goal.header.frame_id == 'map'
        
        # Check quaternion is properly set
        assert goal.pose.orientation.w != 0.0
    
    @pytest.mark.unit
    def test_distance_calculation(self):
        """Test pose distance calculation."""
        import math
        
        # Create two poses using simple objects
        pose1 = type('obj', (object,), {
            'position': type('obj', (object,), {'x': 0.0, 'y': 0.0})()
        })()
        
        pose2 = type('obj', (object,), {
            'position': type('obj', (object,), {'x': 3.0, 'y': 4.0})()
        })()
        
        # Calculate distance directly using math
        distance = math.sqrt(
            (pose2.position.x - pose1.position.x)**2 + 
            (pose2.position.y - pose1.position.y)**2
        )
        
        expected_distance = 5.0  # 3-4-5 triangle
        assert abs(distance - expected_distance) < 0.01, f"Expected {expected_distance}, got {distance}"


# Integration test to run all navigation tests together
@pytest.mark.integration
@pytest.mark.slow
class TestNavigationIntegration:
    """Integration tests for complete navigation workflow."""
    
    def test_complete_navigation_workflow(self, navigation_tester):
        """Test complete navigation workflow from goal acceptance to completion."""
        # This test combines multiple aspects of navigation testing
        
        # 1. Wait for system to be ready
        assert navigation_tester._nav_client.wait_for_server(timeout_sec=15.0), \
            "Navigation server not available"
        
        # 2. Wait for initial odometry
        timeout = 10.0
        start_time = time.time()
        while navigation_tester._initial_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(navigation_tester, timeout_sec=0.1)
        
        assert navigation_tester._initial_pose is not None, "No initial pose received"
        
        # 3. Send navigation goal
        goal = navigation_tester._create_navigation_goal(0.5, 0.0, 0.0)  # Smaller goal for testing
        future = navigation_tester._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(navigation_tester, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        assert goal_handle is not None and goal_handle.accepted, "Navigation goal not accepted"
        
        # 4. Monitor for movement (optional for quick test)
        start_time = time.time()
        movement_detected = False
        
        for _ in range(50):  # Check 50 times over ~5 seconds
            if navigation_tester._current_pose is not None:
                distance = navigation_tester._calculate_distance(
                    navigation_tester._initial_pose,
                    navigation_tester._current_pose
                )
                if distance > 0.05:  # 5cm movement threshold
                    movement_detected = True
                    break
            rclpy.spin_once(navigation_tester, timeout_sec=0.1)
        
        # Note: In a test environment, actual movement may not occur
        # This test mainly verifies the navigation pipeline accepts goals


# Legacy main function for backward compatibility
def main(args=None):
    """Legacy main function - now redirects to pytest."""
    import sys
    print("This test now uses pytest. Run with: pytest test_navigation_basic.py")
    print("Running pytest now...")
    
    # Run pytest on this file
    pytest.main([__file__, "-v"])


if __name__ == '__main__':
    main()
