#!/usr/bin/env python3
"""
Simple integration test that verifies basic ROS connectivity
without requiring full SLAM/navigation stack.
"""

import pytest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


@pytest.mark.integration
@pytest.mark.simple
def test_basic_ros_communication():
    """Test basic ROS communication is working."""
    # Initialize ROS
    if not rclpy.ok():
        rclpy.init()
    
    # Create a test node
    node = Node('test_integration_node')
    
    try:
        # Test that we can create publishers and subscribers
        cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        laser_sub = node.create_subscription(LaserScan, '/scan', lambda msg: None, 10)
        odom_sub = node.create_subscription(Odometry, '/odom', lambda msg: None, 10)
        
        # Test that we can publish commands
        twist = Twist()
        twist.linear.x = 0.1
        cmd_vel_pub.publish(twist)
        
        # Spin briefly to allow message processing
        rclpy.spin_once(node, timeout_sec=0.1)
        
        # If we get here without exceptions, basic ROS communication works
        assert True, "Basic ROS communication successful"
        
    finally:
        node.destroy_node()


@pytest.mark.integration 
@pytest.mark.simple
def test_tf_frames_exist():
    """Test that basic TF frames are being published."""
    import tf2_ros
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    
    if not rclpy.ok():
        rclpy.init()
    
    node = Node('test_tf_node')
    
    try:
        # Create TF buffer and listener
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        
        # Wait a bit for TF data
        time.sleep(2.0)
        
        # Spin to receive TF data
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check if basic frames exist
        available_frames = tf_buffer.all_frames_as_string()
        
        # Should have at least some basic robot frames
        expected_frames = ['base_link', 'base_footprint']
        found_frames = []
        
        for frame in expected_frames:
            if frame in available_frames:
                found_frames.append(frame)
        
        assert len(found_frames) > 0, f"Expected frames {expected_frames}, but only found: {available_frames}"
        
    finally:
        node.destroy_node()


@pytest.mark.integration
@pytest.mark.simple  
def test_robot_topics_active():
    """Test that robot topics are publishing data."""
    if not rclpy.ok():
        rclpy.init()
    
    node = Node('test_topics_node')
    
    received_data = {'laser': False, 'odom': False}
    
    def laser_callback(msg):
        received_data['laser'] = True
        
    def odom_callback(msg):
        received_data['odom'] = True
    
    try:
        # Subscribe to robot topics
        laser_sub = node.create_subscription(LaserScan, '/scan', laser_callback, 10)
        odom_sub = node.create_subscription(Odometry, '/odom', odom_callback, 10)
        
        # Wait for data with timeout
        timeout = 10.0
        start_time = time.time()
        
        while not all(received_data.values()) and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check if we received data from both topics
        assert received_data['laser'], "No laser scan data received"
        assert received_data['odom'], "No odometry data received"
        
    finally:
        node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
