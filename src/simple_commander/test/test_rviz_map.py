#!/usr/bin/env python3
"""
RViz2 and Map Testing Script for Explorer Bot
Tests RViz2 configuration and verifies map topic data availability using pytest.
"""

import pytest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import subprocess
import signal
import time
import sys
import os
import yaml
from unittest.mock import Mock, patch


class RVizMapTester(Node):
    """Test node for RViz2 configuration and map data."""
    
    def __init__(self, node_name='rviz_map_tester'):
        super().__init__(node_name)
        
        self._map_received = False
        self._map_data = None
        
        # Subscribe to map topic
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )
        
        self.get_logger().info("RViz Map Tester initialized")
    
    def _map_callback(self, msg):
        """Callback for map data."""
        self._map_received = True
        self._map_data = msg
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height} cells, resolution: {msg.info.resolution}")


@pytest.fixture
def rviz_map_tester(ros_node):
    """Create an RVizMapTester instance for testing."""
    tester = RVizMapTester()
    yield tester
    tester.destroy_node()


class TestRVizConfiguration:
    """Test class for RViz configuration validation."""
    
    @pytest.mark.unit
    def test_rviz_config_file_exists(self, rviz_config_path):
        """Test that RViz configuration file exists."""
        assert os.path.exists(rviz_config_path), f"RViz config file not found: {rviz_config_path}"
    
    @pytest.mark.unit
    def test_rviz_config_structure(self, rviz_config_path):
        """Test that RViz configuration has proper YAML structure."""
        with open(rviz_config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for essential sections
        assert 'Visualization Manager' in config, "Missing Visualization Manager section"
        assert 'Displays' in config['Visualization Manager'], "Missing Displays section"
        
        displays = config['Visualization Manager']['Displays']
        assert isinstance(displays, list), "Displays should be a list"
        
        # Check for essential display types
        display_classes = [display.get('Class', '') for display in displays if isinstance(display, dict)]
        
        expected_displays = [
            'rviz_default_plugins/RobotModel',
            'rviz_default_plugins/Map',
            'rviz_default_plugins/LaserScan',
            'rviz_default_plugins/PointCloud2'
        ]
        
        for expected in expected_displays:
            assert any(expected in cls for cls in display_classes), \
                f"Missing expected display type: {expected}"
    
    @pytest.mark.unit
    def test_rviz_config_topics(self, rviz_config_path):
        """Test that RViz configuration references correct topics."""
        with open(rviz_config_path, 'r') as f:
            content = f.read()
        
        # Check for expected topics in the configuration
        expected_topics = ['/map', '/scan', '/robot_description']
        
        for topic in expected_topics:
            assert topic in content, f"Topic {topic} not found in RViz configuration"


class TestMapFunctionality:
    """Test class for map topic functionality."""
    
    @pytest.mark.integration
    @pytest.mark.slow
    def test_map_topic_data(self, rviz_map_tester):
        """Test if map topic contains valid data."""
        timeout = 30.0
        start_time = time.time()
        
        while not rviz_map_tester._map_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(rviz_map_tester, timeout_sec=0.1)
        
        assert rviz_map_tester._map_received, "No map data received within timeout"
        assert rviz_map_tester._map_data is not None, "Map data is None"
        
        # Validate map data structure
        map_data = rviz_map_tester._map_data
        assert map_data.info.width > 0, "Map width should be positive"
        assert map_data.info.height > 0, "Map height should be positive"
        assert map_data.info.resolution > 0, "Map resolution should be positive"
        assert len(map_data.data) == map_data.info.width * map_data.info.height, \
            "Map data length doesn't match width x height"
    
    @pytest.mark.unit
    def test_map_message_structure(self):
        """Test OccupancyGrid message structure."""
        # Create a mock map message
        map_msg = OccupancyGrid()
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.resolution = 0.05
        map_msg.data = [0] * (100 * 100)
        
        # Test the structure
        assert hasattr(map_msg, 'info'), "Map message should have info field"
        assert hasattr(map_msg, 'data'), "Map message should have data field"
        assert hasattr(map_msg.info, 'width'), "Map info should have width"
        assert hasattr(map_msg.info, 'height'), "Map info should have height"
        assert hasattr(map_msg.info, 'resolution'), "Map info should have resolution"
        assert hasattr(map_msg.info, 'origin'), "Map info should have origin"


class TestRVizLaunch:
    """Test class for RViz launch functionality."""
    
    @pytest.mark.integration
    @pytest.mark.slow
    def test_rviz_launch_no_crash(self, rviz_config_path):
        """Test if RViz2 launches without immediate crashes."""
        # This test may need to be skipped in headless environments
        if not os.environ.get('DISPLAY'):
            pytest.skip("No DISPLAY environment variable - skipping RViz launch test")
        
        try:
            # Launch RViz2 with timeout
            process = subprocess.Popen(
                ['rviz2', '-d', rviz_config_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait for RViz2 to start
            time.sleep(5.0)
            
            # Check if process is still running (no immediate crash)
            assert process.poll() is None, "RViz2 crashed during startup"
            
            # Kill the process
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)
            
        except subprocess.TimeoutExpired:
            # Force kill if timeout
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except Exception as e:
            pytest.fail(f"Failed to launch RViz2: {e}")
    
    @pytest.mark.unit  
    def test_rviz_command_availability(self):
        """Test that rviz2 command is available."""
        result = subprocess.run(['which', 'rviz2'], capture_output=True)
        assert result.returncode == 0, "rviz2 command not found in PATH"


class TestConfigurationFiles:
    """Test class for configuration file validation."""
    
    @pytest.mark.unit
    def test_slam_config_exists(self, slam_config_path):
        """Test that SLAM configuration file exists."""
        assert os.path.exists(slam_config_path), f"SLAM config file not found: {slam_config_path}"
    
    @pytest.mark.unit
    def test_slam_config_structure(self, slam_config_path):
        """Test SLAM configuration structure."""
        with open(slam_config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        assert 'slam_toolbox' in config, "Missing slam_toolbox section"
        slam_params = config['slam_toolbox']['ros__parameters']
        
        # Check for essential parameters
        essential_params = ['odom_frame', 'map_frame', 'base_frame', 'scan_topic']
        for param in essential_params:
            assert param in slam_params, f"Missing SLAM parameter: {param}"
        
        # Validate frame values
        assert slam_params['odom_frame'] == 'odom', "odom_frame should be 'odom'"
        assert slam_params['map_frame'] == 'map', "map_frame should be 'map'"
        assert slam_params['base_frame'] == 'base_link', "base_frame should be 'base_link'"
    
    @pytest.mark.unit
    def test_launch_file_exists(self, launch_file_path):
        """Test that launch file exists and has proper structure."""
        assert os.path.exists(launch_file_path), f"Launch file not found: {launch_file_path}"
        
        # Check that it's a valid Python file
        with open(launch_file_path, 'r') as f:
            content = f.read()
        
        # Basic syntax check
        compile(content, launch_file_path, 'exec')
        
        # Check for essential launch components
        assert 'slam_toolbox' in content, "Launch file should reference slam_toolbox"
        assert 'rviz2' in content, "Launch file should reference rviz2"
        assert 'generate_launch_description' in content, "Missing launch description function"


# Integration test for complete RViz workflow
@pytest.mark.integration
class TestRVizIntegration:
    """Integration tests for complete RViz and mapping workflow."""
    
    def test_complete_rviz_workflow(self, rviz_map_tester, rviz_config_path):
        """Test complete RViz workflow including configuration validation."""
        # 1. Validate configuration file
        assert os.path.exists(rviz_config_path), "RViz configuration file missing"
        
        # 2. Validate YAML structure
        with open(rviz_config_path, 'r') as f:
            config = yaml.safe_load(f)
        assert 'Visualization Manager' in config, "Invalid RViz configuration structure"
        
        # 3. Test map topic subscription (if available)
        # Note: In test environment, map topic may not be available
        # This mainly tests the subscription setup
        assert rviz_map_tester._map_sub is not None, "Map subscription not created"
        
        # 4. Verify topic name
        topic_names = [sub.topic_name for sub in rviz_map_tester.subscriptions]
        assert '/map' in topic_names, "Not subscribed to /map topic"


# Legacy main function for backward compatibility
def main(args=None):
    """Legacy main function - now redirects to pytest."""
    print("This test now uses pytest. Run with: pytest test_rviz_map.py")
    print("Running pytest now...")
    
    # Run pytest on this file
    pytest.main([__file__, "-v"])


if __name__ == '__main__':
    main()
