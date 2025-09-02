"""
Pytest configuration and shared fixtures for Stage 1.2 tests.
"""

import pytest
import rclpy
import threading
import time
import os
from rclpy.executors import MultiThreadedExecutor


@pytest.fixture(scope="session")
def rclpy_init():
    """Initialize ROS 2 for the test session."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope="function")
def ros_node(rclpy_init):
    """Create a ROS 2 node for testing."""
    from rclpy.node import Node
    
    node = Node('test_node')
    yield node
    node.destroy_node()


@pytest.fixture(scope="function") 
def executor(rclpy_init):
    """Create a MultiThreadedExecutor for testing."""
    executor = MultiThreadedExecutor()
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    yield executor
    executor.shutdown()


@pytest.fixture
def rviz_config_path():
    """Return path to RViz configuration file."""
    package_dir = os.path.dirname(os.path.dirname(__file__))  # Go up to simple_commander directory
    config_path = os.path.join(package_dir, 'rviz', 'explorer_bot.rviz')
    assert os.path.exists(config_path), f"RViz config not found: {config_path}"
    return config_path


@pytest.fixture
def slam_config_path():
    """Return path to SLAM configuration file."""
    package_dir = os.path.dirname(os.path.dirname(__file__))  # Go up to simple_commander directory
    config_path = os.path.join(package_dir, 'config', 'slam_toolbox.yaml')
    assert os.path.exists(config_path), f"SLAM config not found: {config_path}"
    return config_path


@pytest.fixture
def launch_file_path():
    """Return path to launch file."""
    package_dir = os.path.dirname(os.path.dirname(__file__))  # Go up to simple_commander directory
    launch_path = os.path.join(package_dir, 'launch', 'simple_turtlebot4.launch.py')
    assert os.path.exists(launch_path), f"Launch file not found: {launch_path}"
    return launch_path


@pytest.fixture
def wait_for_topic():
    """Helper fixture to wait for ROS topics to become available."""
    def _wait_for_topic(node, topic_name, msg_type, timeout=10.0):
        """Wait for a topic to become available and return a subscription."""
        subscription = node.create_subscription(
            msg_type,
            topic_name,
            lambda msg: None,  # Dummy callback
            10
        )
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            topic_names = node.get_topic_names_and_types()
            if any(name == topic_name for name, _ in topic_names):
                return subscription
            time.sleep(0.1)
        
        pytest.fail(f"Topic {topic_name} not available after {timeout} seconds")
    
    return _wait_for_topic


@pytest.fixture
def mock_navigation_server():
    """Mock navigation action server for testing."""
    # This would be implemented if we need to mock the navigation server
    # For now, we'll assume integration tests run with real system
    pass


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "integration: integration tests requiring ROS system"
    )
    config.addinivalue_line(
        "markers", "unit: unit tests that can run standalone"
    )
    config.addinivalue_line(
        "markers", "slow: tests that take more than 30 seconds"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection to add markers based on test names."""
    for item in items:
        # Add integration marker to tests that require ROS system
        if "integration" in item.nodeid or "nav" in item.nodeid or "rviz" in item.nodeid:
            item.add_marker(pytest.mark.integration)
        
        # Add unit marker to configuration tests
        if "config" in item.nodeid or "validation" in item.nodeid:
            item.add_marker(pytest.mark.unit)
        
        # Add slow marker to navigation tests
        if "navigation" in item.nodeid:
            item.add_marker(pytest.mark.slow)
