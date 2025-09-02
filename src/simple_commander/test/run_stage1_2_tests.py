#!/usr/bin/env python3
"""
Pytest-based Test Runner for Stage 1.2
Professional test runner using pytest framework for RViz2 configuration and navigation tests.
"""

import sys
import os
import subprocess
import pytest
import argparse


def check_ros_environment():
    """Check if ROS environment is properly set up."""
    print("=== Checking ROS Environment ===")
    
    # Check if ROS is sourced
    if 'ROS_DISTRO' not in os.environ:
        print("✗ ROS_DISTRO not set. Please source ROS setup files.")
        return False
    
    print(f"✓ ROS_DISTRO: {os.environ['ROS_DISTRO']}")
    
    # Check if workspace is built
    install_dir = '/workspace/install'
    if not os.path.exists(install_dir):
        print("✗ Workspace not built. Please run 'colcon build' first.")
        return False
    
    print("✓ Workspace appears to be built")
    return True


def check_pytest_availability():
    """Check if pytest is available."""
    try:
        import pytest
        print(f"✓ pytest version {pytest.__version__} available")
        return True
    except ImportError:
        print("✗ pytest not available. Please install: apt install python3-pytest")
        return False


def run_unit_tests():
    """Run unit tests only."""
    print("\n=== Running Unit Tests ===")
    test_dir = os.path.dirname(__file__)
    
    args = [
        test_dir,
        "-v",
        "-m", "unit",
        "--tb=short",
        "--no-header"
    ]
    
    return pytest.main(args)


def run_integration_tests():
    """Run integration tests (require running ROS system)."""
    print("\n=== Running Integration Tests ===")
    test_dir = os.path.dirname(__file__)
    
    args = [
        test_dir,
        "-v", 
        "-m", "integration",
        "--tb=short",
        "--no-header",
        "-x"  # Stop on first failure for integration tests
    ]
    
    return pytest.main(args)


def run_all_tests():
    """Run all tests."""
    print("\n=== Running All Tests ===")
    test_dir = os.path.dirname(__file__)
    
    args = [
        test_dir,
        "-v",
        "--tb=short",
        "--no-header"
    ]
    
    return pytest.main(args)


def run_specific_test(test_file):
    """Run a specific test file."""
    print(f"\n=== Running {test_file} ===")
    
    if not os.path.exists(test_file):
        print(f"✗ Test file not found: {test_file}")
        return 1
    
    args = [
        test_file,
        "-v",
        "--tb=short",
        "--no-header"
    ]
    
    return pytest.main(args)


def main():
    """Main entry point for the test runner."""
    parser = argparse.ArgumentParser(
        description="Stage 1.2 Test Runner using pytest",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --unit              # Run only unit tests
  %(prog)s --integration       # Run only integration tests  
  %(prog)s --all               # Run all tests
  %(prog)s --file test_rviz_map.py  # Run specific test file
  %(prog)s --ros-check         # Check ROS environment only
  
Test Markers:
  unit         - Tests that don't require running ROS system
  integration  - Tests that require running ROS system
  slow         - Tests that take more than 30 seconds
  rviz         - Tests that require RViz2
  navigation   - Tests that require navigation stack
        """
    )
    
    parser.add_argument('--unit', action='store_true',
                       help='Run unit tests only')
    parser.add_argument('--integration', action='store_true',
                       help='Run integration tests only (requires running ROS system)')
    parser.add_argument('--all', action='store_true',
                       help='Run all tests')
    parser.add_argument('--file', type=str,
                       help='Run specific test file')
    parser.add_argument('--ros-check', action='store_true',
                       help='Check ROS environment only')
    parser.add_argument('--list-tests', action='store_true',
                       help='List available tests without running them')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Stage 1.2 Professional Test Suite (pytest-based)")
    print("=" * 70)
    
    # Check dependencies
    if not check_pytest_availability():
        return 1
    
    if not check_ros_environment():
        return 1
    
    if args.ros_check:
        print("\n✓ ROS environment check completed successfully")
        return 0
    
    if args.list_tests:
        test_dir = os.path.dirname(__file__)
        pytest_args = [test_dir, "--collect-only", "-q"]
        return pytest.main(pytest_args)
    
    # Run tests based on arguments
    exit_code = 0
    
    if args.unit:
        exit_code = run_unit_tests()
    elif args.integration:
        exit_code = run_integration_tests()
    elif args.file:
        exit_code = run_specific_test(args.file)
    elif args.all:
        exit_code = run_all_tests()
    else:
        # Default: run unit tests first, then integration if unit tests pass
        print("No specific test type specified. Running unit tests first...")
        
        exit_code = run_unit_tests()
        
        if exit_code == 0:
            print("\n✓ Unit tests passed! Would you like to run integration tests?")
            print("Note: Integration tests require a running ROS system with TurtleBot4 simulation.")
            
            response = input("Run integration tests? [y/N]: ").lower().strip()
            if response in ['y', 'yes']:
                exit_code = run_integration_tests()
            else:
                print("Skipping integration tests.")
        else:
            print("\n✗ Unit tests failed. Fix unit tests before running integration tests.")
    
    # Print final summary
    print("\n" + "=" * 70)
    if exit_code == 0:
        print("🎉 All selected tests PASSED!")
        print("\nStage 1.2 (RViz2 Configuration and Basic Navigation) is working correctly.")
    else:
        print("❌ Some tests FAILED.")
        print("\nPlease check the test output above for details.")
    print("=" * 70)
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
