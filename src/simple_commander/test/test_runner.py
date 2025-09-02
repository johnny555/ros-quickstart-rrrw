#!/usr/bin/env python3
"""
Simple test runner shortcuts for common testing scenarios.
"""

import os
import sys
import subprocess


def run_pytest_command(args):
    """Run pytest with the given arguments."""
    test_dir = os.path.dirname(__file__)
    cmd = ['python3', '-m', 'pytest'] + args + [test_dir]
    
    print(f"Running: {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


def main():
    """Main entry point with simple command shortcuts."""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 test_runner.py unit         # Run unit tests")
        print("  python3 test_runner.py integration  # Run integration tests")
        print("  python3 test_runner.py all          # Run all tests")
        print("  python3 test_runner.py list         # List all tests")
        print("  python3 test_runner.py nav          # Run navigation tests only")
        print("  python3 test_runner.py rviz         # Run RViz tests only")
        return 1
    
    command = sys.argv[1].lower()
    
    if command == 'unit':
        return run_pytest_command(['-v', '-m', 'unit'])
    elif command == 'integration':
        return run_pytest_command(['-v', '-m', 'integration'])
    elif command == 'all':
        return run_pytest_command(['-v'])
    elif command == 'list':
        return run_pytest_command(['--collect-only', '-q'])
    elif command == 'nav':
        return run_pytest_command(['-v', '-k', 'navigation'])
    elif command == 'rviz':
        return run_pytest_command(['-v', '-k', 'rviz'])
    else:
        print(f"Unknown command: {command}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
