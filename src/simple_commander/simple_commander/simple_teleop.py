#!/usr/bin/env python3

"""
Simple teleop test script for the TurtleBot4 simulation.
Use keyboard to control the robot: w/s for forward/backward, a/d for turning, space to stop.
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.publisher = self.create_publisher(Twist, '/turtlebot4/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        self.get_logger().info('Simple TurtleBot4 Teleop started!')
        self.get_logger().info('Controls: w=forward, s=backward, a=left, d=right, space=stop, q=quit')
    
    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish a Twist message"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
    
    def run(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    # Forward
                    self.publish_twist(self.linear_speed, 0.0)
                    self.get_logger().info('Moving forward')
                    
                elif key == 's':
                    # Backward
                    self.publish_twist(-self.linear_speed, 0.0)
                    self.get_logger().info('Moving backward')
                    
                elif key == 'a':
                    # Turn left
                    self.publish_twist(0.0, self.angular_speed)
                    self.get_logger().info('Turning left')
                    
                elif key == 'd':
                    # Turn right
                    self.publish_twist(0.0, -self.angular_speed)
                    self.get_logger().info('Turning right')
                    
                elif key == ' ':
                    # Stop
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().info('Stopped')
                    
                elif key == 'q':
                    # Quit
                    self.get_logger().info('Quitting teleop')
                    break
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # Stop the robot
            self.publish_twist(0.0, 0.0)
            self.get_logger().info('Teleop stopped')


def main(args=None):
    rclpy.init(args=args)
    
    teleop = SimpleTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
