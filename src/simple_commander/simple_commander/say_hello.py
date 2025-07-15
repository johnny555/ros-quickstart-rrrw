#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SayHelloNode(Node):
    def __init__(self):
        super().__init__('say_hello_node')
        
        # Create publisher for chatter topic
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        # Create timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Track start time for 10-second duration
        self.start_time = time.time()
        self.duration = 10.0  # 10 seconds
        self.should_stop = False  # Flag to control when to stop
        
        self.get_logger().info(f'Say Hello Node started - will run for {self.duration} seconds')

    def timer_callback(self):
        # Check if duration has elapsed
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.duration:
            self.get_logger().info(f'{self.duration} seconds elapsed - shutting down')
            self.should_stop = True
            return
        
        # Publish info message
        self.get_logger().info('Hello!')
        
        # Publish to chatter topic
        msg = String()
        msg.data = 'hello'
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Published to chatter: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    
    node = SayHelloNode()
    
    try:
        # Use spin_once with timeout instead of spin
        while rclpy.ok() and not node.should_stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Only shutdown if ROS is still running
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()