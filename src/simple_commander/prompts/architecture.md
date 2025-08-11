# Architecture and Tech Choices 

We are using ROS 2, with Python. It's `rclpy` library. 

When making a project, use ament-cmake instead of ament-python and use CMake instead of setup.py. 

Use standard ROS message types where ever possible.

Design nodes to have a single purpose.

Orchestrate the system using Behavior Trees. 

This repo should implement atleast one ros2 package named `explorer_bot` at `src/explorer_bot`

We are using ROS 2 jazzy.

# Standard ROS Setup 

This is a workspace folder. So all packages should be inside a `src` folder. This means that any new packages would have their root at `src/<package_name>/` .

When building, always build with `colcon build --merge-install --symlink-install` flags. 

Use `TwistStamped` instead of `Twist` message data types.

Create parameter `yaml` files instead of passing parameters to nodes in launch files.

## External Packages 

We are using the turtlebot4 example worlds in the repos located at `/workspace/src/turtlebot4_simulator`. 


## MultiThreading

When making a ROS 2 node, always use a `MultiThreadedExecutor` and a `ReentrentCallbackGroup` unless explicitly needed otherwise. 

For example:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.reentrant_cb_group = ReentrantCallbackGroup()

        # Timer in a reentrant callback group
        self.timer_reentrant = self.create_timer(1.0, self.timer_reentrant_callback, callback_group=self.reentrant_cb_group)

    def timer_reentrant_callback(self):
        self.get_logger().info('Reentrant timer callback triggered.')
        time.sleep(2) # Simulate long processing
        self.get_logger().info('Reentrant timer callback finished.')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File Style 

When returning the `LaunchDescription` in a launch file, simply create the LaunchDescription object and pass it a list of actions. For example 

```python

return LaunchDescription([
    gz_server,
    gz_client, 
    robot_description,
])
```


# Code Style Guide 

If possible, use a functional approach (pure functions with no side-effects).

Provide an overview comment for each code file. Instead of simply referencing the milestone or features, actually list the features that are implemented in the file.

Don't use python `exec` or `subprocess` to execute scripts on the command line. It is nearly always possible to do it another way such as instantiating a ros node or including a ros launch file instead. 

Avoid the use of keywords that are part of the `ros2` cli in file names, unless they are to be used by that cli. For example, don't name a script `launch_*.py` which is designed to be run with `ros2 run <package> launch_*.py`, and don't name a launch file starting with `run`, i.e. `run_<something>.launch.py` . But its ok for a launch file to end with `<something>.launch.py`. 

When commenting a function or method don't create a comment that simply restates the name of the function or method. If the name is descriptive enough then leave it without a comment.
