#!/usr/bin/env python3
"""
Simple TurtleBot4 launch file that starts:
- Gazebo Harmonic
- Robot description for TurtleBot4
- ROS2 control with joint state publisher and diff_drive controller
- Spawns robot based on robot description (minimal setup)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use simulation time'),
    DeclareLaunchArgument('world', default_value='april_maze',
                          description='Gazebo world file (without .sdf extension)'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='TurtleBot4 model type'),
    DeclareLaunchArgument('x', default_value='0.0',
                          description='X position of robot spawn'),
    DeclareLaunchArgument('y', default_value='0.0',
                          description='Y position of robot spawn'),
    DeclareLaunchArgument('z', default_value='0.0',
                          description='Z position of robot spawn'),
    DeclareLaunchArgument('yaw', default_value='0.0',
                          description='Yaw orientation of robot spawn'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Name of the robot in simulation'),
    DeclareLaunchArgument('spawn_apriltags', default_value='false',
                          choices=['true', 'false'],
                          description='Spawn AprilTag panels model into the world'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Launch RViz2 with explorer bot configuration'),
    DeclareLaunchArgument('use_slam', default_value='true',
                          choices=['true', 'false'],
                          description='Enable SLAM toolbox for mapping'),
    DeclareLaunchArgument('use_nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Enable Nav2 navigation stack')
]


def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_simple_commander = get_package_share_directory('simple_commander')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    spawn_apriltags = LaunchConfiguration('spawn_apriltags')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')

    # Controller configuration file
    controller_config = PathJoinSubstitution([
        pkg_simple_commander,
        'config',
        'diff_drive_controller.yaml'
    ])

    # Set Gazebo resource paths to include TurtleBot4 meshes and models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            # Put april_world ahead so its worlds/models are preferred
            os.path.join(get_package_share_directory('april_world'), 'worlds'),
            os.path.join(get_package_share_directory('april_world'), 'models'),
            get_package_share_directory('april_world'),
            # Also include simple_commander resources
            os.path.join(pkg_simple_commander, 'worlds'),
            os.path.join(pkg_simple_commander, 'models'),
            pkg_simple_commander,
            # Include system ROS packages for irobot_create_description meshes
            '/opt/ros/jazzy/share',
            # Existing resources
            os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
        ])
    )

    # Robot description using our custom TurtleBot URDF
    robot_description_content = Command([
        'xacro', ' ',
        PathJoinSubstitution([pkg_simple_commander, 'urdf', 'turtlebot.urdf.xacro']),
    ])

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments=[
            ('gz_args', [world, '.sdf', ' -r', ' -v 4'])
        ]
    )

    # Robot State Publisher - publishes robot description and transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content}
        ]
    )

    # Controller Manager - manages ros2_control controllers (starts after robot_state_publisher)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            controller_config
        ],
        output='screen'
    )

    # Joint State Publisher - publishes joint states
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-topic', 'robot_description'
        ],
        output='screen'
    )


    # Combined ROS-Gazebo bridges for simulation time, cmd_vel, odometry, and joint states
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridges',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ['/world/', world, '/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points', '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']
        ],
        output='screen'
        , remappings=[
            (['/world/', world, '/model/', robot_name, '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points'], '/points')
        ]
    )

    # Diff Drive Controller - enables differential drive control
    control_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_drive_controller'],
        output='screen'
    )
    # RQT Robot Steering - GUI for teleoperation
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        arguments=['--force-discover'],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
            ('/odom', '/diff_drive_controller/odom')
        ]
    )

    # SLAM Toolbox - for mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                pkg_simple_commander,
                'config',
                'slam_toolbox.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_slam),
        remappings=[
            ('/scan', '/scan_converted')
        ]
    )

    # Pointcloud to Laserscan converter - convert RGBD pointcloud to laser scan for SLAM
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_frame': 'base_link'},
            {'transform_tolerance': 0.01},
            {'min_height': -0.5},
            {'max_height': 2.0},
            {'angle_min': -1.5708},  # -90 degrees
            {'angle_max': 1.5708},   # 90 degrees
            {'angle_increment': 0.0087},  # ~0.5 degrees
            {'scan_time': 0.3333},
            {'range_min': 0.45},
            {'range_max': 4.0},
            {'use_inf': True},
            {'inf_epsilon': 1.0}
        ],
        remappings=[
            ('/cloud_in', '/points'),
            ('/scan', '/scan_converted')
        ]
    )

    # Nav2 navigation stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'), 
                'launch', 
                'navigation_launch.py'
            ])
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('slam', 'False'),  # We're using slam_toolbox separately
            ('map', ''),  # No pre-built map needed for SLAM
        ],
        condition=IfCondition(use_nav2)
    )

    # RViz2 with custom configuration
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_simple_commander,
            'rviz',
            'explorer_bot.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(control_spawner)
    ld.add_action(rqt_robot_steering)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(slam_toolbox)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz2)
    
    return ld
