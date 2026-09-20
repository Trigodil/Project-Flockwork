"""Stage 1 launch file. Starts Gazebo with the single-drone world, bridges
Gazebo topics to ROS 2, and starts the PID controller node.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_swarm_control = get_package_share_directory('swarm_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_swarm_control, 'worlds', 'single_drone.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Bridges the Gazebo velocity command and odometry topics to ROS 2,
    # remapped to plain /cmd_vel and /odom so the controller node stays
    # sim-agnostic.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/x3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            ('/X3/gazebo/command/twist', '/cmd_vel'),
            ('/model/x3/odometry', '/odom'),
        ],
        output='screen',
    )

    controller = Node(
        package='swarm_control',
        executable='single_drone_controller_node',
        name='single_drone_controller',
        output='screen',
    )

    return LaunchDescription([gz_sim, bridge, controller])
