from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import rclpy


def generate_launch_description():


    interactive_marker = Node(
        package='kuka_mini',
        executable='kuka_mini_marker.py',
        name='interactive_marker_mini',
        output='screen',
    )


    iiwa_vel_node = Node(
        package='kuka_mini',
        executable='iiwa_vel_telop_real.py',
        name='iiwa_vel_telop_real',
        output='screen',
    )

    mini_control_=Node(
        package='kuka_mini',
        executable='mini_impedance_trajectory_world',
        name='mini_2r_active_real',
        output='screen',
    )

    nodes=[ 
        iiwa_vel_node,
        mini_control_,
        interactive_marker,
        ]


    return LaunchDescription(nodes)