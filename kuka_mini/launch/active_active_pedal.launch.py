from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import rclpy


def generate_launch_description():


    pedal_node = Node(
        package='joy',
        executable='joy_node',
        name='pedal',
        output='screen',
    )


    iiwa_vel_node = Node(
        package='kuka_mini',
        executable='iiwa_vel_real_final.py',
        name='iiwa_vel_real_final',
        output='screen',
    )

    mini_control_=Node(
        package='kuka_mini',
        executable='iiwa_mini_active_real',
        name='mini_2r_active_real',
        output='screen',
    )

    nodes=[ 
        pedal_node,
        iiwa_vel_node,
        mini_control_,
        ]


    return LaunchDescription(nodes)