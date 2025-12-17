from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import rclpy


def generate_launch_description():

    mini_position = Node(
        package='mini_2r',
        executable='mini_2r_position.py',
        name='kinematics_sim',
        output='screen',
    )

    mini_generator_trajectory_pub = Node(
        package='mini_2r',
        executable='generator_trajectory.py',
        name='trajectory_generator',
        output='screen',
    )


    # Include  the impedance control law
    mini_impedance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mini_2r'),  
                'launch',
                'mini_impedance_trajectory.launch.py'      
            ])
        ]),
    )

    interative_marker = Node(
        package='mini_2r',
        executable='interactive_marker_mini.py',
        name='interactive_marker_mini',
        output='screen',
    )

    nodes=[ 
        mini_generator_trajectory_pub,
        mini_impedance_launch,
        mini_position,
        #interative_marker,
        ]


    return LaunchDescription(nodes)