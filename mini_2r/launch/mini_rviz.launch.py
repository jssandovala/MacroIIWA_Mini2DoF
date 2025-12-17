from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='mini_2r').find('mini_2r')
    
    urdf_file = PathJoinSubstitution([pkg_share, 'description', 'urdf', 'rrbot4.urdf.xacro'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('mini_2r'), 'rviz', 'mini_InteractiveMarker.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
        ],
    )

    
    mini_pos = Node(
        package='mini_2r',
        executable='interactive_marker_mini.py',
        name='_interactive_marker_mini',
        output='screen',
    )

    

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
            remappings=[
                ('joint_states', 'RR/joint_states'),
            ],

        ),
  
        rviz_node,
        #mini_pos,
    ])