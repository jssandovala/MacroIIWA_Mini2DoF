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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rrbot'
        ],
        output='screen'
    )
    
   
    # Load controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager','/controller_manager'], 

    )
    
    load_joint_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_effort_controller', '--controller-manager','/controller_manager'],
        
    )
    
    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                load_joint_state_broadcaster,
                load_joint_effort_controller,
            ],
        )
    )
    
    # Node to control the 2R robot with interactive markers
    interactive_marker = Node(
        package='mini_2r',
        executable='interactive_marker_mini_sim.py',
        name='_interactive_marker_mini_sim',
        output='screen',
    )

    mini_speed_impedance = Node(
        package='mini_2r',
        executable='mini_speed_and_impedance.py',
        name='_mini_speed_and_impedance',
        output='screen',
    )

    # Include  the impedance control law
    mini_impedance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mini_2r'),  
                'launch',
                'mini_impedance_sim.launch.py'      
            ])
        ]),
    )



    publisher_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    nodes=[ gazebo,
        spawn_entity,
        rviz_node,
        load_controllers_after_spawn,
        interactive_marker,
        mini_impedance_launch,
        #mini_speed_impedance,
        publisher_robot
        ]


    return LaunchDescription(nodes)