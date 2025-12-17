from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    default_urdf_path = '/home/amir/ros2_ws/src/mini_2r/description/urdf/mini.urdf'
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
    )
      
    mini_impedance_sim = Node(
        package='mini_2r',
        executable='mini_impedance_sim',
        name='mini_impedance_sim',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path')
        }]
    )
    
    return LaunchDescription([
        urdf_path_arg,
        mini_impedance_sim
    ])
