from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value='path_ugv_wp.csv'),
        DeclareLaunchArgument('acceleration_limit', default_value='1.5'),
        DeclareLaunchArgument('speed_state_1', default_value='4.5'),
        DeclareLaunchArgument('speed_state_2', default_value='6.0'),
        DeclareLaunchArgument('speed_state_3', default_value='10.0'),
        DeclareLaunchArgument('speed_state_4', default_value='4.5'),

        Node(
            package='ugv_controller',
            executable='new_path_follower_node',
            name='new_path_follower',
            output='screen',
            parameters=[{
                'path_file': LaunchConfiguration('path_file'),
                'acceleration_limit': LaunchConfiguration('acceleration_limit'),
                'speed_state_1': LaunchConfiguration('speed_state_1'),
                'speed_state_2': LaunchConfiguration('speed_state_2'),
                'speed_state_3': LaunchConfiguration('speed_state_3'),
                'speed_state_4': LaunchConfiguration('speed_state_4')
            }],
            arguments=['--ros-args', '--log-level', 'INFO']  
        )
    ])
