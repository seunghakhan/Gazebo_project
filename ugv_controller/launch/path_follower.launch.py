from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        DeclareLaunchArgument('path_file', default_value='/home/han/PX4/ugv_final/ws_final/src/ugv_controller/ugv_wp.csv'),
        DeclareLaunchArgument('acceleration_limit', default_value='1.5'),
        DeclareLaunchArgument('speed_state_1', default_value='4.5'),
        DeclareLaunchArgument('speed_state_2', default_value='6.0'),
        DeclareLaunchArgument('speed_state_3', default_value='10.0'),
        DeclareLaunchArgument('speed_state_4', default_value='4.5')
    ]

    params = {
        'path_file': LaunchConfiguration('path_file'),
        'acceleration_limit': LaunchConfiguration('acceleration_limit'),
        'speed_state_1': LaunchConfiguration('speed_state_1'),
        'speed_state_2': LaunchConfiguration('speed_state_2'),
        'speed_state_3': LaunchConfiguration('speed_state_3'),
        'speed_state_4': LaunchConfiguration('speed_state_4')
    }

    return LaunchDescription(args + [
        Node(
            package='ugv_controller',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[params]
        )
    ])

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('path_file', default_value='path_ugv_wp.csv'),
#         DeclareLaunchArgument('max_speed_m1', default_value='4.0'),
#         DeclareLaunchArgument('max_speed_m2', default_value='4.0'),
#         DeclareLaunchArgument('max_speed_m3', default_value='6.0'),
#         DeclareLaunchArgument('max_speed_m4', default_value='4.0'),
#         DeclareLaunchArgument('arrival_threshold_m1', default_value='0.5'),
#         DeclareLaunchArgument('arrival_threshold_m2', default_value='0.7'),
#         DeclareLaunchArgument('arrival_threshold_m3', default_value='1.0'),
#         DeclareLaunchArgument('arrival_threshold_m4', default_value='0.5'),

#         Node(
#             package='ugv_controller',
#             executable='path_follower_node',
#             name='path_follower',
#             output='screen',
#             parameters=[{
#                 'path_file': LaunchConfiguration('path_file'),
#                 'max_speed_m1': LaunchConfiguration('max_speed_m1'),
#                 'max_speed_m2': LaunchConfiguration('max_speed_m2'),
#                 'max_speed_m3': LaunchConfiguration('max_speed_m3'),
#                 'max_speed_m4': LaunchConfiguration('max_speed_m4'),
#                 'arrival_threshold_m1': LaunchConfiguration('arrival_threshold_m1'),
#                 'arrival_threshold_m2': LaunchConfiguration('arrival_threshold_m2'),
#                 'arrival_threshold_m3': LaunchConfiguration('arrival_threshold_m3'),
#                 'arrival_threshold_m4': LaunchConfiguration('arrival_threshold_m4')
#             }]
#         )
#     ])
