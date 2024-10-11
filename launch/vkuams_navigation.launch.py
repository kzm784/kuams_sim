import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = os.path.join(get_package_share_directory(
        'kuams_sim'), 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'vkuams_samplemap.yaml'))

    param_dir = os.path.join(get_package_share_directory(
        'kuams_sim'), 'config')
    param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'navigation_param.yaml'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('kuams_sim'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('kuams_sim'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'navigation.rviz')

    # Path to the collision_monitor launch file
    collision_monitor_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_collision_monitor'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # Include the collision_monitor launch file with additional parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [collision_monitor_launch_file_dir, '/collision_monitor_node.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),
    ])
