import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    aiil_gazebo_dir = get_package_share_directory('aiil_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(aiil_gazebo_dir, 'config', 'navigation.yaml'),
            description='Path to the Nav2 YAML config file'
        ),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Whether to launch SLAM (true) or load a map (false)'
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Path to a static map (used if slam:=false)'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'slam': slam,
                'map': map_file,
            }.items()
        )
    ])
