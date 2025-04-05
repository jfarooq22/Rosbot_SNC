import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    # Path to nav2 params file (in your package)
    nav2_params_file = os.path.join(
        get_package_share_directory('aiil_rosbot_demo'),
        'config',
        'navigation_pro3.yaml'
    )

    # External Nodes (from system packages)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('rosbot_search_nav'),
            'launch',
            'nav.launch.py'
        )
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'params_file': nav2_params_file,
        'slam': 'true'
    }.items()
)

    # Your custom nodes (in your package)
    navigation_node = Node(
        package='rosbot_search_nav',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    hazard_node = Node(
        package='rosbot_search_nav',
        executable='hazard_detector_node',
        name='hazard_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tracking_node = Node(
        package='rosbot_search_nav',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_sim_time,

        # Core system components
        slam_node,

        # Slight delay to stabilize TF before launching Nav2
        TimerAction(period=5.0, actions=[nav2_launch]),

        # Slight delay to ensure SLAM and Nav2 are fully ready
        TimerAction(period=7.0, actions=[
            navigation_node,
            hazard_node,
            tracking_node
        ])
    ])
