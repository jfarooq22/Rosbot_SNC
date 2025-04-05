from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot_search_nav',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        Node(
            package='rosbot_search_nav',
            executable='hazard_detector_node',
            name='hazard_detector_node',
            output='screen'
        ),
        Node(
            package='rosbot_search_nav',
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        ),
    ])
