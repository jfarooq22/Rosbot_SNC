from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    explore_params = os.path.join(
        get_package_share_directory('explore_lite'),
        'config',
        'params_costmap.yaml'  
    )

    return LaunchDescription([
        TimerAction(
            period=2.0,
            actions=[Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                remappings=[
                    ('/costmap_topic', '/global_costmap/costmap'),
                    ('/costmap_updates_topic', '/global_costmap/costmap_updates'),
                ],
                parameters=[
                    explore_params
                ]
            )]
        ),

        TimerAction(
            period=5.0,
            actions=[Node(
                package='rosbot_search_nav',
                executable='navigation_node',
                name='navigation_node',
                output='screen',
                parameters=[{
                    'use_sim_time': False 
                }]
            )]
        )
    ])
