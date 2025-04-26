from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/israamaciaas/ros2_ws/custom_maps/test3.yaml'
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        Node (
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
    