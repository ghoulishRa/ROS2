import os
from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('puzzlebot_sim'),
                        'urdf',
                        urdf_file_name
                        )

    with open(urdf_default_path, 'r') as infp:
        robot_description_content = infp.read()

    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'map']
                                )

    robot_state_publisher = Node (
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        name='robot_state_publisher',
        output="screen"
    )

    joint_pub = Node (
        package="act2_tf",
        executable="joint_pub"
    )

    rviz2 = Node (
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
        get_package_share_directory('act2_tf'), 'rviz', 'puzzlebot.rviz')],
        output='screen'
    )
    
    ld = LaunchDescription([static_transform_node, static_transform_node_2, robot_state_publisher, joint_pub, rviz2])
    return ld