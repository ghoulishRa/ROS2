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

    robot_state_publisher = Node (
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        name='robot_state_publisher',
        output="screen"
    )
     

    rviz2 = Node (
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
        get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
        output='screen'
    )

    joint_pub = Node(
        package="puzzlebot_sim",
        name="joint_pub",
        executable= "joint_pub",

    )
    ld = LaunchDescription([robot_state_publisher, joint_pub, rviz2])
    return ld