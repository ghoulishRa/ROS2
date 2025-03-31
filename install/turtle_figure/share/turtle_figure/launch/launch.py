import os
from launch import LaunchDescription
import launch
from launch_ros.actions import Node
#from ament_index_python.packages import get_package_share_directory #for parameters

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        #namespace='turtlesim1',
        executable='turtlesim_node',
        output = 'screen'
      )
    turtle_figure_node = Node(
        package='turtle_figure',
        #namespace='turtle_figure_launch',
        executable='turtleFigureNode'
      )
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output = 'screen'

    )
    ld = LaunchDescription([turtlesim_node, turtle_figure_node, rqt_graph_node])
    return ld