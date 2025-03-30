from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
         package='potMap',
         namespace='potMap_launch',
         executable='potMap_node',
      )

   ])