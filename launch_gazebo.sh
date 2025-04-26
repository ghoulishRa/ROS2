
#/bin/bash
export TURTLEBOT3_MODEL=burger
#ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=0.0 y_pose:=0.0
ros2 launch turtlebot3_gazebo empty_world.launch.py x_pose:=0.0 y_pose:=0.0
