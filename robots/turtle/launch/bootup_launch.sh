#!/bin/bash

#
# See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1
#
# This file goes to ~/launch folder on Create 1 Turtlebot Raspberry Pi
# Use it to start the on-board part of the robot.
# On the Desktop machine type:
#     cd ~/robot_ws; colcon build; ros2 launch articubot_one turtle.launch.py
#

cd /home/ros/launch
source /opt/ros/jazzy/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/launch/myturtle.py

