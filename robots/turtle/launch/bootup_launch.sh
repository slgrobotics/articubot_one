#!/bin/bash

#
# See https://github.com/slgrobotics/articubot_one/wiki/MockBots-Initiative#set-up-your-robot
#
# This file goes in the ~/launch folder on the Turtle's Raspberry Pi
# Use it to start the on-board part of the robot.
#
# On the Workstation type:
#     cd ~/robot_ws; colcon build; ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
#

source /opt/ros/lyrical/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

ros2 launch articubot_one turtle.launch.py

# Show arguments example:
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/turtle/launch/turtle.launch.py  --show-arguments
