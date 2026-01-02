#!/bin/bash

#
# See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1
#
# This file goes to ~/launch folder on Stingray robot's Raspberry Pi
# Use it to start the on-board part of the robot.
# On the Desktop machine type:
#     cd ~/robot_ws; colcon build; ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
#

source /opt/ros/jazzy/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/robots/stingray/launch/stingray.launch.py

# Show arguments example:
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/stingray/launch/stingray.launch.py  --show-arguments
