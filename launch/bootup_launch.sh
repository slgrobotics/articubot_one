#!/bin/bash

cd /home/ros/launch
source /opt/ros/jazzy/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/launch/dragger.launch.py

