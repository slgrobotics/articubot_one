#!/bin/bash

cd /home/ros/launch
source /opt/ros/humble/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/launch/dragger.launch.py
