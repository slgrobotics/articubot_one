#!/bin/bash

cd /home/ros/launch
source /opt/ros/humble/setup.bash
source /home/ros/plucky_ws/install/setup.bash

ros2 launch /home/ros/launch/plucky.launch.py

