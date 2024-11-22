#!/bin/bash

source /opt/ros/jazzy/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/launch/dragger.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/launch/launch_sim_nav.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/launch/dragger_slam.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/launch/dragger.launch.py  --show-arguments
#ros2 launch /home/ros/launch/dragger.launch.py
