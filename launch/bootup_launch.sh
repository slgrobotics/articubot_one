#!/bin/bash

source /opt/ros/jazzy/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

#aplay -D default:CARD=Set ~/wav/cat_meow.wav
#aplay ~/wav/cat_meow.wav

ros2 launch /home/ros/robot_ws/src/articubot_one/robots/plucky/launch/plucky.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/sim/launch/sim_nav.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/plucky/launch/slam_toolbox.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/plucky/launch/plucky.launch.py  --show-arguments
