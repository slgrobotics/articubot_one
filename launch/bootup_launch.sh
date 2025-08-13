#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros_ws
colcon build
source /home/ubuntu/ros_ws/install/setup.bash
#aplay -D default:CARD=Set ~/wav/cat_meow.wav
aplay ~/wav/space.wav
ros2 launch /home/ubuntu/ros_ws/src/articubot_one/robots/stingray/launch/stingray.launch.py
#ros2 launch /home/ubuntu/ros_ws/src/articubot_one/robots/stingray/launch/stingray_sim_nav.launch.py
#ros2 launch /home/ubuntu/ros_ws/src/articubot_one/robots/stingray/launch/stingray_slam_toolbox.launch.py
# Show arguments example:
#ros2 launch /home/ubutntu/ros_ws/src/articubot_one/robots/stingray/launch/stingray.launch.py  --show-arguments
