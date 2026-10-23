#!/bin/bash

source /opt/ros/lyrical/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

#aplay -D default:CARD=Set ~/wav/cat_meow.wav
#aplay ~/wav/cat_meow.wav

ros2 launch articubot_one seggy.launch.py
#ros2 launch articubot_one seggy.launch.py localizer_type:=map_server_tf
#ros2 launch articubot_one seggy.launch.py localizer_type:=amcl
#ros2 launch articubot_one seggy.launch.py localizer_type:=cartographer

#ros2 launch articubot_one dragger.launch.py
#ros2 launch articubot_one dragger.launch.py localizer_type:=slam_toolbox
#ros2 launch articubot_one dragger.launch.py localizer_type:=amcl
#ros2 launch articubot_one dragger.launch.py localizer_type:=cartographer

#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/seggy/launch/seggy_sim_nav.launch.py
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/seggy/launch/seggy_slam_toolbox.launch.py

# Show arguments example:
#ros2 launch /home/ros/robot_ws/src/articubot_one/robots/plucky/launch/plucky.launch.py  --show-arguments
