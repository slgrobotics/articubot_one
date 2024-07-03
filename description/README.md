
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/diffdrive_arduino.git
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/slgrobotics/articubot_one.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ~/robot_ws/install/setup.bash

## Teleop:
~/teleop.sh &


##############################################################
# https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
# PX4 uses Gazebo Garden, ROS Humble- Fortress. 
sudo apt remove gz-garden
sudo apt remove gz-tools2
sudo apt autoremove
# Gazebo install:
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt install ros-humble-gazebo-ros-pkgs

##############################################################
# https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-gazebo

# in three terminals:
source ~/robot_ws/install/setup.bash

#  --- this runs Gazebo Fortress the standard way, empty. Keep it running.
export DISPLAY=sergeipwr.local:0.0
ros2 launch gazebo_ros gazebo.launch.py

#  --- this provided "robot_description" topic. Ensure fresh colcon build in ~/robot_ws
ros2 launch articubot_one rsp.launch.py use_sim_time:=true

#  --- this spawns robot "entity" (live model) in Gazebo, showing it. It finishes right away.
#  --- Use right click on world tree on the left pane to "delete" my_bot
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot


# from Gazebo standard launch:
# [gzserver-1] [INFO] [1719962794.009994255] [camera_controller]: Publishing camera info to [/camera/camera_info]
# [gzserver-1] [WARN] [1719962794.154366378] [rcl]: Found remap rule '~/out:=scan'. This syntax is deprecated. Use '--ros-args --remap ~/out:=scan' instead.

################################################################
## This launches whole set (Gazebo + robot):
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one launch_sim.launch.py
