#!/usr/bin/env bash

#
# Clone all repositories needed for articubot_one workspace.
#
# put this script in the source root of your workspace (~/robot_ws/src)
# and run it to update all git repositories in the sub-directories:
#
#     mkdir -p ~/robot_ws/src
#     cd ~/robot_ws/src
#     ./clone_all.sh

set -x

git clone https://github.com/slgrobotics/articubot_one.git -b lyrical
git clone https://github.com/slgrobotics/outdoors_loc_nav.git -b lyrical
git clone https://github.com/slgrobotics/ros_battery_monitoring.git -b lyrical
git clone https://github.com/slgrobotics/rviz_attitude_plugin.git
git clone https://github.com/slgrobotics/scan_to_range.git
git clone https://github.com/slgrobotics/slg_bt_plugins.git
git clone https://github.com/slgrobotics/dock_detector.git
git clone https://github.com/slgrobotics/dock_detector_interfaces.git
git clone https://github.com/slgrobotics/dock_detector_nav2.git

set +x
