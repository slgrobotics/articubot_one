#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# See /opt/ros/jazzy/lib/python3.12/site-packages/nav2_simple_commander/example_waypoint_follower.py
#     https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander/nav2_simple_commander
#     https://automaticaddison.com/how-to-send-waypoints-to-the-ros-2-navigation-stack-nav-2/
#
# Run this script:
#  cd ~/robot_ws; colcon build; ros2 run articubot_one xy_waypoint_follower.py

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.

Note: When nodes are started, X, Y and Z are set to zero (see initial_pose.pose.position below)
      The goal_pose.pose.position.* coordinates are relative to initial pose.
      This script is good for sim, but for real use case the robot must be placed in the same spot 
      when its ROS2 software is started
"""

"""
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
"""
def main():
 
    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    '''
    # Set our demo's initial pose if necessary
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    '''
    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    # https://github.com/ros-navigation/navigation2/issues/2283
    #navigator.waitUntilNav2Active()  # must use AMCL, will wait for amcl/get_state service available
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Set the robot's goal poses
    goal_poses = []

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 5.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    # additional goals can be appended
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 5.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses) ???

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                print('Timeout 10 minutes: canceling task')
                navigator.cancelTask()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=300.0):
                print('Timeout 5 minutes: Executing alternative destination waypoint')
                goal_pose_alt = PoseStamped()
                goal_pose_alt.header.frame_id = 'map'
                goal_pose_alt.header.stamp = now.to_msg()
                goal_pose_alt.pose.position.x = 0.0
                goal_pose_alt.pose.position.y = 0.0
                goal_pose_alt.pose.position.z = 0.0
                goal_pose_alt.pose.orientation.x = 0.0
                goal_pose_alt.pose.orientation.y = 0.0
                goal_pose_alt.pose.orientation.z = 0.0
                goal_pose_alt.pose.orientation.w = 1.0
                goal_poses = [goal_pose_alt]
                nav_start = now
                navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()

