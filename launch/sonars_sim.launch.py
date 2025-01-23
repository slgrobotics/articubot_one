import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

# See /opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py

def generate_launch_description():


    sonar_F_L_node = Node(
                package='topic_tools',
                executable='relay_field',
                name='sonar_F_L_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['/sonar_F_L', '/sonar_F_L_Range', 'sensor_msgs/msg/Range', \
                            '{header: {stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                              frame_id: m.header.frame_id}}']
            )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(sonar_F_L_node)

    return ld
