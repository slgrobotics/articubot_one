import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

# See https://github.com/ros-tooling/topic_tools/tree/jazzy?tab=readme-ov-file#relayfield
#     https://github.com/gazebosim/ros_gz/issues/586
#     https://github.com/ros/ros_comm/pull/639
#     https://github.com/ros-tooling/topic_tools/tree/jazzy/topic_tools

def generate_launch_description():


    # Note: radiation_type ULTRASOUND=0 INFRARED=1
    #   range: m.ranges[0],  - this doesn't work, See https://github.com/ros-tooling/topic_tools/blob/jazzy/topic_tools/topic_tools/relay_field.py


    sonar_F_L_node = Node(
                package='topic_tools',
                executable='relay_field',
                name='sonar_F_L_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['/sonar_F_L_sim', '/sonar_F_L', 'sensor_msgs/msg/Range', \
                            '{ \
                                header: { \
                                        stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                                        frame_id: m.header.frame_id \
                                      }, \
                                max_range: m.range_max, \
                                min_range: m.range_min, \
                                radiation_type: 0, \
                                range: "m.ranges[3]", \
                                variance: 0.01 \
                              } \
                         ']
            )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(sonar_F_L_node)

    return ld
