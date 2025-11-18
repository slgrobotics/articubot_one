from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# See https://github.com/ros-tooling/topic_tools/tree/jazzy?tab=readme-ov-file#relayfield
#     https://github.com/gazebosim/ros_gz/issues/586
#     https://github.com/ros/ros_comm/pull/639
#     https://github.com/ros-tooling/topic_tools/tree/jazzy/topic_tools

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    # Note: radiation_type ULTRASOUND=0 INFRARED=1
    # See https://github.com/ros-tooling/topic_tools/blob/jazzy/topic_tools/topic_tools/relay_field.py

    sonar_F_L_node = Node(
                package='topic_tools',
                namespace=namespace,
                executable='relay_field',
                name='sonar_F_L_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['sonar_F_L_sim', 'sonar_F_L', 'sensor_msgs/msg/Range', \
                            '{ \
                                header: { \
                                        stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                                        frame_id: m.header.frame_id \
                                      }, \
                                field_of_view: 0.1, \
                                max_range: m.range_max, \
                                min_range: m.range_min, \
                                radiation_type: 0, \
                                range: "m.ranges[3]", \
                                variance: 0.01 \
                              } \
                         ']
            )

    sonar_F_R_node = Node(
                package='topic_tools',
                namespace=namespace,
                executable='relay_field',
                name='sonar_F_R_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['sonar_F_R_sim', 'sonar_F_R', 'sensor_msgs/msg/Range', \
                            '{ \
                                header: { \
                                        stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                                        frame_id: m.header.frame_id \
                                      }, \
                                field_of_view: 0.1, \
                                max_range: m.range_max, \
                                min_range: m.range_min, \
                                radiation_type: 0, \
                                range: "m.ranges[3]", \
                                variance: 0.01 \
                              } \
                         ']
            )

    sonar_B_L_node = Node(
                package='topic_tools',
                namespace=namespace,
                executable='relay_field',
                name='sonar_B_L_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['sonar_B_L_sim', 'sonar_B_L', 'sensor_msgs/msg/Range', \
                            '{ \
                                header: { \
                                        stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                                        frame_id: m.header.frame_id \
                                      }, \
                                field_of_view: 0.1, \
                                max_range: m.range_max, \
                                min_range: m.range_min, \
                                radiation_type: 0, \
                                range: "m.ranges[3]", \
                                variance: 0.01 \
                              } \
                         ']
            )

    sonar_B_R_node = Node(
                package='topic_tools',
                namespace=namespace,
                executable='relay_field',
                name='sonar_B_R_relay',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                arguments=['sonar_B_R_sim', 'sonar_B_R', 'sensor_msgs/msg/Range', \
                            '{ \
                                header: { \
                                        stamp: {sec: m.header.stamp.sec, nanosec: m.header.stamp.nanosec}, \
                                        frame_id: m.header.frame_id \
                                      }, \
                                field_of_view: 0.1, \
                                max_range: m.range_max, \
                                min_range: m.range_min, \
                                radiation_type: 0, \
                                range: "m.ranges[3]", \
                                variance: 0.01 \
                              } \
                         ']
            )


    # Note: topic_tools don't work since October 2025 update.
    #       Here is alternative approach using "scan_to_range" node.
    # See https://chatgpt.com/s/t_691cc863d70c81919fa279f4bcc2e06a
    #     https://chatgpt.com/s/t_691cc8351e648191a4cf0e7c5dd05428

    # ---------- FRONT RIGHT ----------
    sonar_F_R_node_ = Node(
            package="scan_to_range",
            executable="scan_to_range",
            namespace=namespace,
            name="scan_to_range_FR",
            parameters=[
                {
                    "input_topic": "sonar_F_R_sim",
                    "output_topic": "sonar_F_R",
                }
            ],
            output="screen",
        )

    # ---------- FRONT LEFT ----------
    sonar_F_L_node_ = Node(
            package="scan_to_range",
            executable="scan_to_range",
            namespace=namespace,
            name="scan_to_range_FL",
            parameters=[
                {
                    "input_topic": "sonar_F_L_sim",
                    "output_topic": "sonar_F_L",
                }
            ],
            output="screen",
        )

    # ---------- REAR RIGHT ----------
    sonar_B_R_node_ = Node(
            package="scan_to_range",
            executable="scan_to_range",
            namespace=namespace,
            name="scan_to_range_RR",
            parameters=[
                {
                    "input_topic": "sonar_B_R_sim",
                    "output_topic": "sonar_B_R",
                }
            ],
            output="screen",
        )

    # ---------- REAR LEFT ----------
    sonar_B_L_node_ = Node(
            package="scan_to_range",
            executable="scan_to_range",
            namespace=namespace,
            name="scan_to_range_RL",
            parameters=[
                {
                    "input_topic": "sonar_B_L_sim",
                    "output_topic": "sonar_B_L",
                }
            ],
            output="screen",
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    # using topic_tools relay_field nodes:
    #ld.add_action(sonar_F_L_node)
    #ld.add_action(sonar_F_R_node)
    #ld.add_action(sonar_B_L_node)
    #ld.add_action(sonar_B_R_node)

    # using scan_to_range nodes:
    ld.add_action(sonar_F_L_node_)
    ld.add_action(sonar_F_R_node_)
    ld.add_action(sonar_B_L_node_)
    ld.add_action(sonar_B_R_node_)

    return ld
