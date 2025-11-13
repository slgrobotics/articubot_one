import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    # Build substitution-based paths so robot_model can be used at launch-time
    # Path to the robot xacro file (resolved at launch time)
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'description', 'robot.urdf.xacro'
    ])

    # Produce full XML/SDF description by invoking xacro on the substitution path
    robot_description_sdf = Command(['xacro', xacro_file, 'sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    # See https://github.com/ros/robot_state_publisher/tree/jazzy
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_sdf, value_type=str),
                    #'publish_frequency' : 5.0,  # Defaults to 20.0 Hz. Only affects non-static joints.
                    #'frame_prefix': '',
                    #'ignore_timestamp': 'false'
                    }]
    )

    # Optional nodes to interact with joints from the RViz2:
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        namespace=namespace,
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output=['screen']
    )

    # For publishing and controlling the robot pose in the pop-up GUI:
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        namespace=namespace,
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output=['screen'],
        # no need to supply SDF source here, it will be picked from topic /robot_description by default
        #arguments=[robot_description_sdf]
    )

    # Launch!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        LogInfo(msg=['============ starting ROBOT STATE PUBLISHER  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),
        LogInfo(msg=['xacro_file: ', xacro_file]),

        node_robot_state_publisher,

        # Either of these two will override /joint_states topic with zeroes.
        # Normally, joint_state_broadcaster publishes it.
        #node_joint_state_publisher,
        #node_joint_state_publisher_gui
    ])
