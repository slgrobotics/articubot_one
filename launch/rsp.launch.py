import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

import xacro


def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Produce the URDF string from collection of .xacro's:
    xacro_file = os.path.join(package_path,'description','robot.urdf.xacro')

    robot_description_sdf = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace='/',
        parameters=[{'robot_description': ParameterValue(robot_description_sdf, value_type=str),
                    #'publish_frequency' : 5.0,  - this has no effect. The topic is published on demand.
                    'use_sim_time': use_sim_time
                    }]
    )

    # Optional nodes to interact with joints from the RViz2:
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='/',
        output=['screen']
    )

    # For publishing and controlling the robot pose in the pop-up GUI:
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace='/',
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

        node_robot_state_publisher,

        # Either of these two will override /joint_states topic with zeroes.
        # Normally, joint_state_broadcaster publishes it.
        #node_joint_state_publisher,
        #node_joint_state_publisher_gui
    ])
