from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Reusable helpers
from articubot_one.launch_utils.helpers import (
    include_launch,
    namespace_wrap,
)

#
# Generate launch description for robot localizers
#
# Localizers are SLAM and/or EKF nodes that provide robot pose estimates
# to be used by navigation stack (Nav2) and other components.
# They can be robot-specific, and that's why we have this separate launch file.
#

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from <robot_model>.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot specific files reside under "robots" directory - dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    # Optional map file for localizers that support it (map_server, amcl)
    map_file = LaunchConfiguration('map', default='')

    # Use LaunchConfigurationEquals conditions to conditionally include localizers
    # based on the 'localizer_type' launch argument
    localizer_actions = [
        include_launch(
            package_name,
            ['launch', 'localizers', 'slam_toolbox.launch.py'],
            {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'robot_model': robot_model,
                'map': map_file
            },
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localizer_type'), 'slam_toolbox'))
        ),
        include_launch(
            package_name,
            ['launch', 'localizers', 'cartographer.launch.py'],
            {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'robot_model': robot_model,
                'map': map_file
            },
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localizer_type'), 'cartographer'))
        ),
        include_launch(
            package_name,
            ['launch', 'localizers', 'map_server.launch.py'],
            {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'robot_model': robot_model,
                'map': map_file
            },
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localizer_type'), 'map_server'))
        ),
        include_launch(
            package_name,
            ['launch', 'localizers', 'map_server_tf.launch.py'],
            {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'robot_model': robot_model,
                'map': map_file
            },
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localizer_type'), 'map_server_tf'))
        ),
        include_launch(
            package_name,
            ['launch', 'localizers', 'amcl.launch.py'],
            {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'robot_model': robot_model,
                'map': map_file
            },
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localizer_type'), 'amcl'))
        ),
    ]


    # Multi-robot safe: wrap everything under the namespace
    robot_localizers = namespace_wrap(namespace, localizer_actions)

    # ==========================
    # Final LaunchDescription
    # ==========================
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for drive nodes'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        DeclareLaunchArgument(
            'robot_model',
            default_value='',
            description='Robot model (e.g., seggy, plucky, dragger, turtle)'
        ),

        DeclareLaunchArgument(
            'localizer_type',
            default_value='slam_toolbox',
            choices=['slam_toolbox', 'cartographer', 'map_server', 'map_server_tf', 'amcl'],
            description='Localizer type to use: slam_toolbox, cartographer, map_server, or amcl'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map YAML file for map_server and amcl localizers (optional)'
        ),

        LogInfo(msg=[
            '============ starting LOCALIZERS  namespace="', namespace,
            '"  use_sim_time=', use_sim_time, '  robot_model=', robot_model,
            '  localizer_type=', LaunchConfiguration('localizer_type'),
            '  map=', map_file
        ]),

        robot_localizers
    ])
