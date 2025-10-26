
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node

# Based on /opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py
# This combines map_server and AMCL for proper localization

def generate_launch_description():

    # Get the launch directory
    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)
    robot_path = os.path.join(package_path, 'robots', 'stingray')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_yaml_file_default = os.path.join(package_path, 'assets', 'maps','empty_map.yaml')
    params_file_default = os.path.join(robot_path, 'config', 'nav2_params.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=map_yaml_file_default, description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_default,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    start_localization_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=namespace,
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[{'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=namespace,
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'autostart': True}, 
                    {'node_names': ['map_server', 'amcl']}
                ],
            ),
        ]
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localization nodes
    ld.add_action(start_localization_nodes)

    return ld


