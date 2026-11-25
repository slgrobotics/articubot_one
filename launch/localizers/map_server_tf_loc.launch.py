"""
Map Server Localizer Launch Wrapper
Provides static map serving for GPS-based navigation or obstacle avoidance with empty maps.
Convenient when used with GPS, does not provide pose estimation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    default_map = PathJoinSubstitution([
        FindPackageShare(package_name), 'assets', 'maps', 'empty_map_tf.yaml'
    ])

    # 'map' - path to map YAML; omitted or empty uses packaged "empty_map_tf.yaml"

    requested_map = LaunchConfiguration('map', default=default_map)
    # An explicit empty argument overrides launch defaults; resolve it here so
    # the shared map server receives this localizer's ground-scale map.
    map_file = IfElseSubstitution(
        EqualsSubstitution(requested_map, ''),
        if_value=default_map,
        else_value=requested_map,
    )

    # Note: The EKF filter must be launched beforehand (in the Drive or Sensors launch file)
    # It is needed to provide the odom->base_link transform

    tf_map_odom = include_launch(
        package_name,
        ['launch', 'tf_map_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }
    )

    # Map server with map_server and map_saver params from config
    map_server = include_launch(
        package_name,
        ['launch', 'map_server.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'map': map_file,
            'params_file': PathJoinSubstitution([FindPackageShare(package_name), 'config', 'map_server_params.yaml']),
        }
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Path to map YAML; omitted or empty uses packaged empty_map_tf.yaml'
        ),

        LogInfo(msg=[
            '============ starting EKF + static TF + MAP SERVER LOCALIZER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  map=', map_file
        ]),

        tf_map_odom,
        map_server,
    ])
