"""
Map Server Localizer Launch Wrapper
Provides static map serving for GPS-based navigation or obstacle avoidance with empty maps.
Convenient when used with GPS, does not provide pose estimation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')
    map_file = LaunchConfiguration('map', default='')

    # EKF localizer (needed for slam_toolbox to provide odom->base_link transform)
    ekf_localizer = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
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
        DeclareLaunchArgument('map', default_value='', description='Path to map YAML file for map_server (optional)'),

        LogInfo(msg=[
            '============ starting EKF + MAP SERVER LOCALIZER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  map=', map_file
        ]),

        ekf_localizer,
        map_server,
    ])
