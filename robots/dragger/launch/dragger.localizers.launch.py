from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Reusable helpers
from articubot_one.launch_utils.launch_utils import (
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

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    # ==========================
    # most used localizer - SLAM Toolbox
    slam_toolbox_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'mapper_params.yaml'
    ])

    slam_toolbox = include_launch(
        "slam_toolbox",
        ['launch', 'online_async_launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'slam_params_file': slam_toolbox_params_file
        }
    )

    # ekf_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    ekf_localizer = include_launch(
        package_name,
        ['launch', 'ekf_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    # ==========================
    # optional alternative - Cartographer
    cartographer = include_launch(
        package_name,
        ['launch', 'cartographer.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'robot_model': robot_model
        }
    )

    # ==========================
    # optional alternative - Map Server
    # Map server is convenient when used with GPS and an empty map, for obstacle avoidance.
    #map_yaml_file = PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'maps', 'empty_map.yaml'])   # this is default anyway
    map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = include_launch(
        package_name,
        ['launch', 'map_server.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            # default map (empty) OR:
            # 'map': map_yaml_file,
        }
    )

    # ==========================
    # debugging helper - rarely needed
    # for experiments: a bad alternative to ekf_localizer for slam_toolbox - static transform publisher
    tf_localizer = Node(
        package="tf2_ros",
        namespace=namespace,
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    # ==========================

    navsat_localizer = include_launch(
        package_name,
        ['launch', 'dual_ekf_navsat.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    #
    # Group all localizers —
    # You generally want either:
    #   - map_server (GPS)
    #   - slam_toolbox (LIDAR)
    # not both at once.
    #
    localizer_actions = [
        navsat_localizer,
        map_server,     # localization is left to GPS
        # slam_toolbox, # localization via LIDAR — enable if desired
        # ekf_localizer,
        # cartographer,
        # tf_localizer, # debugging only
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

        LogInfo(msg=[
            '============ starting LOCALIZERS  namespace="', namespace,
            '"  use_sim_time=', use_sim_time, '  robot_model=', robot_model
        ]),

        robot_localizers
    ])
