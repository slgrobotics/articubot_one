from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    # -------------------------------------------------------
    # "ekf_imu_odom" is needed, providing "a valid transform from your configured odom_frame to base_frame"
    # it does IMU + ODOM fusing. Publishes /odometry/local and TF odom->base_link
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see slam_toolbox_params.yaml
    # -------------------------------------------------------

    ekf_imu_odom = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    # ==========================
    #
    # You generally want either:
    #   - map_server (GPS)
    #   - slam_toolbox (LIDAR)
    #
    # Note: SLAM Toolbox seems to be very ineffective outdoors in my experiments.
    #       The Navsat transform keeps drifting even with good non-RTK GPS signal.
    #       The LIDAR range is limited in sunshine, and there are few features to lock on to.
    #       Therefore, for outdoor use I am using only GPS-based localization with map server.
    #       Adding a previously saved map to the map server is possible.
    #
    # See https://github.com/slgrobotics/outdoors_loc_nav
    #     https://github.com/slgrobotics/articubot_one/wiki/Conversations-with-Overlords#question-6
    #

    map_yaml_file = PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'maps', 'empty_map.yaml'])   # this is similar to the "outdoors_loc_nav" default

    outdoors_loc_nav = include_launch(
        "outdoors_loc_nav",
        ['launch', 'outdoors_loc.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'localizer': 'slam_toolbox',  # 'map_server' or 'slam_toolbox' or 'cartographer' or 'amcl'  Default: 'map_server'
            'map': map_yaml_file,         # optional map file for amcl or map_server
            #'map': '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml',
        }
    )

    # Multi-robot safe: wrap everything under the namespace
    localizer_actions = [
        ekf_imu_odom,
        outdoors_loc_nav, # external package preferred for outdoors
    ]

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
