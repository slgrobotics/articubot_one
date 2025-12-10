"""
Dragger-specific localizers launcher.
Includes the generic launch/localizers.launch.py with dragger defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch, namespace_wrap


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='dragger')
    map_file = LaunchConfiguration('map', default='') # can be '' for empty map

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

    localizer_type = 'slam_toolbox' # 'amcl', 'map_server', 'cartographer', 'slam_toolbox'

    # Indoors only
    # Include the generic localizers launcher with dragger defaults
    indoor_localizers = include_launch(
        package_name,
        ['launch', 'localizers.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'localizer_type': localizer_type,
            'map': map_file,
        }
    )

    # Outdoors only
    #  - localization with GPS + LIDAR (SLAM Toolbox)
    outdoors_loc_nav = include_launch(
        "outdoors_loc_nav",
        ['launch', 'outdoors_loc.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'localizer': 'map_server',  # 'map_server' or 'slam_toolbox' or 'cartographer' or 'amcl'  Default: 'map_server'
            'map': map_file,
        }
    )

    # Multi-robot safe: wrap everything under the namespace
    outdoor_localizers = [
        ekf_imu_odom,
        outdoors_loc_nav, # external package preferred for outdoors
    ]

    # ---- Choose indoors or outdoors ----
    #robot_localizers = namespace_wrap(namespace, indoor_localizers)
    robot_localizers = namespace_wrap(namespace, outdoor_localizers)

    return LaunchDescription([
        
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value='dragger'),
        DeclareLaunchArgument('localizer_type', default_value=''),
        DeclareLaunchArgument('map', default_value=map_file),

        LogInfo(msg=[
            '============ starting Dragger LOCALIZERS  namespace="', namespace,
            '"  use_sim_time=', use_sim_time, '  robot_model=', robot_model
        ]),

        robot_localizers
    ])

