"""
Dragger-specific localizers launcher.
Includes the generic launch/localizers.launch.py with dragger defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.helpers import include_launch, namespace_wrap


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='dragger')
    map_file = LaunchConfiguration('map', default='') # can be '' for empty map
    localizer_type = LaunchConfiguration('localizer_type', default='slam_toolbox')
    rtabmap_enabled = LaunchConfiguration('rtabmap_enabled', default='false')

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
    # ==========================


    # -----------------------------------------------------------------------------------
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

    # -----------------------------------------------------------------------------------
    # Outdoors only
    #  - localization with GPS + LIDAR (SLAM Toolbox)
    outdoor_localizers = include_launch(
        "outdoors_loc_nav",
        ['launch', 'outdoors_loc.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'localizer': localizer_type,  # Default: 'map_server'
            'map': map_file,
        }
    )

    # Launch visual SLAM/localization with RTAB-Map if requested.
    # Dragger uses separate Raspberry Pi with dual cameras
    #  - see https://github.com/slgrobotics/ros2_inference_stereo
    rtabmap = include_launch(
        package_name,
        ['launch', 'rtabmap.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'database_path': '~/.ros/rtabmap.db',
            'delete_db_on_start': 'true',
            'localization': 'false',
            'odom_topic': '/odometry/local',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/stereo/depth/image_rect_raw',
            'camera_info_topic': '/camera/camera_info',
            'subscribe_scan': 'false',
            'scan_topic': '/scan',
            'imu_topic': '/imu/data',
        },
        condition=IfCondition(rtabmap_enabled)
    )

    # -----------------------------------------------------------------------------------
    # Choose indoors or outdoors:
    #robot_localizers = namespace_wrap(namespace, [indoor_localizers], rtabmap)
    robot_localizers = namespace_wrap(namespace, [outdoor_localizers], rtabmap)

    return LaunchDescription([
        
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value='dragger'),
        DeclareLaunchArgument('localizer_type', default_value=''),
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('rtabmap_enabled', default_value='false'),

        LogInfo(msg=[
            '============ starting Dragger LOCALIZERS  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  localizer_type=', localizer_type,
            '  map=', map_file,
            '  rtabmap_enabled=', rtabmap_enabled
        ]),

        robot_localizers
    ])

