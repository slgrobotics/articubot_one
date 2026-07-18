"""
Seggy-specific localizers launcher.
Includes the generic launch/localizers.launch.py with seggy defaults.
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
    robot_model = LaunchConfiguration('robot_model', default='seggy')
    map_file = LaunchConfiguration('map', default='') # can be '' for empty map
    localizer_type = LaunchConfiguration('localizer_type', default='slam_toolbox')
    rtabmap_enabled = LaunchConfiguration('rtabmap_enabled', default='false')

    # Include the generic localizers launcher with seggy defaults
    localizers = include_launch(
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

    rtabmap = include_launch(
        package_name,
        ['launch', 'rtabmap.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_topic': '/odometry/filtered',
            'rgb_topic': '/oak/rgb/image_rect',
            'depth_topic': '/oak/stereo/image_raw',
            'camera_info_topic': '/oak/rgb/camera_info',
            'subscribe_scan': 'false',
            'scan_topic': '/scan',
            'imu_topic': '/imu/data',
        },
        condition=IfCondition(rtabmap_enabled)
    )

    robot_localizers = namespace_wrap(namespace, [localizers, rtabmap])

    return LaunchDescription([
        
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value='seggy'),
        DeclareLaunchArgument('localizer_type', default_value=''),
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('rtabmap_enabled', default_value='false'),

        LogInfo(msg=[
            '============ starting Seggy LOCALIZERS  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  localizer_type=', localizer_type,
            '  map=', map_file,
            '  rtabmap_enabled=', rtabmap_enabled
        ]),

        robot_localizers
    ])

