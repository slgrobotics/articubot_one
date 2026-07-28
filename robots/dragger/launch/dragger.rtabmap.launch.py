"""
Dragger-specific RTAB-Map launcher.
Use for debugging.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.helpers import include_launch, namespace_wrap


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='dragger')

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
            'rgb_topic': '/camera_stereo/rgb/image_raw',
            'depth_topic': '/camera_stereo/depth/image_rect_raw',
            'camera_info_topic': '/camera_stereo/camera_info',
            'subscribe_scan': 'false',
            'scan_topic': '/scan',
            'imu_topic': '/imu/data',
        },
    )

    wrapped_rtabmap = namespace_wrap(namespace, [rtabmap])

    return LaunchDescription([
        
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value='dragger'),

        LogInfo(msg=[
            '============ starting Dragger RTAB-Map  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model
        ]),

        wrapped_rtabmap
    ])

