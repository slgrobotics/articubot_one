"""
SLAM Toolbox Localizer Launch Wrapper
Provides SLAM-based localization using LIDAR and odometry.
Internally launches EKF to provide required transform from odom to base_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    slam_toolbox_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'slam_toolbox_params.yaml'
    ])

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

    # SLAM Toolbox
    slam_toolbox = include_launch(
        "slam_toolbox",
        ['launch', 'online_async_launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'slam_params_file': slam_toolbox_params_file
        }
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),
        ekf_localizer,
        slam_toolbox,
    ])
