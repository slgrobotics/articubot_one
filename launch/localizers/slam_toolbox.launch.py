"""
SLAM Toolbox Localizer Launch Wrapper
Provides SLAM-based localization using LIDAR and odometry.
Internally launches EKF to provide required transform from odom to base_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')
    map_file = LaunchConfiguration('map', default='')

    slam_toolbox_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'slam_toolbox_params.yaml'
    ])

    # SLAM Toolbox only publishes the map to odom transform. 
    # It needs EKF filter to publish odom to base_link transform.
    ekf_localizer = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    #
    # Function to prepare SLAM Toolbox params with optional "map_file" rewritten
    #
    def _prepare_slam_params(context, *args, **kwargs):
        """Resolve base params file and optionally add map_file_name override."""

        base_params = context.perform_substitution(slam_toolbox_params_file)
        map_val = context.perform_substitution(map_file)

        # Load the base SLAM Toolbox params file
        try:
            import yaml
            import tempfile
            
            with open(base_params, 'r') as f:
                params_content = yaml.safe_load(f) or {}
            
            # Merge map_file_name if provided (non-empty)
            if map_val:
                params_content.setdefault('slam_toolbox', {}).setdefault('ros__parameters', {})
                params_content['slam_toolbox']['ros__parameters']['map_file_name'] = map_val
            
            # Write merged params to a temporary file (see /tmp/slam_toolbox_params_*.yaml)
            with tempfile.NamedTemporaryFile(
                mode='w', delete=False, prefix='slam_toolbox_params_', suffix='.yaml'
            ) as tmp:
                yaml.safe_dump(params_content, tmp)
                merged_params_file = tmp.name
        
        except Exception as e:
            from launch.actions import LogInfo
            return [LogInfo(msg=[
                'ERROR preparing SLAM Toolbox params: ', str(e)
            ])]

        # Include SLAM Toolbox with the (possibly merged) params file
        return [
            include_launch(
                "slam_toolbox",
                ['launch', 'online_async_launch.py'],
                {
                    'use_sim_time': use_sim_time,
                    'namespace': namespace,
                    'slam_params_file': merged_params_file
                }
            )
        ]

    slam_toolbox = OpaqueFunction(function=_prepare_slam_params)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('map', default_value='', description='Path to serialized map file for SLAM Toolbox (optional)'),

        LogInfo(msg=[
            '============ starting EKF + SLAM Toolbox LOCALIZER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  map=', map_file
        ]),

        #ekf_localizer,
        slam_toolbox,
    ])


