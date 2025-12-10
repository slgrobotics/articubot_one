"""
Seggy-specific localizers launcher.
Includes the generic launch/localizers.launch.py with seggy defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='seggy')
    map_file = LaunchConfiguration('map', default='')

    localizer_type = 'slam_toolbox' # 'amcl', 'map_server', 'cartographer', 'slam_toolbox'

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

    return LaunchDescription([
        
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value='seggy'),
        DeclareLaunchArgument('localizer_type', default_value=''),
        DeclareLaunchArgument('map', default_value=map_file),

        localizers,
    ])

