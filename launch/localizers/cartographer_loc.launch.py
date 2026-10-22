"""
Cartographer Localizer Launch Wrapper
Provides SLAM-based localization using Cartographer with orientation initialization.

Launch:
    as part of the dragger launch - set localizer_type:=cartographer in dragger.launch.py
    (other robots should work too)

See also "outdoors_loc_nav" package with its own cartographer.launch.py and config files:
    https://github.com/slgrobotics/outdoors_loc_nav/blob/lyrical/outdoors_loc_nav/launch/cartographer.launch.py

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.helpers import include_launch


def generate_launch_description():
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')
    map_file = LaunchConfiguration('map', default='')

    # Note: The EKF filter must be launched beforehand (in the Drive or Sensors launch file)
    # It is needed to provide the odom->base_link transform

    cartographer_orientator = include_launch(
        'outdoors_loc_nav',
        ['launch', 'orientation_initializer.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }
    )

    cartographer = include_launch(
        package_name,
        ['launch', 'cartographer.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'robot_model': robot_model
        }
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('map', default_value='', description='Path to map YAML file for map_server (optional)'),

        LogInfo(msg=[
            '============ starting EKF + CARTOGRAPHER LOCALIZER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
            '  map=', map_file
        ]),

        #cartographer_orientator,
        cartographer,
    ])
