"""Robot adapter for the Wi-Fi V2 run coordinator.

All source choices are launch arguments forwarded by robot launches. The Wi-Fi
package owns run cleanup, live-map acquisition, simulation and node startup.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from wifi_logger_visualizer.suite_launch import ARGUMENTS, launch_suite


def _launch_nodes(context):
    return launch_suite(context)


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wifi_logger_visualizer'),
                          'config', 'wifi_logger_config.yaml')
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=config if key == 'config_filepath' else value)
          for key, value in ARGUMENTS.items()],
        OpaqueFunction(function=_launch_nodes),
    ])
