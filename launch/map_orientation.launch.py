"""Adapt the standalone map_orientation_saver estimator to this robot project.

Generic and robot-specific localizers include this adapter. It keeps the
estimator implementation, default parameters, and saving CLI in the external
map_orientation_saver package; no estimator logic lives in articubot_one.

Dragger's real-hardware NavSat corrections are read from its robot config and
converted to the estimator's degrees. Gazebo uses true-ENU heading with zero
correction. An explicit orientation_config_filepath is applied afterward so
robot/operator overrides retain their previous precedence.

Examples:
    ros2 launch articubot_one map_orientation.launch.py robot_model:=dragger
    ros2 launch articubot_one seggy.launch.py use_sim_time:=true
    ros2 launch articubot_one dragger.launch.py use_map_orientation_estimator:=false

Map saving is now invoked with:
    ros2 run map_orientation_saver map_saver_with_orientation -f <map_name>
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from articubot_one.launch_utils.helpers import namespace_wrap


def _start(context):
    if not IfCondition(LaunchConfiguration('use_map_orientation_estimator')).evaluate(context):
        return []
    share = get_package_share_directory('articubot_one')
    robot = LaunchConfiguration('robot_model').perform(context)
    config = os.path.expanduser(LaunchConfiguration('orientation_config_filepath').perform(context))
    orientation_share = get_package_share_directory('map_orientation_saver')
    parameters = [os.path.join(orientation_share,'config','map_orientation.yaml')]
    # Reuse the robot's NavSat heading corrections rather than maintaining copies.
    navsat = os.path.join(share,'robots',robot,'config','navsat_transform.yaml')
    simulated = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
    if simulated:
        # Gazebo's imu.xacro publishes absolute ENU orientation, not magnetic yaw.
        parameters.append({'heading_reference': 'true_enu',
                           'magnetic_declination_degrees': 0.0, 'yaw_offset': 0.0})
    elif robot and os.path.isfile(navsat):
        with open(navsat) as stream:
            settings = (yaml.safe_load(stream) or {}).get('navsat_transform',{}).get('ros__parameters',{})
        corrections = {}
        # NavSat's public API uses radians; our estimator exposes degrees.
        if 'magnetic_declination_radians' in settings:
            corrections['magnetic_declination_degrees'] = math.degrees(
                float(settings['magnetic_declination_radians']))
        if 'yaw_offset' in settings:
            corrections['yaw_offset'] = float(settings['yaw_offset'])
        parameters.append(corrections)
    if config:
        parameters.append(config)
    parameters.append({'use_sim_time': simulated})
    node = Node(package='map_orientation_saver',executable='map_orientation_node',
                name='map_orientation_node',parameters=parameters,output='screen',
                remappings=[('/tf','tf'),('/tf_static','tf_static')])
    return [namespace_wrap(LaunchConfiguration('namespace'),[node])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value=''),
        DeclareLaunchArgument('robot_model',default_value=''),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('use_map_orientation_estimator',default_value='true'),
        DeclareLaunchArgument('orientation_config_filepath',default_value='',
                              description='Optional estimator YAML overriding defaults and NavSat corrections'),
        OpaqueFunction(function=_start),
    ])
