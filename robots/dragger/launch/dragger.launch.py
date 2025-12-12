from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import (
    include_launch,
    delayed_include,
    namespace_wrap,
)

#
# Generate launch description for Dragger robot — multi-robot safe version
#
# If launched with "use_sim_time=true", will be running in Gazebo simulation.
#
# Multi-robot usage example. On the robots' Raspberry Pi's:
#
#   ros2 launch articubot_one dragger.launch.py namespace:=robot1
#   ros2 launch articubot_one dragger.launch.py namespace:=robot2
#
# Every robot runs a fully isolated stack under its namespace.
#

def generate_launch_description():

    package_name = 'articubot_one'
    robot_model = 'dragger'  # static per robot type

    # Launch arguments (can be overridden)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Note: for Gazebo simulation choose "robot_world" in dragger.drive.launch.py

    # -------------------------------------------------------
    # Localizers include - use dragger specific localizers launch
    # -------------------------------------------------------

    localizer_type = 'map_server' # 'amcl', 'map_server', 'cartographer', 'slam_toolbox'

    # You generally want either:
    #   - map_server (GPS) with empty map, to allow Nav2 to use Local Costmap for obstacle avoidance
    #   - slam_toolbox (LIDAR) if you have a previously saved map or have many static features in environment
    #
    # Dragger is outdoors only and has Navsat to provide map->odom TF
    #
    # Note: SLAM Toolbox seems to be very ineffective outdoors in my experiments (not many static features).
    #       The Navsat transform keeps drifting even with good non-RTK GPS signal.
    #       The LIDAR range is limited in sunshine, and there are few features to lock on to.
    #       Therefore, for outdoor use I am using only GPS-based localization with map server.
    #       Adding a previously saved map to the map server is possible.
    #
    # See https://github.com/slgrobotics/outdoors_loc_nav
    #     https://github.com/slgrobotics/articubot_one/wiki/Conversations-with-Overlords#question-6
    #

    # Choose one:
    # Map file for localizers that support it (map_server, amcl):
    map_file = '' # empty 600x600 cells 0.25 m per cell map by default (or no starting map for SLAM Toolbox)
    #map_file = PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'maps', 'empty_map.yaml'])
    #map_file = PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'maps', 'warehouse.yaml']) # result of run in Warehouse world
    #map_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml' # original Nav2 warehouse map
    #
    # For SlAM Toolbox, we can use previously saved serialized map:
    #map_file = 'dragger_map_serial' # previously saved serialized map, relative to launch directory (normally ~/robot_ws)
    #map_file = '/home/sergei/robot_ws/dragger_map_serial' # previously saved serialized map, full path OK too

    localizers_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.localizers.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'localizer_type': localizer_type,
            'map': map_file
        }
    )

    # -------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------
    robot_state_publisher = include_launch(
        package_name,
        ['launch', 'rsp.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
    )

    # -------------------------------------------------------
    # Drive & Sensors
    # -------------------------------------------------------
    drive_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.drive.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
    )

    sensors_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.sensors.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
        condition=UnlessCondition(use_sim_time),  # real robot only
    )

    # -------------------------------------------------------
    # Sonars (real robot + simulation variants)
    # Dragger has four sonars in the corners, we have two separate launch files for them:
    # -------------------------------------------------------

    # Sonar broadcasters for *** real robot ***
    sonars_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.sonars.launch.py'],
        condition=UnlessCondition(use_sim_time),  # real robot only
    )

    # Sonar topic relays (scan->range) for *** Gazebo simulation ***
    sonars_sim_include = include_launch(
        package_name,
        ['launch', 'sonars_sim.launch.py'],
        condition=IfCondition(use_sim_time),  # simulation only
    )

    # -------------------------------------------------------
    # Navigation include - use generic navigation.launch.py
    # -------------------------------------------------------

    navigation_include = include_launch(
        package_name,
        ['launch', 'navigation.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }
    )

    # -------------------------------------------------------
    # Timed includes — Localizers first, Nav2 after
    #
    # Note: TimerAction does not work inside included launch files:
    #       https://chatgpt.com/s/t_691df1a57c6c819194bea42f267a8570
    # -------------------------------------------------------

    # Localizers are run with a delay to allow IMU and odometry to stabilize
    # Navigation stack is run with a further delay to allow map to stabilize
    loc_delay = 18.0    # seconds
    nav_delay = 25.0

    delayed_loc = delayed_include(loc_delay, "LOCALIZERS", localizers_include)
    delayed_nav = delayed_include(nav_delay, "NAVIGATION", navigation_include)

    # -------------------------------------------------------
    # GROUP EVERYTHING UNDER A NAMESPACE FOR MULTI-ROBOT
    #
    # Ensures:
    #   /robot1/tf
    #   /robot1/odom
    #   /robot1/map
    #   /robot1/scan
    #   /robot1/nav2/...
    # -------------------------------------------------------

    namespaced_actions = namespace_wrap(namespace, [
        robot_state_publisher,
        drive_include,
        sensors_include,
        sonars_include,
        sonars_sim_include,
        delayed_loc,
        delayed_nav,
    ])

    # -------------------------------------------------------
    # Final LaunchDescription
    # -------------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for multi-robot deployment'
        ),

        LogInfo(msg=[
            '============ starting DRAGGER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model
        ]),

        # EVERYTHING runs inside namespace_wrap()
        namespaced_actions,
    ])
