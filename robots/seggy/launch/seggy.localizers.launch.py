from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

#
# Generate launch description for Seggy robot localizers
#
# Localizers are SLAM and/or EKF nodes that provide robot pose estimates
# to be used by navigation stack (Nav2) and other components.
# They can be robot-specific, and that's why we have this separate launch file.
#

def generate_launch_description():

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from <robot_model>.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, create1...
    robot_model = LaunchConfiguration('robot_model', default='')

    # Build substitution-based paths so robot_model can be used at launch-time
    slam_toolbox_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'launch', 'seggy_slam_toolbox.launch.py'
    ])
    cartographer_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'launch', 'cartographer.launch.py'
    ])

    # We do not need to use PathJoinSubstitution for map_server_path and ekf_odom_path, which do not depend on robot_model
    # Still using it here for consistency and fully dynamic resolution
    map_server_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'launch', 'map_server.launch.py'
    ])
    ekf_odom_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'launch', 'ekf_odom.launch.py'
    ])

    # most used localizer
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([slam_toolbox_path]),
                launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace, 'robot_model': robot_model}.items()
    )

    # optional alternative
    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([cartographer_path]),
                launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace, 'robot_model': robot_model}.items()
    )

    # optional alternative
    # Map server is convenient when used with GPS and an empty map, for obstacle avoidance.
    #map_yaml_file = os.path.join(package_path,'assets','maps','empty_map.yaml')   # this is default anyway
    map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([map_server_path]
                ), launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()       # empty_map - default
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items() # warehouse
    )

    # ekf_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    ekf_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ekf_odom_path]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model, 'namespace': namespace}.items()
    )

    # debugging helper - rarely needed
    # for experiments: a bad alternative to ekf_localizer for slam_toolbox - static transform publisher
    tf_localizer = Node(package = "tf2_ros", 
                    namespace=namespace,
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    
    localizers_include = GroupAction(
        actions=[
            ekf_localizer, # needed for slam_toolbox. cartographer doesn't need it when cartographer.launch.py uses direct mapping
            #tf_localizer,
            # use either map_server, OR cartographer OR slam_toolbox, as they are all mappers
            #cartographer, # localization via LIDAR
            slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=10.0, actions=[localizers_include])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for drive nodes'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'robot_model',
            default_value='',
            description='Robot model (e.g., seggy, plucky, dragger)'),

        LogInfo(msg=['============ starting LOCALIZERS  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),

        delayed_loc
    ])
