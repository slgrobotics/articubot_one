from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter
from launch_ros.actions import Node

# See /opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py

def generate_launch_description():

    # Get the launch directory
    package_name='articubot_one'

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    # for experiments: a bad alternative to ekf_imu_odom for slam_toolbox - static transform publisher
    start_tf = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments=[
                        '--x', '0.0',     # X translation in meters
                        '--y', '0.0',     # Y translation in meters
                        '--z', '0.0',     # Z translation in meters
                        '--roll', '0.0',  # Roll in radians
                        '--pitch', '0.0', # Pitch in radians
                        '--yaw', '0.0',   # Yaw in radians (e.g., 90 degrees)
                        '--frame-id', 'map', # Parent frame ID
                        '--child-frame-id', 'odom' # Child frame ID
                    ]
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Log the map file being used
    ld.add_action(LogInfo(msg=[
        '============ starting Static TF map -> odom  namespace="', namespace,
        '"  use_sim_time=', use_sim_time
    ]))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_tf)

    return ld
