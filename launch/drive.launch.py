import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#
# Generate launch description for a typical differential drive robot drive system
#
# This includes controller manager and necessary controllers:
# - twist mux for cmd_vel arbitration
# - controller manager,
# - diff drive,
# - joint state broadcaster,
# - battery state broadcaster
#
# Use example (see seggy.drive.launch.py):
#     drive_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(package_path, 'launch', 'drive.launch.py')),
#         launch_arguments={
#             'namespace': namespace,
#             'use_sim_time': use_sim_time,
#             'robot_model': robot_model
#         }.items()
#     )

def generate_launch_description():

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    # Build substitution-based paths so robot_model can be used at launch-time
    controllers_params_file_sub = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'controllers.yaml'
    ])

    # Include twist_mux for command velocity arbitration
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    controller_manager = Node(
        package="controller_manager",
        namespace=namespace,
        executable="ros2_control_node",
        parameters=[controllers_params_file_sub],
        output="screen"
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )

    battery_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["battery_state_broadcaster", "--controller-ros-args", "--remap battery_state_broadcaster/battery_state:=battery/battery_state"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["diff_cont", "--controller-ros-args", "--remap /tf:=diff_cont/tf"], # isolate TFs, if published.
        output="screen"
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    delayed_battery_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[battery_state_broadcaster_spawner]
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner]
        )
    )

    drive_include = GroupAction(
        actions=[
            twist_mux,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            delayed_battery_state_broadcaster_spawner
        ]
    )

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

        LogInfo(msg=['============ starting ROBOT DRIVE  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),

        drive_include
    ])
