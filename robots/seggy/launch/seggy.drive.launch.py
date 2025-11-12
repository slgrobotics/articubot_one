import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    # Keep interface compatible with being included from seggy.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Compute package paths (keeps this file standalone when included)
    package_name = 'articubot_one'
    robot_model = 'seggy'
    package_path = get_package_share_directory(package_name)
    robot_path = os.path.join(package_path, 'robots', robot_model)

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
    #             ), launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    # Include twist_mux for command velocity arbitration
    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    controllers_params_file = os.path.join(robot_path,'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        namespace=namespace,
        executable="ros2_control_node",
        parameters=[controllers_params_file],
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
        # joystick,
        drive_include,
    ])