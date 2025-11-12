import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Keep interface compatible with being included from seggy.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')


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
        drive_include,
    ])
