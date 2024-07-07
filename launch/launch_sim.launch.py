import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # Start Gazebo Fortress (GZ, Ignition)
    try: 
        gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    except:
        gazebo_launch_file = ""

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', ['-r -v 1 empty.sdf'])],
    )

    # spawn entity (robot model) in the Gazebo Fortress (Ignition, GZ):
    spawn_sim_robot = Node(package='ros_gz_sim', executable='create',
            arguments=[
                '-name', 'dragger',
                '-topic', '/robot_description',
                '-allow_renaming', 'true'],
            output='screen')

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    # No ned to run controller_manager - it runs within Gazebo ROS2 Bridge.
    # Only configure controllers, after the robot shows up live in GZ:

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_sim_robot,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
    )

    # Launch them all!
    return LaunchDescription([
        gazebo,
        rsp,
#        joystick,
#        twist_mux,
        spawn_sim_robot,
#        delayed_diff_drive_spawner,
#        delayed_joint_broad_spawner,
#        rviz,
        bridge
    ])
