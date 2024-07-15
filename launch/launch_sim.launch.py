import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(package_path,'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace='/',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        namespace='/',
        # use_sim_time must be False here, or time stamp will be 0:
        parameters=[{'use_sim_time': False}, {'frame_id': 'base_link'}],
        remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
    )

    # Start Gazebo Harmonic (GZ, Ignition)
    # -- set gazebo sim resource path for meshes and STLs:
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_path, 'worlds'), ':' +
            os.path.join(package_path, 'description')
            ]
        )

    # -- where to find the world SDF:
    gazebo_arguments = LaunchDescription([
            DeclareLaunchArgument('world', default_value='test_robot_world',
                                  description='Gz sim Test World'),
        ]
    )

    # -- how to launch Gazebo UI:
    gazebo_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                '.sdf',
                ' -v 4',
                ' -r']
            )
        ]
    )

    # spawn entity (robot model) in the Gazebo gz_sim:
    spawn_sim_robot = Node(package='ros_gz_sim',
        executable='create',
        namespace='/',
        arguments=[
            '-name', 'dragger',
            '-topic', '/robot_description',
            '-allow_renaming', 'true'],
        output='screen')

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='/',
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='/',
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
        namespace='/',
        #arguments=['-d', os.path.join(package_path, 'config', 'view_bot.rviz')],
        #arguments=['-d', os.path.join(package_path, 'config', 'map.rviz')],
        arguments=['-d', os.path.join(package_path, 'config', 'main.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='/',
        parameters=[{
            'config_file': os.path.join(package_path, 'config', 'gz_ros_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_resource_path,
        gazebo_arguments,
        gazebo_ui,
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
        spawn_sim_robot,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rviz,
        bridge
    ])
