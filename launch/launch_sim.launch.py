import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch_ros.actions import Node, SetRemap
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
                ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam_toolbox_params_file = os.path.join(package_path,'config','mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py')]
                ), launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_toolbox_params_file}.items()
    )

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                #PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("nav2_bringup"),'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': 'true', 'autostart' : 'false'}.items()
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

    # -- where to find meshes:
    gazebo_models_path = os.path.join(package_path, 'description')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # -- where to find the world SDF:
    gazebo_arguments = LaunchDescription([
            DeclareLaunchArgument('world', default_value='test_robot_world',
                                  description='Gz sim Test World'),
            #DeclareLaunchArgument('world', default_value='baylands',
            #                      description='Gz sim Baylands World'),
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

    # spawn entity (robot model) in the Gazebo gz_sim
    # see arguments:  ros2 run ros_gz_sim create --helpshort
    spawn_sim_robot = Node(package='ros_gz_sim',
        executable='create',
        namespace='/',
        arguments=[
            '-name', 'dragger',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.4',
            '-Y', '0.333',
            '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}],
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
        # remappings don't work here. Use relay.
        #arguments=["diff_cont", "--controller-manager", "/controller_manager", "--ros-args", "--remap",  "/diff_cont/odom:=/odom"],
        #remappings=[('/diff_cont/odom','/odom')]
    )

    # No need to run controller_manager - it runs within Gazebo ROS2 Bridge.
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

    # Gazebo controller_manager is not subject to renaming through parameters, so we use topic relay here:
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        namespace='/',
        parameters=[{ 'input_topic': '/diff_cont/odom', 'output_topic': '/odom'}],
        # see what odom0 says in config/dual_ekf_navsat_params.yaml
        #parameters=[{ 'input_topic': '/odometry/local', 'output_topic': '/odom'}],
    )

    navsat_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','dual_ekf_navsat.launch.py')]
                ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # =========================================================================

    gz_include = GroupAction(
        actions=[

            #SetRemap(src='/diff_cont/odom', dst='/odom'),

            gazebo_resource_path,
            gazebo_arguments,
            gazebo_ui,
            spawn_sim_robot,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            rviz,
            bridge,
            odom_relay,
            #gps_fix_translator
        ]
    )

    nav_include = GroupAction(
        actions=[
            navsat_localizer,
            slam_toolbox,
            nav2
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
        gz_include,
        nav_include
    ])
