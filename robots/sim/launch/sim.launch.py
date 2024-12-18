import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    robot_model='sim'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': 'true', 'robot_model' : robot_model}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
                ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
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
                ), launch_arguments={'use_sim_time': 'true', 'autostart' : 'true'}.items()
    )

    # Start Gazebo Harmonic (GZ, Ignition)
    # -- set gazebo sim resource path for worlds and meshes (STLs):
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_path, 'assets', 'worlds'), ':' + os.path.join(package_path, 'assets')
            ]
        )

    # Specify the world SDF:
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
            # Robot's starting position on the Grid:
            '-x', '0.0', # positive - towards East
            '-y', '0.0', # positive - towards North
            '-z', '0.4', # let it gently settle on the ground plane
            '-Y', '0.333', # yaw (heading) in radians, related to 0=East, e.g. 0.333 = 30 degrees towards North
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
            'config_file': os.path.join(robot_path, 'config', 'gz_ros_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Obsolete: dual_ekf_navsat_params.yaml can subscribe to /diff_cont/odom directly
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
                ), launch_arguments={'use_sim_time': 'true', 'robot_model' : robot_model}.items()
    )

    #map_yaml_file = os.path.join(package_path, 'assets', 'maps','empty_map.yaml')   # this is default anyway
    map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','map_server.launch.py')]
                ), launch_arguments={'use_sim_time': 'true'}.items()
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': 'true'}.items() # warehouse
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
            #odom_relay,
            #gps_fix_translator
        ]
    )

    localizers_include = GroupAction(
        actions=[
            LogInfo(msg='============ starting LOCALIZERS ==============='),
            navsat_localizer,
            # use either map_server OR slam_toolbox, as both are mappers
            map_server,    # localization is left to GPS
            #slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=5.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=8.0, actions=[nav2])

    # start the demo autonomy task (script)
    # See /opt/ros/jazzy/lib/python3.12/site-packages/nav2_simple_commander/example_waypoint_follower.py
    #waypoint_follower = Node(
    #    package='nav2_simple_commander',
    #    executable='example_waypoint_follower',
    #    emulate_tty=True,
    #    output='screen',
    #)

    waypoint_follower = Node(
        package='articubot_one',
        executable='xy_waypoint_follower',
        emulate_tty=True,
        output='screen',
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        #joystick,
        twist_mux,
        gz_include,
        delayed_loc,
        #delayed_nav
        #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
    ])
