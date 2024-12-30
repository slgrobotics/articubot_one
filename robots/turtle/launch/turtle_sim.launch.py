import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

#
# To launch Create 1 Turtle sim:
#
# cd ~/robot_ws; colcon build; ros2 launch articubot_one turtle_sim.launch.py
#

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    robot_model='turtle'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox_params_file = os.path.join(package_path,'config','mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','slam_toolbox.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_toolbox_params_file}.items()
    )

    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','cartographer.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #map_yaml_file = os.path.join(package_path,'assets','maps','empty_map.yaml')   # this is default anyway
    map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','map_server.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()       # empty_map - default
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items() # warehouse
    )

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     #'use_composition': 'True',
                                     'odom_topic': 'diff_cont/odom',
                                     'autostart' : 'true',
                                     'params_file' : nav2_params_file }.items()
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
            '-name', 'turtle',
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
        parameters=[{'use_sim_time': use_sim_time}],
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
            # use either cartographer OR slam_toolbox, as both are mappers
            cartographer,  # localization via LIDAR
            #slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=5.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=10.0, actions=[nav2])

    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        rsp,
        joystick,
        twist_mux,
        gz_include,
        delayed_loc,
        #delayed_nav
    ])

