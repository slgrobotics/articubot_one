import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from nav2_common.launch import ReplaceString
from launch_ros.actions import Node

#
# To launch Create 1 Turtle sim:
#
# cd ~/robot_ws; colcon build; ros2 launch articubot_one turtle_sim.launch.py
#

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    namespace=''

    package_name='articubot_one' #<--- CHANGE ME

    robot_model='turtle'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','turtle_slam_toolbox.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
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

    # odom_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    odom_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','ekf_odom.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    # alternative to odom_localizer for slam_toolbox
    tf_localizer = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','turtle_nav.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time }.items()
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
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-name', robot_model,
            '-topic', '/robot_description',
            # Robot's starting position on the Grid:
            '-x', '0.0', # positive - towards East
            '-y', '0.0', # positive - towards North
            '-z', '0.4', # let it gently settle on the ground plane
            '-Y', '0.333', # yaw (heading) in radians, related to 0=East, e.g. 0.333 = 30 degrees towards North
            '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # No need to run controller_manager - it runs within Gazebo ROS2 Bridge.
    # We only need to point spawners to the correct controller_manager instance in the arguments.
    # Only configure controllers, after the robot shows up live in GZ:

    joint_broad_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        # remappings don't work here. Use relay.
        #arguments=["diff_cont", "--controller-manager", "/controller_manager", "--ros-args", "--remap",  "/diff_cont/odom:=/odom"],
        #remappings=[('diff_cont/odom','odom')]
    )

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

    rviz_and_joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','launch_rviz.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Bridge ROS topics and Gazebo messages for establishing communication

    gz_model_name = robot_model  # see spawn_sim_robot 'name' above, it becomes "gz model" for all gz queries

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=os.path.join(package_path, 'config', 'gz_ros_bridge.yaml'),
        replacements={"<model_name>": gz_model_name, "<namespace>": namespace, "///": "/", "//": "/"},
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        namespace=namespace,
        executable='parameter_bridge',
        parameters=[{
            'config_file': namespaced_gz_bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # =========================================================================

    gz_include = GroupAction(
        actions=[

            #SetRemap(src='diff_cont/odom', dst='odom'),

            gazebo_resource_path,
            gazebo_arguments,
            gazebo_ui,
            spawn_sim_robot,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            gz_bridge,
            #odom_relay,
            #gps_fix_translator
        ]
    )

    localizers_include = GroupAction(
        actions=[
            LogInfo(msg='============ starting LOCALIZERS ==============='),
            odom_localizer, # needed for slam_toolbox. cartographer doesn't need it when cartographer.launch.py uses direct mapping
            #tf_localizer,
            # use either cartographer OR slam_toolbox, as both are mappers
            #cartographer,  # localization via LIDAR
            slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=5.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=10.0, actions=[nav2])

    # start the demo autonomy task (script)
    # See /opt/ros/jazzy/lib/python3.12/site-packages/nav2_simple_commander/example_waypoint_follower.py
    #waypoint_follower = Node(
    #    package='nav2_simple_commander',
    #    namespace=namespace,
    #    executable='example_waypoint_follower',
    #    emulate_tty=True,
    #    output='screen',
    #)


    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        rsp,
        rviz_and_joystick,
        twist_mux,
        gz_include,
        delayed_loc,
        delayed_nav
        #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
    ])

