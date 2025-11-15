from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, GroupAction, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

#
# Generate launch description for a typical differential drive robot drive system
#
# This includes necessary controllers for Gazebo-resident Controller Manager:
# - twist mux for cmd_vel arbitration
# - diff drive,
# - joint state broadcaster,
#
# Also, contains Gazebo and RViz launching, simulated robot spawning, and ROS-Gazebo bridging.
#
# Use example (see seggy.drive.launch.py):
#     drive_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(package_path, 'launch', 'drive_sim.launch.py')),
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

    # Include twist_mux for command velocity arbitration
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #
    # Start Gazebo (Harmonic, Ionic) (GZ, Ignition)
    # -- set gazebo sim resource path for worlds and meshes (STLs):
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare(package_name), 'assets', 'worlds']),
            TextSubstitution(text=':'),
            PathJoinSubstitution([FindPackageShare(package_name), 'assets'])
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
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    rviz_and_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'launch_rviz.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Bridge ROS topics and Gazebo messages for establishing communication

    gz_model_name = robot_model  # see spawn_sim_robot 'name' above, it becomes "gz model" for all gz queries

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'gz_ros_bridge.yaml']),
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

    #
    # Gazebo-related variations of spawners
    # 
    # No need to run controller_manager - it runs within Gazebo ROS2 Bridge.
    # We only need to point spawners to the correct controller_manager instance in the arguments.
    # Only configure controllers, after the robot shows up live in GZ:

    joint_broad_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager", "--controller-ros-args", "--remap /tf:=diff_cont/tf" # isolate TFs, if published.
                   # remappings don't work in simulation. Use relay. They aren't needed anyway, all is configured to subscribe to /diff_cont/odom topic.
                   #"--controller-ros-args", "--remap odom:=/odom", # remap odom to root namespace, if needed
                   #"--controller-ros-args", "--remap /tf:=diff_cont/tf" # isolate TFs, if published (it is not, "enable_odom_tf:false" in controllers.yaml).
                   ],
        output="screen"
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

    # =========================================================================

    # have Gazebo and drive system launched as separate groups:

    gz_include = GroupAction(
        actions=[

            #SetRemap(src='diff_cont/odom', dst='odom'),

            gazebo_resource_path,
            gazebo_arguments,
            gazebo_ui,
            spawn_sim_robot,
            gz_bridge,
            #odom_relay,
        ]
    )

    drive_include = GroupAction(
        actions=[
            rviz_and_joystick,
            twist_mux,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
        ]
    )

    # Launch them all!

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for drive nodes'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'robot_model',
            default_value='',
            description='Robot model (e.g., seggy, plucky, dragger)'),

        LogInfo(msg=['============ starting ROBOT DRIVE (SIM)  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),

        gz_include,
        drive_include
    ])
