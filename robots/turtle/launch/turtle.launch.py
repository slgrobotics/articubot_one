import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    namespace=''

    package_name='articubot_one' #<--- CHANGE ME

    robot_model='turtle'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
    
    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # Define the ComposableNodeContainer for Nav2 composition:
    container_nav2 = ComposableNodeContainer(
        package='rclcpp_components',
        namespace=namespace,
        executable='component_container_mt', # _mt for multi-threaded
        name='nav2_container',
        composable_node_descriptions=[], # leave empty, as we are using nav2_launch.py to load components
        parameters=[nav2_params_file],   # must be passed here - see https://github.com/ros-navigation/navigation2/issues/4011
        output='screen'
    )

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','turtle_nav.launch.py')]
#                ), launch_arguments={'use_sim_time': use_sim_time }.items()
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     'use_composition': 'True',
                                     'container_name': 'nav2_container',
                                     'odom_topic': 'odometry/local',
                                     #'use_respawn': 'true',
                                     'autostart' : 'true',
                                     'params_file' : nav2_params_file }.items() # pass nav2 params file if not using composition
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

    delayed_loc = TimerAction(period=10.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=20.0, actions=[nav2])

    rviz_config = os.path.join(package_path, 'config', 'main.rviz')  # 'view_bot.rviz'  'map.rviz'

    rviz = Node(
        package='rviz2',
        namespace=namespace,
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        rsp,
        joystick,
        twist_mux,
        delayed_loc,
        #container_nav2,  # Add the container to the launch description, if 'use_composition': 'True' is set
        #delayed_nav,
        #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
        rviz
    ])

