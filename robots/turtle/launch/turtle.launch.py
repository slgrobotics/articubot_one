import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

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
    
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','turtle_nav.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time }.items()
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

    rviz_config = os.path.join(package_path, 'config', 'main.rviz')  # 'view_bot.rviz'  'map.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/',
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
        #delayed_nav,
        #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
        rviz
    ])

