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
                )
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','slam_toolbox.launch.py')]
                )
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

    localizers_include = GroupAction(
        actions=[
            LogInfo(msg='============ starting LOCALIZERS ==============='),
            # use either cartographer OR slam_toolbox, as both are mappers
            cartographer,
            #slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=5.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=10.0, actions=[nav2])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/',
        #arguments=['-d', os.path.join(package_path, 'config', 'view_bot.rviz')],
        #arguments=['-d', os.path.join(package_path, 'config', 'map.rviz')],
        arguments=['-d', os.path.join(package_path, 'config', 'main.rviz')],
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
        twist_mux,
        joystick,
        delayed_loc,
        #delayed_nav,
        rviz
    ])

