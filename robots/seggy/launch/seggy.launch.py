import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():

    package_name='articubot_one'

    # Make namespace overridable at runtime
    namespace = LaunchConfiguration('namespace', default='')

    robot_model='seggy'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
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
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     'use_composition': 'True',
                                     'container_name': 'nav2_container',
                                     'odom_topic': 'odometry/local',
                                     #'use_respawn': 'true',
                                     'autostart' : 'true',
                                     'params_file' : nav2_params_file }.items() # pass nav2 params file if not using composition
    )

    # Include separate drive launch for better modularity
    drive_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','seggy.drive.launch.py')]),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    # Include separate sensors launch for better modularity
    sensors_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','seggy.sensors.launch.py')]),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    # Include separate localizers launch for better modularity
    localizers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','seggy.localizers.launch.py')]),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    delayed_nav = TimerAction(period=20.0, actions=[nav2])

    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for seggy nodes'),

        rsp,
        drive_include,
        sensors_include,
        localizers_include,
        container_nav2,  # Add the container to the launch description, if 'use_composition': 'True' is set
        delayed_nav
    ])
