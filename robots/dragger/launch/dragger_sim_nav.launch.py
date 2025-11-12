import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer

#
# To launch nav2 in sim mode on Raspberry Pi because it crashes on my Intel desktop
#

def generate_launch_description():

    namespace=''

    package_name='articubot_one'

    package_path = get_package_share_directory(package_name)

    robot_model='dragger'

    robot_path = os.path.join(package_path, 'robots', robot_model)

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # Define the ComposableNodeContainer for Nav2 composition:
    container = ComposableNodeContainer(
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
                ), launch_arguments={'use_sim_time': 'true',
                                     'use_composition': 'True',
                                     'container_name': 'nav2_container',
                                     'odom_topic': 'odometry/local',
                                     #'use_respawn': 'true',
                                     'autostart' : 'false',
                                     'params_file' : nav2_params_file }.items() # pass nav2 params file if not using composition
    )

    return LaunchDescription([
        container,  # Add the container to the launch description, if 'use_composition': 'True' is set
        nav2
    ])
