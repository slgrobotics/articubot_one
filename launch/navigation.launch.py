from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer

#
# A generic wrapper launch file to start Nav2 navigation stack for a given robot model
# Typically included from the robot's main launch file (e.g., seggy.launch.py)
#
# - takes 'nav2_params.yaml' from the robot's config directory
# - runs "navigation_launch.py" in a container with appropriate arguments
#

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept launch arguments from parent (for example, seggy.launch.py)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    # Build substitution for nav2 params so robot_model can be dynamic
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'nav2_params.yaml'
    ])

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
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'use_composition': 'True',
                          'container_name': 'nav2_container',
                          'odom_topic': 'odometry/local',
                          #'use_respawn': 'true',
                          'autostart': 'true',
                          'params_file': nav2_params_file}.items()  # pass nav2 params file if not using composition
    )

    return LaunchDescription([

        LogInfo(msg=['============ starting NAVIGATION (generic wrapper)  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),
        #LogInfo(msg=['Nav2 params file: ', nav2_params_file]),

        container_nav2,  # Add the container to the launch description, if 'use_composition': 'True' is set
        nav2
    ])
