from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from articubot_one.launch_utils.helpers import include_launch


"""
AMCL (Adaptive Monte Carlo Localization) Localizer
Provides probabilistic localization using particle filters.
Requires a pre-built map and initial pose estimate (/initialpose)
Provides pose estimates through Monte Carlo sampling.
Often used with Nav2 for navigation with known maps.
"""

def generate_launch_description():
    
    package_name = 'articubot_one'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')
    # Optional map file to pass to map_server (empty -> map_server default)
    map_file = LaunchConfiguration('map', default='')

    # Note: EKF filter must be launched prior (in the Drive or Sensors launch file) 
    # It is needed to provide odom->base_link transform

    # Include map server for AMCL (AMCL expects a map)
    map_server = include_launch(
        package_name,
        ['launch', 'map_server.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'map': map_file,
            'params_file': PathJoinSubstitution([FindPackageShare(package_name), 'config', 'map_server_params.yaml']),
        }
    )

    # Path to AMCL params file inside the robot config directory
    amcl_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config', 'amcl_params.yaml'
    ])

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=namespace,
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odom', 'odometry/local'), ('scan', 'scan')],
        name='amcl',  # Explicit name for lifecycle manager
    )

    # Lifecycle manager to activate AMCL
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=namespace,
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl']
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('map', default_value='', description='Path to map YAML file for map_server (optional)'),

        LogInfo(msg=[
            '============ starting AMCL + MAP SERVER LOCALIZER  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
        ]),

        LogInfo(msg=[
            '  amcl_params_file=', amcl_params_file,
            '  map=', map_file
        ]),

        map_server,
        amcl_node,
        lifecycle_manager,
    ])
