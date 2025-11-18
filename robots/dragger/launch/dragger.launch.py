from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#
# Generate launch description for Dragger robot
#
# if launched with "use_sim_time=true", will be running in Gazebo simulation
#
# On the robot's Raspberry Pi:
#
#     ros2 launch articubot_one dragger.launch.py
#
# On the Desktop in simulation:
#
#     ros2 launch articubot_one dragger.launch.py use_sim_time:=true
#

def generate_launch_description():

    package_name='articubot_one'

    # Make namespace overridable at runtime
    namespace = LaunchConfiguration('namespace', default='')

    robot_model='dragger'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    # Include separate launch files for better modularity

    drive_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'dragger.drive.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    sensors_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'dragger.sensors.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items(),
        condition=UnlessCondition(use_sim_time) # only for real robot, not Gazebo simulation
    )

    # Dragger has four sonars in the corners, we have two separate launch files for them:

    # Sonar broadcasters for real robot
    sonars_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'dragger.sonars.launch.py'])
        ),
        condition=UnlessCondition(use_sim_time) # only for real robot, not Gazebo simulation
    )

    # Sonar broadcasters for Gazebo simulation
    sonars_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'sonars_sim.launch.py'])
        ),
        condition=IfCondition(use_sim_time) # only for Gazebo simulation
    )

    localizers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'dragger.localizers.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    # Include generic navigation stack launch, it will pick up Dragger's "nav2_params.yaml"
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model, 'delay': '20.0'}.items()
    )

    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for dragger nodes'),

        robot_state_publisher,
        drive_include,
        sensors_include,
        sonars_include,
        sonars_sim_include,
        localizers_include,
        navigation_include
    ])
