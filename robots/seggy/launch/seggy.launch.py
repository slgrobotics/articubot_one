from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#
# Generate launch description for Seggy robot
#
# if launched with "use_sim_time=true", will be running in Gazebo simulation
#
# On the robot's Raspberry Pi:
#
#     ros2 launch articubot_one seggy.launch.py
#
# On the Desktop in simulation:
#
#     ros2 launch articubot_one seggy.launch.py use_sim_time:=true
#

def generate_launch_description():

    package_name='articubot_one'

    # Make namespace overridable at runtime
    namespace = LaunchConfiguration('namespace', default='')

    robot_model='seggy'

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
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'seggy.drive.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    sensors_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'seggy.sensors.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items(),
        condition=UnlessCondition(use_sim_time) # only for real robot, not Gazebo simulation
    )

    localizers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'seggy.localizers.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
    )

    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'seggy.navigation.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'robot_model': robot_model}.items()
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
            description='Namespace for seggy nodes'),

        robot_state_publisher,
        drive_include,
        sensors_include,
        localizers_include,
        navigation_include
    ])
