from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

#
# Generate launch description for Turtle robot drive system
#
# This launch file includes the generic "drive_sim.launch.py" with appropriate
# parameters for turtle robot in simulation.
#

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept launch arguments from parent (turtle.launch.py)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    # For real robot, include the Roomba Create 1 specific drive launch and pass through all arguments.
    # Use the non-sim drive when "use_sim_time" is false
    drive_launch = Node(
        package='create_driver',
        namespace=namespace,
        executable='create_driver',
        name='create_driver',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            'robot_model': 'CREATE_1', # not the same as robot_model argument
            'dev': '/dev/ttyUSB0',
            'baud': 57600,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'latch_cmd_duration': 2.0,
            'loop_hz': 66.0,    # control loop frequency, odom publishing etc. Create 1 sends all sensor data every 15ms (66Hz)
            'publish_tf': False,
            'gyro_offset': 0.0,
            'gyro_scale': 1.21,
            'distance_scale': 1.05
        }],
        remappings=[('cmd_vel', 'diff_cont/cmd_vel'),('odom','diff_cont/odom')],
        condition=UnlessCondition(use_sim_time)
    )

    # Use the sim-specific generic drive launch when "use_sim_time" is true.
    # It will pick URDF and controller config based on robot_model argument.
    drive_sim_launch_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'launch', 'drive_sim.launch.py'
    ])  

    drive_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drive_sim_launch_path),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # add any robot-specific drive launch operations here if needed

    return LaunchDescription([

        # not needed for files not meant to run directly, parent declares and passes arguments,
        # but for consistency and clarity, adding declarations is harmless:
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),

        drive_launch,
        drive_sim_launch
    ])
