from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#
# Generate launch description for Seggy robot drive system
#
# This launch file includes the generic "drive.launch.py" with appropriate
# parameters for the Seggy robot.
#

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept launch arguments from parent (seggy.launch.py)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    # Seggy uses common differential drive system, and we have a "generic" drive launch for that.
    # Include the "generic" drive launch and pass through all arguments.
    # Use the non-sim drive when "use_sim_time" is false
    drive_launch_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'launch', 'drive.launch.py'
    ])

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drive_launch_path),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }.items(),
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
            'robot_model': robot_model,
            # 'robot_world': 'warehouse', # see assets/worlds/*.sdf Default: 'test_robot_world'
            # 'initial_x': '1.0',
            # 'initial_y': '1.0',
            # 'initial_z': '20.0',
            # 'initial_yaw': '1.57'
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
