from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

#
# Generate launch description for Stingray robot drive system
#
# Real robot: Unlike other robots (based on https://github.com/slgrobotics/diffdrive_arduino) 
#             Stingray cannot use generic "drive.launch.py" and must run RoboClaw driver and twist_mux
#             Same pattern can be used for robots with custom wheels driving hardware
#             See https://github.com/wimblerobotics/roboclaw_driver
#
# Gazebo sim: This launch file includes the generic "drive_sim.launch.py" with appropriate
#             parameters for stingray robot in simulation.
#

def generate_launch_description():

    package_name = 'articubot_one'

    # Accept launch arguments from parent (stingray.launch.py)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    # -----------------------------------------------------------------------------
    # For real robot, include twist_mux for command velocity arbitration
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'twist_mux.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    roboclaw_launch_file = PathJoinSubstitution([
        FindPackageShare(package_name), "robots", robot_model, "config", "roboclaw.yaml"
    ])

    roboclaw_params_file = PathJoinSubstitution([
        FindPackageShare("roboclaw_driver"), "launch", "roboclaw_driver.launch.py"
    ])

    # For real robot, include the RoboClaw driver:
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboclaw_launch_file),
        launch_arguments={
            "params_file": roboclaw_params_file,
        }.items(),
        remappings=[('cmd_vel', 'diff_cont/cmd_vel'),('odom','diff_cont/odom')],
        condition=UnlessCondition(use_sim_time)
    )

    # -----------------------------------------------------------------------------
    # Gazebo sim: Use the sim-specific generic drive launch when "use_sim_time" is true.
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

    # -----------------------------------------------------------------------------
    # add any robot-specific drive launch operations here if needed

    return LaunchDescription([

        # not needed for files not meant to run directly, parent declares and passes arguments,
        # but for consistency and clarity, adding declarations is harmless:
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=''),

        twist_mux,
        drive_launch,
        drive_sim_launch
    ])
