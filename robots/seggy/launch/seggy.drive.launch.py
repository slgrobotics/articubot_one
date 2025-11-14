from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = 'articubot_one'

    # Accept launch arguments from parent (seggy.launch.py)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='')

    package_path = get_package_share_directory(package_name)

    # Include the main / common drive launch and pass through all arguments.
    # Use the non-sim drive when use_sim_time is false, and the sim-specific
    # drive when use_sim_time is true.
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'drive.launch.py')]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }.items(),
        condition=UnlessCondition(use_sim_time)
    )

    drive_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'drive_sim.launch.py')]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # add any robot-specific drive launch operations here if needed

    return LaunchDescription([
        drive_launch,
        drive_sim_launch
    ])
