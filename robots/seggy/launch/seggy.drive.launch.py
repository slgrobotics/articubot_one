from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Accept launch arguments from parent (seggy.launch.py)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    robot_model = LaunchConfiguration('robot_model', default='seggy')

    package_name = 'articubot_one'
    package_path = get_package_share_directory(package_name)

    # Include the main / common drive launch and pass through all arguments
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'drive.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'robot_model': robot_model
        }.items()
    )

    # add any robot-specific launch operations here if needed

    return LaunchDescription([
        drive_launch
    ])