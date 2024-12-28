import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#
# it is sometimes necessary to launch slam_toolbox manually in a separate shell
#

def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_toolbox_params_file = os.path.join(package_path,'robots','turtle','config','mapper_params.yaml')

    slam_toolbox = IncludeLaunchDescription(
                # see /opt/ros/jazzy/share/slam_toolbox/launch
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_toolbox_params_file}.items()
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        LogInfo(msg='starting SLAM_TOOLBOX: ' + slam_toolbox_params_file),
        LogInfo(msg=use_sim_time),

        slam_toolbox
    ])
