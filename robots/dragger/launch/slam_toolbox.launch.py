import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#
# Dragger LD14 LIDAR starts with a delay, so it is sometimes necessary to launch slam_toolbox manually in a separate shell
#

def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    slam_toolbox_params_file = os.path.join(package_path,'config','mapper_params_dragger.yaml')

    slam_toolbox = IncludeLaunchDescription(
                # see /opt/ros/jazzy/share/slam_toolbox/launch
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py')]
                ), launch_arguments={'use_sim_time': 'false', 'slam_params_file': slam_toolbox_params_file}.items()
    )

    return LaunchDescription([
        slam_toolbox
    ])
