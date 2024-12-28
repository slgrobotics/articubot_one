import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#
# To launch nav2 in sim mode on Raspberry Pi because it crashes on my Intel desktop
#

def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    robot_model='turtle'

    robot_path = os.path.join(package_path, 'robots', robot_model)

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': 'true',
                                     'autostart' : 'false',
                                     'params_file' : nav2_params_file }.items()
    )

    return LaunchDescription([
        nav2
    ])
