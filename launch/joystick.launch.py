from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    package_name='articubot_one'

    package_path = get_package_share_directory(package_name)

    # See XBox buttons mapping: https://europe1.discourse-cdn.com/unity/original/3X/5/8/58e7b2a50ec35ea142ae9c4d27c9df2d372cd1f3.jpeg
    # Overriding '/opt/ros/jazzy/share/teleop_twist_joy/config/xbox.config.yaml' - because teleop-launch.py isn't accepting buttons assignments:
    joystick_params_file = os.path.join(package_path,'config','joystick.yaml')

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("teleop_twist_joy"),'launch','teleop-launch.py')]
                ), launch_arguments={
                    'use_sim_time': use_sim_time,
                    'joy_config': 'xbox',
                    'joy_dev': '0',
                    'joy_vel':'cmd_vel_joy',
                    'publish_stamped_twist': 'true',
                    'config_filepath': joystick_params_file,
                    }.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            
        joystick       
    ])