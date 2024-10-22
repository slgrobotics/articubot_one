from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    # See /opt/ros/jazzy/share/twist_mux/launch/twist_mux_launch.py

    twist_mux_params = os.path.join(package_path,'config','twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace='/',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time, 'use_stamped': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
    )

    twist_mux_ = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("twist_mux"),'launch','twist_mux_launch.py')]
                ), launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_stamped': 'true',
                    'cmd_vel_out': '/diff_cont/cmd_vel',
                    'config_topics': twist_mux_params,
                    }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        twist_mux       
    ])