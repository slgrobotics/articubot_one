import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # For any real robot (not a sim), launch on workstation:
    #   ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = os.path.join(package_path, 'config', 'main.rviz')  # 'view_bot.rviz'  'map.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        joystick,
        rviz
    ])
