import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # For Dragger robot, launch on workstation:
    #   ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'),

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/',
        #arguments=['-d', os.path.join(package_path, 'config', 'view_bot.rviz')],
        #arguments=['-d', os.path.join(package_path, 'config', 'map.rviz')],
        arguments=['-d', os.path.join(package_path, 'config', 'main.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Launch it:
    return LaunchDescription([
        rviz
    ])
