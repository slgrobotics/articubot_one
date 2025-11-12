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

    namespace=''

    package_name='articubot_one'

    package_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = os.path.join(package_path, 'config', 'main.rviz')  # 'view_bot.rviz'  'map.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_overlay = Node(
        package='battery_state_rviz_overlay',
        executable='battery_state_rviz_overlay',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            # see https://github.com/slgrobotics/ros_battery_monitoring/blob/main/battery_state_rviz_overlay/src/battery_state_rviz_overlay_parameters.yaml
            'width' : 450,
            'height' : 10,
            'line_height' : 25,
            'horizontal_distance' : 20,
            'vertical_distance' : 20,
            'font' : 'DejaVu Sans Mono',
            'text_size' : 15.0,
            'horizontal_alignment' : 0,
            'vertical_alignment' : 3,
            'bg_color_a' : 0.1,
            'fg_color_rgba' : '0.0 0.8 0.6 1.0'
        }],
        output='screen',
        remappings=[('battery_state','battery/battery_state')]
    )

    # topic_tools does not work after October 2025 ROS2 Jazzy update:
    battery_pie_chart_relay = Node(
        package='topic_tools',
        namespace=namespace,
        executable='relay_field',
        # executable='transform',
        name='battery_pie_chart_relay',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['battery/battery_state', 'battery/percentage', 'std_msgs/Float32', '{ data: m.percentage }', '--qos-reliability', 'reliable' ]
        #arguments=['battery/battery_state', '--field percentage', 'battery/percentage', 'std_msgs/Float32', '--qos-reliability', 'reliable' ]
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
        #battery_pie_chart_relay,
        rviz,
        rviz_overlay
    ])
