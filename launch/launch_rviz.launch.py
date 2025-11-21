from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # For any real robot (not a sim), launch on workstation:
    #   ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'main.rviz'])

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

    # topic_tools don't work after October 2025 ROS2 Jazzy update:
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
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'joystick.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        LogInfo(msg=['============ starting RViz and Joystick  namespace: "', namespace, '"  use_sim_time: ', use_sim_time]),

        joystick,
        #battery_pie_chart_relay,  # TODO: topic_tools do not work after October 2025 ROS2 Jazzy update
        rviz,
        rviz_overlay
    ])
