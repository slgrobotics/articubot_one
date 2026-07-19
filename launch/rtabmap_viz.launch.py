from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='rtabmap')
    odom_topic = LaunchConfiguration('odom_topic', default='/odometry/local')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rtabmap'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/local'),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace=namespace,
            output='screen',
            remappings=[
                ('odom', odom_topic),
            ],
            arguments=[
                '--ros-args',
                '--log-level', 'info',
            ],
        ),
    ])
