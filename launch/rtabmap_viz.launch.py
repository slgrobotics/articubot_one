from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch it:
#  ros2 launch articubot_one rtabmap_viz.launch.py

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    odom_topic = LaunchConfiguration('odom_topic')
    frame_id = LaunchConfiguration('frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    wait_for_transform = LaunchConfiguration('wait_for_transform')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='rtabmap',
            description='Namespace used by the RTAB-Map nodes',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odometry/local',
            description='Odometry topic displayed by rtabmap_viz',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Robot base frame',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation or rosbag clock',
        ),
        DeclareLaunchArgument(
            'wait_for_transform',
            default_value='0.2',
            description='Maximum TF wait time in seconds',
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace=namespace,
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'use_sim_time': use_sim_time,
                'wait_for_transform': wait_for_transform,
            }],
            remappings=[
                ('odom', odom_topic),
            ],
            ros_arguments=[
                '--log-level',
                'info',
            ],
        ),
    ])
