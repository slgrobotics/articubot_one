from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

#
# Generate the launch description for twist_mux with namespace support
#
# Note: this is a temporary solution until use_stamped is supported in the official twist_mux launch file
#

def generate_launch_description():

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # See /opt/ros/jazzy/share/twist_mux/launch/twist_mux_launch.py
    #     https://github.com/ros-teleop/twist_mux/tree/rolling/src

    twist_mux_params = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'twist_mux.yaml'])

    # Note: this is how it should be, but "use_stamped" isn't working in that launch file
    #       see /opt/ros/jazzy/share/twist_mux/launch/twist_mux_launch.py
    twist_mux_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('twist_mux'), 'launch', 'twist_mux_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_stamped': 'true',
            'cmd_vel_out': 'diff_cont/cmd_vel',
            'config_topics': twist_mux_params,
        }.items()
    )

    # temporarily use direct Node instantiation until use_stamped is supported in the official launch file:
    twist_mux_node = Node(
        package="twist_mux",
        namespace=namespace,
        executable="twist_mux",
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time, 'use_stamped': True}],
        remappings=[('cmd_vel_out','diff_cont/cmd_vel')]
    )

    # See https://www.youtube.com/watch?v=PN_AxCug5lg
    #     https://answers.ros.org/question/370487/what-is-interactive_marker_twist_server-package/
    #     https://github.com/ros-visualization/interactive_marker_twist_server/tree/humble-devel
    #
    # In robotics, a twist is a 6x1 column vector of linear and angular velocities. In ROS, this is implemented as a Twist message,
    #  which you can see for yourself by entering the following in a terminal:
    #     rosmsg show geometry_msgs/Twist

    #
    # An interactive marker is another ROS tool to interface with robots through RViz. You can drag around this marker,
    #  and generally a robot will try to follow it. The server publishes twists (velocity commands) to get to the interactive marker pose (position command).
    #     sudo apt install ros-${ROS_DISTRO}-interactive-marker-twist-server
    #     ros2 launch interactive_marker_twist_server interactive_markers.launch.xml

    twist_marker_node = Node(
        package='twist_mux',
        namespace=namespace,
        executable='twist_marker',
        output='screen',
        remappings={('twist', 'diff_cont/cmd_vel')},
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_stamped': 'true',
            'frame_id': 'base_link',
            'scale': 1.0,
            'vertical_position': 2.0}]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        LogInfo(msg=['============ starting TWIST_MUX  namespace: "', namespace, '"  use_sim_time: ', use_sim_time]),

        twist_mux_node,
        #twist_marker_node,  # enable if you want interactive marker control from RViz
    ])