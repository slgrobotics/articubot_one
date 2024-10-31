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
    #     https://github.com/ros-teleop/twist_mux/tree/rolling/src

    twist_mux_params = os.path.join(package_path,'config','twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace='/',
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
    # An interactive marker is another ROS tool to interface with robots through rviz. You can drag around this marker,
    #  and generally a robot will try to follow it. The server publishes twists (velocity commands) to get to the interactive marker pose (position command).
    #     sudo apt install ros-${ROS_DISTRO}-interactive-marker-twist-server
    #     ros2 launch interactive_marker_twist_server interactive_markers.launch.xml

    twist_marker = Node(
        package='twist_mux',
        executable='twist_marker',
        namespace='/',
        output='screen',
        remappings={('twist', 'diff_cont/cmd_vel')},
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_stamped': 'true',
            'frame_id': 'base_link',
            'scale': 1.0,
            'vertical_position': 2.0}]
    )

    joystick_params_file = os.path.join(package_path,'config','joystick.yaml')

    # see https://github.com/ros-teleop/twist_mux/blob/rolling/scripts/joystick_relay.py
    # currently doesn't support 'use_stamped'
    joystick_relay = Node(
        package='twist_mux',
        executable='joystick_relay.py',
        namespace='/',
        output='screen',
        remappings={('joy_vel_in', 'cmd_vel_joy'),
                    ('joy_vel_out', 'joy_vel')},
        parameters=[joystick_params_file, {
            'use_sim_time': use_sim_time,
            'use_stamped': 'true'
            }]
    )

    # this is how it should be, but "use_stamped" isn't working in that launch file
    # see /opt/ros/jazzy/share/twist_mux/launch/twist_mux_launch.py
    twist_mux_ = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("twist_mux"),'launch','twist_mux_launch.py')]
                ), launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_stamped': 'true',
                    'cmd_vel_out': 'diff_cont/cmd_vel',
                    'config_topics': twist_mux_params,
                    }.items()
    )

    return LaunchDescription([
        twist_mux,
        #twist_marker,
        #joystick_relay
    ])