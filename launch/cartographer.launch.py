# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

# See turtlebot3_ws/src/turtlebot3/turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py
#     ../config/cartographer_lds_2d.lua
#     /opt/ros/jazzy/share/cartographer_ros/launch/

# colcon build; ros2 launch articubot_one cartographer.launch.py use_sim_time:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='articubot_one'

    # Accept namespace from parent launch or use empty default
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot specific files reside under "robots" directory - sim, dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    cartographer_config_dir_default = PathJoinSubstitution([
        FindPackageShare(package_name), 'robots', robot_model, 'config'
    ])

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=cartographer_config_dir_default)

    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        LogInfo(msg=['============ starting CARTOGRAPHER  use_sim_time: ', use_sim_time]),

        Node(
            package='cartographer_ros',
            namespace=namespace,
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[
                ('imu','imu/data'),
                ('scan','scan'),
                #('odom','diff_cont/odom')  # direct mapping
                ('odom','odometry/local')   # ekf filter mapping
                ]
            ),

        Node(
            package='cartographer_ros',
            namespace=namespace,
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution,
                       '-publish_period_sec', publish_period_sec])
    ])
