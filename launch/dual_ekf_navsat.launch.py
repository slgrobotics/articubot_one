# Copyright 2018 Open Source Robotics Foundation, Inc.
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

# see https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
#     https://github.com/ros-navigation/navigation2_tutorials/blob/master/nav2_gps_waypoint_follower_demo/launch/dual_ekf_navsat.launch.py

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():

    package_name='articubot_one'

    # Make namespace overridable at runtime
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot specific files reside under "robots" directory - dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    package_path = get_package_share_directory(package_name)

    # define the launch argument that must be passed from the calling launch file or from the console:
    robot_model_arg= DeclareLaunchArgument('robot_model', default_value='')

    robot_model_path = PythonExpression(["'", package_path, "' + '/robots/", robot_model,"'"])
    
    ekf_params_file = PythonExpression(["'", robot_model_path, "' + '/config/dual_ekf_navsat_params.yaml'"])

    navsat_transform_params_file = PythonExpression(["'", robot_model_path, "' + '/config/navsat_transform.yaml'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'
            ),
            DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),

            LogInfo(msg=['============ starting DUAL EKF NAVSAT  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),
            LogInfo(msg=['EKF params file:', ekf_params_file]),
            LogInfo(msg=['navsat transform params file:', navsat_transform_params_file]),

            launch_ros.actions.Node(
                package="robot_localization",
                namespace=namespace,
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                namespace=namespace,
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                namespace=namespace,
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[navsat_transform_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu", "imu/data"),
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
        ]
    )
