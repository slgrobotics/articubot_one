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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
import launch_ros.actions

#
# Generate launch description for robot_localization EKF odometry node
#
# Use robot-specific configuration file from robots/<robot_model>/config/ekf_odom_params.yaml
# (typically specifies wheels odometry and IMU inputs to fuse for better odometry)
#
# Example usage (see seggy.localizers.launch.py):
#    ekf_localizer = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource(ekf_odom_path)
#                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model, 'namespace': namespace}.items()
#    )

def generate_launch_description():

    package_name='articubot_one'

    # Make namespace overridable at runtime
    namespace = LaunchConfiguration('namespace', default='')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot specific files reside under "robots" directory - dragger, plucky, seggy, turtle...
    robot_model = LaunchConfiguration('robot_model', default='')

    package_path = get_package_share_directory(package_name)

    robot_model_path = PythonExpression(["'", package_path, "' + '/robots/", robot_model,"'"])
    
    ekf_params_file = PythonExpression(["'", robot_model_path, "' + '/config/ekf_odom_params.yaml'"])

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
                "output_location", default_value="~/ekf_odom_example_debug.txt"
            ),

            LogInfo(msg=['============ starting EKF ODOM  namespace: "', namespace, '"  use_sim_time: ', use_sim_time, ', robot_model: ', robot_model]),
            LogInfo(msg=['EKF params file:', ekf_params_file]),

            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/local")],
            )
        ]
    )
