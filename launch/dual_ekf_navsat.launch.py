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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

#
# -------------- Deprecated: use https://github.com/slgrobotics/outdoors_loc_nav ---------------------------------------------
#

#
# Generate launch description for robot_localization dual EKF with navsat (GNSS) transform node
#
# Use robot-specific configuration file from robots/<robot_model>/config/dual_ekf_navsat_transform_params_file.yaml
# and robots/<robot_model>/config/navsat_transform.yaml
# (typically specifies wheels odometry IMU inputs and GNSS coordinates to fuse for better odometry)
#
# Example usage (see dragger.localizers.launch.py):
#    navsat_localizer = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource(dual_ekf_navsat_path
#                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model, 'namespace': namespace}.items()
#    )
#

def generate_launch_description():

    package_name = "articubot_one"

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_model = LaunchConfiguration("robot_model")

    # robots/config folder should contain three files:
    #   navsat_transform.yaml    - defines navsat_transform_node parameters to produce /odometry/gps from /gps/fix + /imu/data
    #   ekf_odom_params.yaml     - defines EKF fusing /diff_cont/odom and /imu/data to produce /odometry/local (works indoors too)
    #   ekf_navsat_params.yaml   - defines EKF fusing /odometry/local and /odometry/gps to produce /odometry/global

    ekf_odom_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "robots", robot_model, "config", "ekf_odom_params.yaml"
    ])

    ekf_navsat_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "robots", robot_model, "config", "ekf_navsat_params.yaml"
    ])

    navsat_transform_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), "robots", robot_model, "config", "navsat_transform.yaml"
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace for multi-robot setups"
        ),

        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation clock if true"
        ),

        DeclareLaunchArgument(
            "robot_model", default_value="", description="Robot model directory: seggy, turtle, dragger, etc."
        ),

        LogInfo(msg=["============ starting Dual EKF + NavSat Transform ============ "]),
        LogInfo(msg=["namespace: ", namespace]),
        LogInfo(msg=["robot_model: ", robot_model]),
        LogInfo(msg=["use_sim_time: ", use_sim_time]),
        LogInfo(msg=["EKF params   - odom: ", ekf_odom_params_file]),
        LogInfo(msg=["EKF params - navsat: ", ekf_navsat_params_file]),
        LogInfo(msg=["NavSat Transform params: ", navsat_transform_params_file]),

        # ------------------ EKF (ODOM) ------------------
        Node(
            package="robot_localization",
            executable="ekf_node",
            namespace=namespace,
            name="ekf_filter_node_odom",
            parameters=[ekf_odom_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/local")],
            output="screen"
        ),

        # ------------------ EKF (MAP) ------------------
        Node(
            package="robot_localization",
            executable="ekf_node",
            namespace=namespace,
            name="ekf_filter_node_navsat",
            parameters=[ekf_navsat_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/global")],
            output="screen"
        ),

        # ------------------ NavSat Transform ------------------
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            namespace=namespace,
            name="navsat_transform",
            parameters=[navsat_transform_params_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ('imu', 'imu/data'),                # sensor IMU input
                ('gps/fix', 'gps/fix'),             # raw GPS fix
                ('odometry/filtered', 'odometry/global'),  # odom input. "local" here will cause slow drift from initial position
                ("odometry/gps", "odometry/gps"),   # output odom aligned to GPS
                ('gps/filtered', 'gps/filtered')    # output filtered GPS
            ],
            output="screen"
        ),
    ])
