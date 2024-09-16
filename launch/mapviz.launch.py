import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

# see https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
#     https://github.com/ros-navigation/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo
#     https://swri-robotics.github.io/mapviz/
#
#  source ~/mapviz_ws/install/setup.bash
#  ros2 launch articubot_one mapviz.launch.py
#

package_name='articubot_one' #<--- CHANGE ME

package_path = get_package_share_directory(package_name)

mapviz_config_file = os.path.join(package_path, "config", "gps_mapviz_demo.mvc")


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": mapviz_config_file}]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("gps/fix", "gps/fix"),
            ],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])
