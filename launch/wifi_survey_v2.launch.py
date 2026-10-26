import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("wifi_logger_visualizer"),
        "config",
        "wifi_logger_config.yaml",
    )
    monitoring = LaunchConfiguration("monitoring")
    survey = LaunchConfiguration("survey")
    common = {
        "db_path": LaunchConfiguration("db_path"),
        "map_yaml_path": LaunchConfiguration("map_yaml_path"),
        "cell_size": LaunchConfiguration("cell_size"),
    }
    return LaunchDescription([
        DeclareLaunchArgument("monitoring", default_value="true"),
        DeclareLaunchArgument("survey", default_value="false"),
        DeclareLaunchArgument("visualization", default_value="true"),
        DeclareLaunchArgument("db_path", default_value="~/.ros/titania_wifi.db"),
        DeclareLaunchArgument("map_yaml_path"),
        DeclareLaunchArgument("cell_size", default_value="0.25"),
        DeclareLaunchArgument("target_host", default_value="amdc"),
        Node(
            package="wifi_logger_visualizer",
            executable="wifi_logger_node.py",
            condition=IfCondition(
                PythonExpression(["'", monitoring, "' == 'true' or '", survey, "' == 'true'"])
            ),
            parameters=[config, common],
            output="screen",
        ),
        Node(
            package="wifi_logger_visualizer",
            executable="wifi_visualizer_node.py",
            condition=IfCondition(LaunchConfiguration("visualization")),
            parameters=[config, common],
            output="screen",
        ),
        Node(
            package="wifi_logger_visualizer",
            executable="survey_node.py",
            condition=IfCondition(survey),
            parameters=[
                config,
                common,
                {"target_host": LaunchConfiguration("target_host")},
            ],
            output="screen",
        ),
    ])
