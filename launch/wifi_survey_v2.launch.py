import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
import yaml


def _launch_nodes(context):
    config = os.path.expanduser(LaunchConfiguration("config_filepath").perform(context))
    with open(config) as stream:
        settings = yaml.safe_load(stream) or {}
    common = {}
    # Empty launch arguments leave the robot YAML values intact.
    for key in ("db_path", "map_yaml_path", "cell_size", "target_host"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            common[key] = float(value) if key == "cell_size" else value

    monitoring = IfCondition(LaunchConfiguration("monitoring")).evaluate(context)
    survey = IfCondition(LaunchConfiguration("survey")).evaluate(context)
    visualization = IfCondition(LaunchConfiguration("visualization")).evaluate(context)
    if not (monitoring or survey or visualization):
        return []
    map_path = common.get("map_yaml_path") or settings.get("/**", {}).get(
        "ros__parameters", {}
    ).get("map_yaml_path", "")
    if not map_path:
        raise RuntimeError(
            "WiFi V2 requires a saved map YAML and PGM image. Set map_yaml_path "
            "in the WiFi config or pass wifi_map_yaml_path:=/path/to/map.yaml "
            "when launching Seggy (map_yaml_path:=... for this launch directly)."
        )
    common["map_yaml_path"] = os.path.abspath(os.path.expanduser(map_path))
    if not os.path.isfile(common["map_yaml_path"]):
        raise RuntimeError(f"WiFi V2 map YAML does not exist: {common['map_yaml_path']}")

    remappings = [
        ("/tf", "tf"), ("/tf_static", "tf_static"),
        ("/localization/recovery_active", "localization/recovery_active"),
        ("/wifi_monitor/status", "wifi_monitor/status"),
        ("/wifi_monitor/cells", "wifi_monitor/cells"),
        ("/wifi_survey/status", "wifi_survey/status"),
    ]
    nodes = []
    for enabled, executable, name in [
        (monitoring or survey, "wifi_logger_node.py", "wifi_logger_node"),
        (visualization, "wifi_visualizer_node.py", "wifi_visualizer_node"),
        (survey, "survey_node.py", "wifi_survey_node"),
    ]:
        if enabled:
            nodes.append(Node(
                package="wifi_logger_visualizer", executable=executable,
                name=name, parameters=[config, common],
                remappings=remappings, output="screen",
            ))
    if IfCondition(LaunchConfiguration("overlay")).evaluate(context):
        nodes.append(Node(
            package="wifi_logger_visualizer", executable="wifi_overlay_node.py",
            name="wifi_overlay_node", parameters=[config], output="screen",
        ))
    viewer = settings.get("/**/wifi_web_viewer", {}).get("ros__parameters", {})
    if IfCondition(LaunchConfiguration("web_viewer")).evaluate(context):
        defaults = settings.get("/**", {}).get("ros__parameters", {})
        db_path = common.get("db_path", defaults.get("db_path", "~/.ros/titania_wifi.db"))
        cell_size = common.get("cell_size", defaults.get("cell_size", 0.25))
        port = viewer.get("port", 8088)
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("wifi_web_viewer port must be an integer between 1 and 65535")
        # The viewer uses argparse, not rclpy: pass CLI arguments without ROS flags.
        command = [
            ExecutableInPackage(package="wifi_logger_visualizer", executable="web_viewer.py"),
            "--db", os.path.abspath(os.path.expanduser(str(db_path))),
            "--cell-size", str(cell_size),
            "--host", str(viewer.get("host", "0.0.0.0")),
            "--port", str(port),
        ]
        if viewer.get("node_names"):
            command.extend(["--node-names", os.path.expanduser(str(viewer["node_names"]))])
        nodes.append(ExecuteProcess(cmd=command, output="screen", additional_env={"PYTHONUNBUFFERED": "1"}))
    return nodes


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("wifi_logger_visualizer"),
        "config", "wifi_logger_config.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_filepath", default_value=config),
        DeclareLaunchArgument("web_viewer", default_value="true"),
        DeclareLaunchArgument("overlay", default_value="true"),
        DeclareLaunchArgument("monitoring", default_value="true"),
        DeclareLaunchArgument("survey", default_value="false"),
        DeclareLaunchArgument("visualization", default_value="true"),
        DeclareLaunchArgument("db_path", default_value=""),
        DeclareLaunchArgument("map_yaml_path", default_value=""),
        DeclareLaunchArgument("cell_size", default_value=""),
        DeclareLaunchArgument("target_host", default_value=""),
        OpaqueFunction(function=_launch_nodes),
    ])
