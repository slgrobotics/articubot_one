from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'wifi_logger_visualizer'
    package_dir = get_package_share_directory(package_name)
    config_filepath = os.path.join(package_dir, 'config', 'wifi_logger_config.yaml')
 
    ld.add_action(LogInfo(msg=["[wifi_survey.launch.py] ", " config_filepath: ", config_filepath]))
  
    # Declare launch arguments for nodes:
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(os.getcwd(), 'wifi_data.db'),
        description='Path to the SQLite database file'
    )
    
    config_filepath_arg = DeclareLaunchArgument(
        'config_filepath',
        default_value=config_filepath,
        description='Path to the configuration file'
    )
    
    for action in [
        db_path_arg,
        config_filepath_arg,
    ]:
        ld.add_action(action)
    
    wifi_loger_node = Node(
        package=package_name,
        executable='wifi_logger_node.py',
        name='wifi_logger_node',
        parameters=[config_filepath],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output='screen'
    )
    ld.add_action(wifi_loger_node)

    wifi_visualizer_node = Node(
        package=package_name,
        executable='wifi_visualizer_node.py',
        name='wifi_visualizer_node',
        output='screen',
        parameters=[config_filepath],
    )
    ld.add_action(wifi_visualizer_node)
    
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node.py',
        name='heat_mapper_node',
        parameters=[config_filepath],
        output='screen',
    )
    ld.add_action(heat_mapper_node)

    return ld
