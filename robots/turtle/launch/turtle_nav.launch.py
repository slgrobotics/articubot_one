import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

#
# To launch nav2 in sim mode on Raspberry Pi because it crashes on my Intel desktop
#

def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    robot_model='turtle'

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    param_substitutions = {
        # Two navigators and their trees supporting different RViz2 mouse-click navigation modes: 
        #'default_nav_through_poses_bt_xml': os.path.join(robot_path,"behavior_trees","navigate_through_poses_w_replanning_and_recovery.xml"),
        'default_nav_to_pose_bt_xml': os.path.join(robot_path,"behavior_trees","odometry_calibration.xml"),
        #'default_nav_to_pose_bt_xml': os.path.join(robot_path,"behavior_trees","navigate_to_pose_w_replanning_and_recovery.xml")
    }

    # file with substituted parameters, will be something like /tmp/tmpxl0yjxcq
    configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     #'use_composition': 'True',
                                     'odom_topic': 'diff_cont/odom',
                                     'autostart' : 'false',
                                     'params_file' : configured_params }.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        LogInfo(msg='starting NAV2: ' + nav2_params_file),
        LogInfo(msg=use_sim_time),

        nav2
    ])
