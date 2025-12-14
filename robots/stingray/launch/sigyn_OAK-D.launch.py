# Options:
# bt_xml = Full path to behavior tree overriding default_nav_to_pose_bt_xml in the navigation yaml file.
# do_joint_state_gui (false) - Flag to enable joint_state_publisher_gui.
# do_rviz (true) - Launch RViz if true.
# make_map (false) - Make a map vs navigate.
# urdf_file_name (sigyn.urdf.xacro) - URDF file name.
# use_sim_time (true) - Use simulation vs a real robot.
# world (home.world) - World to load if simulating.

import os
import platform
import xacro

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    articubot_package_name='articubot_one' #<--- CHANGE ME
    robot_model='stingray'
    articubot_package_path = get_package_share_directory(articubot_package_name)
    robot_path = os.path.join(articubot_package_path, 'robots', robot_model)

    make_map = LaunchConfiguration("make_map")
    make_map_arg = DeclareLaunchArgument(
        "make_map", default_value="False", description="Make a map vs navigate"
    )
    ld.add_action(make_map_arg)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Simulation mode vs real robot",
    )
    ld.add_action(use_sim_time_arg)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(articubot_package_path, 'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )
    ld.add_action(rsp)


    # Note: controller_manager is provided by Gazebo's gz_ros2_control plugin
    # No need for separate ros2_control_node in simulation
    # controller_params_file = PathJoinSubstitution(
    #     [description_pkg, robot_path, "config", "my_controllers.yaml"])

    # Add delays to ensure Gazebo's controller manager is ready
    delayed_joint_broad_spawner = TimerAction(
        period=3.0,  # Wait for Gazebo controller manager
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["joint_broad"],
        )]
    )
    ld.add_action(delayed_joint_broad_spawner)
    
    # delayed_fwcommand_spawner = TimerAction(
    #     period=4.0,  # Wait after joint broadcaster
    #     actions=[Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         condition=IfCondition(use_sim_time),
    #         arguments=["forward_position_controller", "--param-file", controller_params_file],
    #     )]
    # )
    # ld.add_action(delayed_fwcommand_spawner)

    delayed_diff_drive_spawner = TimerAction(
        period=5.0,  # Wait after other controllers
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["diff_cont"],
        )]
    )
    ld.add_action(delayed_diff_drive_spawner)

    # Bring of the EKF node.
    ekf_config_path = os.path.join(
        robot_path, "config", "ekf_odom_params.yaml"
    )
    start_robot_localization_cmd = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        condition=UnlessCondition(use_sim_time),
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_config_path,
        ],
        remappings=[("/odometry/unfiltered", "/wheel_odom"),("/odometry/filtered", "/odom") ],
    )
    ld.add_action(start_robot_localization_cmd)

    # Launch LDLiDAR LD19
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ldlidar_ros2'), 'launch', 'ld19.launch.py')
        ])
    )
    ld.add_action(ldlidar_launch)

    bno055_params_path = os.path.join(
        get_package_share_directory("bno055"),
        "config",
        "bno055_params.yaml"
    )
    
    bno055_driver_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        respawn=False,          # make True when IMU is reinstalled
        respawn_delay=4,
        parameters=[bno055_params_path],
        # parameters=[{
        #     #   https://github.com/flynneva/bno055
        #     #   https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md
        #     'ros_topic_prefix': 'bno055',
        #     'connection_type': 'i2c',
        #     'i2c_bus': 1,
        #     'i2c_addr': 0x28,   # Adafruit - 0x28, GY Clone - 0x29 (with both jumpers closed)
        #     'data_query_frequency': 20,
        #     'calib_status_frequency': 0.1,
        #     'frame_id': 'imu_link',
        #     'operation_mode': 0x0C, # 0x0C = FMC_ON, 0x0B - FMC_OFF, 0x05 - ACCGYRO, 0x06 - MAGGYRO
        #     'placement_axis_remap': 'P1', # P1 - default, ENU
        #     'acc_factor': 100.0,
        #     'mag_factor': 16000000.0,
        #     'gyr_factor': 900.0,
        #     'grav_factor': 100.0,
        #     'set_offsets': False, # set to true to use offsets below
        #     'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
        #     'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
        #     'offset_gyr': [0x0002, 0xFFFF, 0xFFFF],
        #     # Sensor standard deviation [x,y,z]
        #     # Used to calculate covariance matrices
        #     # defaults are used if parameters below are not provided
        #     'variance_acc': [0.017, 0.017, 0.017], # [m/s^2]
        #     'variance_angular_vel': [0.04, 0.04, 0.04], # [rad/s]
        #     'variance_orientation': [0.0159, 0.0159, 0.0159], # [rad]
        #     'variance_mag': [0.0, 0.0, 0.0], # [Tesla]
        # }],
        remappings=[("imu", "imu/data")]
    )
    # ld.add_action(bno055_driver_node)

    # Launch OAK-D camera
    oakd_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("depthai_ros_driver"),
                "launch",
                "camera.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "camera_model": "OAK-D",
            "parent_frame": "oakd_front_panel",
            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",
        }.items(),
    )
    ld.add_action(oakd_camera)

    # Bring up the twist multiplexer.
    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(articubot_package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(twist_mux)

    # map_path = os.path.join(articubot_package_path, "assets", "maps", "large_map.yaml")
    map_path = os.path.join(articubot_package_path, "assets", "maps", "Stormy.yaml")
    
    nav2_config_path = os.path.join(robot_path, 'config', 'nav2_params.yaml')

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [robot_path, "launch", "nav2_bringup.launch.py"]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "autostart": "True",
            "map": map_path,
            "params_file": nav2_config_path,  # Use original config file
            "slam": "False",
            "use_composition": "True",
            "use_respawn": "True",
            "use_sim_time": use_sim_time,
            "use_localization": "True",
            "container_name": "nav2_container",
        }.items(),
    )
    ld.add_action(nav2_launch)
    
    roboclaw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roboclaw_driver"), 
                "launch", "roboclaw_driver.launch.py"
            )
        ),
        launch_arguments={
            "params_file": os.path.join(
                get_package_share_directory("roboclaw_driver"),
                "config",
                "motor_driver.yaml"
            ),
        }.items(),
    )
    ld.add_action(roboclaw)

    # Include the SLAM Toolbox launch file for mapping
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ],
        ),
        condition=IfCondition(make_map),
        launch_arguments={
            "use_lifecycle_manager": "False",
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                get_package_share_directory("slam_toolbox"),
                "config", "mapper_params_online_async.yaml"
            ),
            # "params_file": "/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml",
        }.items(),
    )
    ld.add_action(slam_toolbox)

    return ld
