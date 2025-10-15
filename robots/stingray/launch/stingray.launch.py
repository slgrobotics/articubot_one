import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    robot_model='stingray'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','stingray_slam_toolbox.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','stingray_cartographer.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    map_yaml_file = os.path.join(package_path,'assets','maps','empty_map.yaml')   # this is default anyway
    #map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','map_server.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()       # empty_map - default
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items() # warehouse
    )

    # odom_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    odom_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','ekf_odom.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation.launch.py')]
        ), launch_arguments={'use_sim_time': use_sim_time,
            #'use_composition': 'True',
            'odom_topic': 'diff_cont/odom',
            #'use_respawn': 'true',
            'autostart' : 'true',
            'params_file' : nav2_params_file }.items()
    )

    roboclaw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("roboclaw_driver"), 
                "launch", "roboclaw_driver.launch.py"
            )
        ),
        launch_arguments={
            "params_file": os.path.join(
                get_package_share_directory("ros2_roboclaw_driver"),
                "config",
                "motor_driver.yaml"
            )
        }.items(),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver',
        output='screen',
        respawn=False,
        respawn_delay=10,
        parameters=['/home/ubuntu/ros_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml'
        ],
	arguments=['--ros-args', '--log-level', 'info']

#          {'product_name': 'YDLIDAR X2L'},
#          {'laser_scan_topic_name': 'scan'},
#          {'point_cloud_2d_topic_name': 'pointcloud2d'},
#          {'frame_id': 'laser_frame'},
#          {'port_name': '/dev/ttyUSBLDR'},
#          {'serial_baudrate' : 115200},
#          {'laser_scan_dir': True},
#          {'enable_angle_crop_func': False},
#          {'angle_crop_min': 135.0},
#          {'angle_crop_max': 225.0}
    )

    bno055_driver_node = Node(
        package='bno055',
        # namespace='',
        executable='bno055',
        name='bno055',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            #   https://github.com/flynneva/bno055
            #   https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md
            'ros_topic_prefix': 'bno055',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 0x28,   # Adafruit - 0x28, GY Clone - 0x29 (with both jumpers closed)
            'data_query_frequency': 20,
            'calib_status_frequency': 0.1,
            'frame_id': 'imu_link',
            'operation_mode': 0x0C, # 0x0C = FMC_ON, 0x0B - FMC_OFF, 0x05 - ACCGYRO, 0x06 - MAGGYRO
            'placement_axis_remap': 'P1', # P1 - default, ENU
            'acc_factor': 100.0,
            'mag_factor': 16000000.0,
            'gyr_factor': 900.0,
            'grav_factor': 100.0,
            'set_offsets': False, # set to true to use offsets below
            'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
            'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
            'offset_gyr': [0x0002, 0xFFFF, 0xFFFF],
            # Sensor standard deviation [x,y,z]
            # Used to calculate covariance matrices
            # defaults are used if parameters below are not provided
            'variance_acc': [0.017, 0.017, 0.017], # [m/s^2]
            'variance_angular_vel': [0.04, 0.04, 0.04], # [rad/s]
            'variance_orientation': [0.0159, 0.0159, 0.0159], # [rad]
            'variance_mag': [0.0, 0.0, 0.0], # [Tesla]
        }],
        remappings=[("imu", "imu/data")]
    )

    drive_include = GroupAction(
        actions=[
            twist_mux,
            # delayed_controller_manager,
            # delayed_diff_drive_spawner,
            # delayed_joint_broad_spawner
        ]
    )

    sensors_include = GroupAction(
        actions=[
            ydlidar_node,
            # bno055_driver_node
        ]
    )

    localizers_include = GroupAction(
        actions=[
            LogInfo(msg='============ starting LOCALIZERS ==============='),
            odom_localizer, # needed for slam_toolbox. cartographer doesn't need it when cartographer.launch.py uses direct mapping
            #navsat_localizer,
            # use either map_server, OR cartographer OR slam_toolbox, as they are all mappers
            # map_server,    # localization is left to GPS
            # cartographer, # localization via LIDAR
            # slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=10.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=20.0, actions=[nav2])

# Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp,
        # joystick,
        drive_include,
        sensors_include,
        roboclaw,
        delayed_loc,
        # delayed_nav
    ])
