import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node

#
# See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1
#

def generate_launch_description():

    namespace=''

    package_name='articubot_one'

    robot_model='turtle'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher =IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox_params_file = os.path.join(package_path,'robots','turtle','config','mapper_params.yaml')

    # ekf_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    ekf_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','ekf_odom.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    # for experiments: map server, optionally with pre-loaded warehouse map
    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','map_server.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()       # empty_map - default
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items() # warehouse
    )

    # for experiments: a bad alternative to ekf_localizer for slam_toolbox - static transform publisher
    tf_localizer = Node(package = "tf2_ros",
                    executable = "static_transform_publisher",
                    #arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
                    #arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"]
                    arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    slam_toolbox = IncludeLaunchDescription(
                # see /opt/ros/jazzy/share/slam_toolbox/launch
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_toolbox_params_file}.items()
    )

    nav2_params_file = os.path.join(robot_path,'config','nav2_params.yaml')

    # Define the ComposableNodeContainer for Nav2 composition:
    container_nav2 = ComposableNodeContainer(
        package='rclcpp_components',
        namespace=namespace,
        executable='component_container_mt', # _mt for multi-threaded
        name='nav2_container',
        composable_node_descriptions=[], # leave empty, as we are using nav2_launch.py to load components
        parameters=[nav2_params_file],   # must be passed here - see https://github.com/ros-navigation/navigation2/issues/4011
        output='screen'
    )

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','turtle_nav.launch.py')]
#                ), launch_arguments={'use_sim_time': use_sim_time }.items()
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     'use_composition': 'True',
                                     'container_name': 'nav2_container',
                                     'odom_topic': 'odometry/local',
                                     'use_respawn': 'true',
                                     'autostart' : 'true',
                                     'params_file' : nav2_params_file }.items() # pass nav2 params file if not using composition
    )

    #
    # Roomba Create 1 specific nodes:
    #

    create_driver_node = Node(
        package='create_driver',
        namespace=namespace,
        executable='create_driver',
        name='create_driver',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            'robot_model': 'CREATE_1',
            'dev': '/dev/ttyUSB0',
            'baud': 57600,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'latch_cmd_duration': 2.0,
            'loop_hz': 66.0,    # control loop frequency, odom publishing etc. Create 1 sends all sensor data every 15ms (66Hz)
            'publish_tf': False,
            'gyro_offset': 0.0,
            'gyro_scale': 1.21,
            'distance_scale': 1.05
        }],
        remappings=[('cmd_vel', 'diff_cont/cmd_vel'),('odom','diff_cont/odom')]
    )

    xv_11_driver_node = Node(
        package='xv_11_driver',
        namespace=namespace,
        executable='xv_11_driver',
        name='xv_11_driver',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'frame_id': 'laser_frame',
            'firmware_version': 2,
        }]
    )

    mpu9250driver_node = Node(
        package="mpu9250",
        namespace=namespace,
        executable="mpu9250",
        name="mpu9250",
        output='screen',
        respawn=True,
        respawn_delay=4,
        emulate_tty=True,
        parameters=[
            {
                #"print" : True,
                "frequency" : 30,
                "i2c_address" : 0x68,
                "i2c_port" : 1,
                "frame_id" : "imu_link",
                "acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535],
                "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202],
                "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845],
                "magnetometer_scale": [1.0, 1.0, 1.0],
                #"magnetometer_bias": [1.3345253592582676, 2.6689567513691685, -2.5294210260199957],
                #"magnetometer_bias": [1.335, 4.0, 1.0],
                #"magnetometer_bias": [1.335, 3.8, -2.5294210260199957],
                "magnetometer_bias": [1.335, 4.0, -2.53],
                "magnetometer_transform": [1.0246518952703103, -0.0240401565528902, 0.0030740476998857395,
                                        -0.024040156552890175, 0.9926708357001245, 0.002288563295390304,
                                            0.0030740476998857356, 0.0022885632953903268, 0.9837206150979054]
            }
        ],
        remappings=[("imu", "imu/data")]
    )

    bno055_driver_node = Node(
        package='bno055',
        namespace=namespace,
        executable='bno055',
        name='bno055',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            # see https://github.com/flynneva/bno055
            #     https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md
            'ros_topic_prefix': '',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 0x29,   # Adafruit - 0x28, GY Clone - 0x29 (with both jumpers closed)
            'data_query_frequency': 30,
            'calib_status_frequency': 0.1,
            'frame_id': 'imu_link',
            'operation_mode': 0x0C, # 0x0C = FMC_ON, 0x0B - FMC_OFF, 0x05 - ACCGYRO, 0x06 - MAGGYRO
            'placement_axis_remap': 'P1', # P1 - default, ENU. See Bosch BNO055 datasheet section "Axis Remap"
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

    # Let the sensors and base to wake up and stabilize for 10 seconds:
    delayed_slam = TimerAction(period=10.0, actions=[slam_toolbox])

    # Nav2 launch delayed by 20 seconds to allow SLAM Toolbox to start first:
    delayed_nav = TimerAction(period=20.0, actions=[nav2])

    return LaunchDescription([

        create_driver_node,
        xv_11_driver_node,
        #mpu9250driver_node,
        bno055_driver_node,
        robot_state_publisher,
        twist_mux,
        ekf_localizer,
        #tf_localizer,
        #map_server,
        delayed_slam,
        container_nav2,
        delayed_nav
    ])
