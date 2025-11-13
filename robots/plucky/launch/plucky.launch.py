import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    namespace=''

    package_name='articubot_one'

    robot_model='plucky'

    package_path = get_package_share_directory(package_name)

    robot_path = os.path.join(package_path, 'robots', robot_model)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher =IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
    #             ), launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','plucky_slam_toolbox.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(robot_path,'launch','cartographer.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Map server is convenient when used with GPS and an empty map, for obstacle avoidance.
    #map_yaml_file = os.path.join(package_path,'assets','maps','empty_map.yaml')   # this is default anyway
    map_yaml_file = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'

    map_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','map_server.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time}.items()       # empty_map - default
                #), launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items() # warehouse
    )

    # ekf_localizer is needed for slam_toolbox, providing "a valid transform from your configured odom_frame to base_frame"
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see mapper_params.yaml
    ekf_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','ekf_odom.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )

    # for experiments: a bad alternative to ekf_localizer for slam_toolbox - static transform publisher
    tf_localizer = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
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
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time,
                                     'use_composition': 'True',
                                     'container_name': 'nav2_container',
                                     'odom_topic': 'odometry/local',
                                     #'use_respawn': 'true',
                                     'autostart' : 'true',
                                     'params_file' : nav2_params_file }.items() # pass nav2 params file if not using composition
    )

    controllers_params_file = os.path.join(robot_path,'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        namespace=namespace,
        executable="ros2_control_node",
        parameters=[controllers_params_file],
        remappings=[
            ('sonar_broadcaster_F_L/range', 'sonar_F_L'),
            ('sonar_broadcaster_F_R/range', 'sonar_F_R'),
            ('sonar_broadcaster_B_L/range', 'sonar_B_L'),
            ('sonar_broadcaster_B_R/range', 'sonar_B_R')
        ]
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )

    battery_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["battery_state_broadcaster", "--controller-ros-args", "--remap battery_state_broadcaster/battery_state:=battery/battery_state"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["diff_cont", "--controller-ros-args", "--remap /tf:=diff_cont/tf"], # isolate TFs, if published.
        output="screen"
    )

    sonar_f_l_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_F_L"]
    )

    sonar_f_r_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_F_R"]
    )

    sonar_b_l_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_B_L"]
    )

    sonar_b_r_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_B_R"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    delayed_battery_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[battery_state_broadcaster_spawner]
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner]
        )
    )

    delayed_sonars_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_spawner,
            on_start=[sonar_f_l_spawner, sonar_f_r_spawner, sonar_b_l_spawner, sonar_b_r_spawner]
        )
    )

    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        namespace=namespace,
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[
          {'product_name': 'LDLiDAR_LD14'},
          {'laser_scan_topic_name': 'scan'},
          {'point_cloud_2d_topic_name': 'pointcloud2d'},
          {'frame_id': 'laser_frame'},
          {'port_name': '/dev/ttyUSBLDR'},
          {'serial_baudrate' : 115200},
          {'laser_scan_dir': True},
          {'enable_angle_crop_func': False},
          {'angle_crop_min': 135.0},
          {'angle_crop_max': 225.0}
        ]
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
              "frequency" : 60,
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

    gps_node = Node(
        package='nmea_navsat_driver',
        namespace=namespace,
        executable='nmea_serial_driver',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[
            {'port' : '/dev/ttyUSBGPS' },
            {'baud' : 9600 },
            {'frame_id' : 'gps_link' },
            {'time_ref_source' : 'gps' },
            {'use_GNSS_time' : False },
            {'useRMC' : False }
        ],
        remappings=[("fix", "gps/fix")]
    )

    navsat_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','dual_ekf_navsat.launch.py')]
                ), launch_arguments={'use_sim_time': 'false', 'robot_model' : robot_model}.items()
    )

    drive_include = GroupAction(
        actions=[
            twist_mux,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            delayed_battery_state_broadcaster_spawner,
            delayed_sonars_spawner
        ]
    )

    sensors_include = GroupAction(
        actions=[
            ldlidar_node,
            gps_node,
            mpu9250driver_node
            #bno055_driver_node
        ]
    )

    localizers_include = GroupAction(
        actions=[
            LogInfo(msg='============ starting LOCALIZERS ==============='),
            ekf_localizer, # needed for slam_toolbox. cartographer doesn't need it when cartographer.launch.py uses direct mapping
            #tf_localizer,
            #navsat_localizer,
            # use either map_server, OR cartographer OR slam_toolbox, as they are all mappers
            #map_server,    # localization is left to GPS
            #cartographer, # localization via LIDAR
            slam_toolbox, # localization via LIDAR
        ]
    )

    delayed_loc = TimerAction(period=10.0, actions=[localizers_include])

    delayed_nav = TimerAction(period=20.0, actions=[nav2])

    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        robot_state_publisher,
        # joystick,
        drive_include,
        sensors_include,
        delayed_loc,
        container_nav2,  # Add the container to the launch description, if 'use_composition': 'True' is set
        delayed_nav
    ])

