import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py')]
    #             )
    # )

    twist_mux = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','twist_mux.launch.py')]
                ), launch_arguments={'use_sim_time': 'false'}.items()
    )

    slam_toolbox_params_file = os.path.join(package_path,'config','mapper_params_dragger.yaml')

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py')]
                ), launch_arguments={'use_sim_time': 'false', 'slam_params_file': slam_toolbox_params_file}.items()
    )

    # You need to press "Startup" button in RViz when autostart=false
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','navigation_launch.py')]
                #PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("nav2_bringup"),'launch','navigation_launch.py')]
                ), launch_arguments={'use_sim_time': 'false', 'autostart' : 'false'}.items()
    )

    controllers_params_file = os.path.join(package_path,'config','controllers_dragger.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_params_file],
        remappings=[('/tf','/diff_cont/tf')]   # to eliminate publishing link to /tf, although "enable_odom_tf: false" anyway
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner],
        )
    )

    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
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

# Does not deliver orientation, it freezes after the first sample:
#
#     mpu9250driver_node = Node(
#        package='mpu9250driver',
#        executable='mpu9250driver',
#        name='mpu9250driver_node',
#        output='screen',
#        respawn=True,
#        respawn_delay=4,
#        emulate_tty=True,
#        parameters=[
#          {'calibrate': True },
#          {'gyro_range': 0 },     # Gyroscope range: 0 -> +-250°/s, 1 -> +-500°/s, 2 -> +-1000°/s, 3 -> +-2000°/s
#          {'accel_range': 0 },    # Acceleration range: 0 -> +-2g, 1 -> +-4g, 2 -> +-8g, 3 -> +-16g
#          {'dlpf_bandwidth': 2 },   # Digital low pass filter bandwidth [0-6]: 0 -> 260Hz, 1 -> 184Hz, 2 -> 94Hz, 3 -> 44Hz, 4 -> 21Hz, 5 -> 10Hz, 6 -> 5Hz
#          {'gyro_x_offset':  0.0 },  # If "calibrate" is true, these values will be overriden by the calibration procedure
#          {'gyro_y_offset': 0.0},
#          {'gyro_z_offset': 0.0 },
#          {'accel_x_offset': 0.0 },
#          {'accel_y_offset': 0.0 },
#          {'accel_z_offset': 0.0 },
#          {'frequency': 100 }
#        ]
#    )

    mpu9250driver_node = Node(
        package="mpu9250",
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

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[
            {'port' : '/dev/ttyUSBGPS' },
            {'baud' : 115200 },
            #{'baud' : 38400 },
            {'frame_id' : 'gps_link' },
            {'time_ref_source' : 'gps' },
            {'use_GNSS_time' : False },
            {'useRMC' : False }
        ],
        remappings=[("fix", "gps/fix")]
    )

    navsat_localizer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','dual_ekf_navsat.launch.py')]
                ), launch_arguments={'use_sim_time': 'false'}.items()
    )

    drive_include = GroupAction(
        actions=[
            twist_mux,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner
        ]
    )

    sensors_include = GroupAction(
        actions=[
            ldlidar_node,
            gps_node,
            mpu9250driver_node
        ]
    )

    nav_include = GroupAction(
        actions=[
            navsat_localizer,
            slam_toolbox,
            nav2
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        # joystick,
        drive_include,
        sensors_include,
        nav_include
    ])
