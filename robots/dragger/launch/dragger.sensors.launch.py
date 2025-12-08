from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

#
# Generate launch description for Dragger robot sensors
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    package_name = 'articubot_one'

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from dragger.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # sensor nodes don't depend on robot_model and don't use package_name

    # Lidar node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        namespace=namespace,
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[
          {'product_name': 'LDLiDAR_LD14P'},  # LDLiDAR_LD14P setting works for LD-19P LIDAR
          {'laser_scan_topic_name': 'scan'},
          {'point_cloud_2d_topic_name': 'pointcloud2d'},
          {'frame_id': 'laser_frame'},
          {'port_name': '/dev/ttyUSBLDR'},
          {'serial_baudrate' : 230400}, # LD-19P has 230400 baud rate
          {'laser_scan_dir': True},
          {'enable_angle_crop_func': False},
          {'angle_crop_min': 135.0},
          {'angle_crop_max': 225.0},
          {'min_intensity': 45},
          {'do_filtering': False},
          {'do_triplets': False}
        ]
    )

    # IMU node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO085%20IMU.md
    bno08x_config_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'bno085_i2c.yaml'
    ])

    bno08x_driver_node = Node(
        package="bno08x_driver",
        namespace=namespace,
        executable="bno08x_driver",
        name="bno08x_driver",
        output='screen',
        respawn=True,
        respawn_delay=4,
        emulate_tty=True,
        parameters=[bno08x_config_path],
        remappings=[("imu", "imu/data")]
    )

    # IMU node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md
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

    # GPS node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/GPS.md
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

    return LaunchDescription([
        ldlidar_node,
        bno08x_driver_node,
        #mpu9250driver_node,
        gps_node
    ])
