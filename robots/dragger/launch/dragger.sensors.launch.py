from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from articubot_one.launch_utils.helpers import include_launch

#
# Generate launch description for Dragger robot sensors
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'dragger'  # static per robot type

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

    # BNO085 by default has active DMP (Digital Motion Processor) and does not need running
    #        a imu_filter_madgwick node from imu_tools to fuse raw IMU data into AHRS orientation quaternion.
    #        The driver takes advantage of that. Device does not need calibration.
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
    mpu9250_driver_node = Node(
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
            "verbose": True,      # default False
            #"raw_only": True,    # default False ("fusing" mode). When True - only publish raw IMU data - /imu/data_raw and /imu/mag
            "frequency" : 60,
            "i2c_address" : 0x68,
            "i2c_port" : 1,
            "frame_id" : "imu_link",
            "acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535],
            "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202],
            "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845],
            # use tests/calibrate_mag.py to get mag calibration values
            #"magnetometer_scale": [1.0, 1.0, 1.0],  # should be 1.0 or omitted if "magnetometer_transform" is present
            "magnetometer_bias": [1.879474231677064e-05, 1.2669697764271128e-05, -3.0470527626723397e-05],
            "magnetometer_transform": [
                1.0000000521217702, 1.2535309229370016e-08, -1.6163252600070903e-09,
                1.2535309234403542e-08, 0.9999999234564466, 2.4705503080522403e-08,
                -1.6163252598524059e-09, 2.470550306997046e-08, 1.000000024421789],
            "madgwick_beta": 0.1,       # beta is often in the 0.01–0.2 ballpark, weight of correction from accelerometer/magnetometer vs gyroscope
            "madgwick_use_mag": True
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

    # We need to run an EKF filter here to ensure its output stabilizes before starting SLAM Toolbox or other Localizers.
    # Localizers/mappers only publish the map to odom transform. Robot needs EKF filter to publish odom to base_link transform.
    # "ekf_imu_odom" is needed, providing "a valid transform from your configured odom_frame to base_frame"
    # it does IMU + ODOM fusing. Publishes /odometry/local and TF odom->base_link
    # also, produces odom_topic: /odometry/local which can be used by Nav2
    # see https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api
    # see slam_toolbox_params.yaml

    ekf_imu_odom = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    # OAK-D camera launch We only supply the camera model and its position/rotation relative to the robot base frame. 
    # The rest of the launch arguments are defaults from launch/oakd.launch.py
    # As the parent is camera_oakd_link_optical, the current positioning zero values are reasonable
    oakd_camera = include_launch(
        package_name,
        ['launch', 'oakd.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'name': 'oak',
            'parent_frame': 'camera_oakd_link_optical',
            'camera_model': 'OAK-D-LITE',
            'cam_pos_x': '0.0',
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.0',
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0'
        }
    )

    return LaunchDescription([
        ldlidar_node,
        bno08x_driver_node,
        #mpu9250_driver_node,
        gps_node,
        ekf_imu_odom,
        #oakd_camera
    ])
