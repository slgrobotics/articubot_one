from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from articubot_one.launch_utils.helpers import include_launch

#
# Generate launch description for Turtle robot sensors
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'turtle'  # static per robot type

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from turtle.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # sensor nodes don't depend on robot_model and don't use package_name

    # Lidar node - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md
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
        parameters=[{
            "verbose": True,      # default False
            #"raw_only": True,    # default False ("fusing" mode). When True - only publish raw IMU data - /imu/data_raw and /imu/mag
            "frequency": 30,
            "temp_pub_rate_hz": 1.0,  # temperature publish rate in Hz
            "frame_id": "imu_link",
            "i2c_address": 0x68,  # also, 0x0C shows up for built-in AK8963 magnetometer
            "i2c_port": 1,        # a.k.a "bus". For Linux on Raspberry Pi Bus=1
            "acceleration_scale": [1.0, 1.0, 1.0],  # small adjustment of scale factors for each axis, should be around 1.0
            "acceleration_bias": [0.0, 0.0, 0.0],
            "gyro_bias": [0.0, 0.0, 0.0],
            # use tests/calibrate_mag.py to get mag calibration values
            #"magnetometer_scale": [1.0, 1.0, 1.0],  # should be 1.0 or omitted if "magnetometer_transform" is present
            "magnetometer_bias": [1.879474231677064e-05, 1.2669697764271128e-05, -3.0470527626723397e-05],
            "magnetometer_transform": [
                1.0000000521217702, 1.2535309229370016e-08, -1.6163252600070903e-09,
                1.2535309234403542e-08, 0.9999999234564466, 2.4705503080522403e-08,
                -1.6163252598524059e-09, 2.470550306997046e-08, 1.000000024421789],
            "madgwick_beta": 0.1,       # beta is often in the 0.01–0.2 ballpark, weight of correction from accelerometer/magnetometer vs gyroscope
            "madgwick_use_mag": True
        }]
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
            # see https://github.com/slgrobotics/bno055
            #     https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md
            'ros_topic_prefix': '',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': [0x29,0x28],   # Adafruit - 0x28, GY Clone - 0x29 (with both jumpers closed)
            'data_query_frequency': 30,
            'calib_status_frequency': 0.1,
            'frame_id': 'imu_link',
            # Fast Magnetometer Calibration mode (FMC) provides faster magnetometer calibration
            #  at the cost of slightly higher noise. NDOF_FMC_OFF is the default mode with slower calibration but lower noise.
            #  ACCGYRO and MAGGYRO modes provide raw accelerometer and gyroscope data without sensor fusion,
            #  which can be useful for certain applications but may require additional processing to obtain orientation data.
            'operation_mode': 0x0C, # 0x0C = NDOF (with FMC), 0x0B - NDOF_FMC_OFF, 0x07 - ACC+GYRO+MAG (AMG)
            'placement_axis_remap': 'P1', # P1 - default, ENU. See Bosch BNO055 datasheet section "Axis Remap"
            'acc_factor': 100.0,
            'mag_factor': 16000000.0,
            'gyr_factor': 900.0,
            'grav_factor': 100.0,
            'set_offsets': False, # set to true to use offsets below
            'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
            'offset_gyr': [0x0002, 0xFFFF, 0xFFFF],
            'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
            'radius_mag': 800,   # means 800 microtesla per LSB
            'radius_acc': 1000,  # means 1G = 1000 units LSB
            # Sensor standard deviation [x,y,z]
            # Used to calculate covariance matrices
            # driver defaults are used if parameters below are not provided - bno055/src/bno055/bno055/registers.py:255
            # see https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478
            'variance_acc': [0.017, 0.017, 0.017],  # [m/s^2]      defaults: [0.017, 0.017, 0.017]
            'variance_angular_vel': [0.04, 0.04, 0.04],  # [rad/s] defaults: [0.04, 0.04, 0.04]
            'variance_orientation': [0.0159, 0.0159, 0.0159],  # [rad] - (roll, pitch, yaw)  defaults: [0.0159, 0.0159, 0.0159]
            'variance_mag': [-1.0, 0.0, 0.0],  # [Tesla]           defaults: [-1.0, 0.0, 0.0] - "unknown" covariance, see REP 117
        }],
        remappings=[("imu", "imu/data")]
    )

    icm20948_driver_node = Node(
        package="ros2_icm20948",
        namespace=namespace,
        executable="icm20948_node",
        name="icm20948_node",
        parameters=[{
            # Note: for Linux on Raspberry Pi iBus=1 is hardcoded in linux_i2c.py
            # SparkFun address is likely 0x69, generic GY-ICM20948 - 0x68
            # Use "i2cdetect -y 1"
            "print": True,
            "i2c_address": [0x68, 0x69],  # try both common addresses by default
            "frame_id": "imu_link",
            "raw_only": False,   # default False ("fusing" mode). When True - only publish raw IMU data - /imu/data_raw and /imu/mag
            "pub_rate_hz": 100,  # integer, default 50 in code, 200 here
            "temp_pub_rate_hz": 1.0,     # float, default 1.0
            "startup_calib_seconds": 5.0,      # default 3 seconds
            "gyro_calib_max_std_dps": 2.0,     # warning threshold - if std dev is too high during calibration; default 1.0
            "accel_calib_max_std_mps2": 0.35,  # same for accel; default 0.35
            "magnetometer_bias": [-4.24107093, -26.30849772, 25.83791568],  # Adafruit on Turtle, calibrated using tests/calibrate_mag.py
            "madgwick_beta": 0.05,
            "madgwick_use_mag": True
        }],
    )

    icm20948_driver_raw_node = Node(
        package="ros2_icm20948",
        namespace=namespace,
        executable="icm20948_node",
        name="icm20948_raw_node",
        parameters=[{
            # Note: for Linux on Raspberry Pi iBus=1 is hardcoded in linux_i2c.py
            # SparkFun address is likely 0x69, generic GY-ICM20948 - 0x68
            # Use "i2cdetect -y 1"
            "print": True,
            "i2c_address": [0x68, 0x69],  # try both common addresses by default
            "frame_id": "imu_link",
            "raw_only": True,    # default False ("fusing" mode). When True - only publish raw IMU data - /imu/data_raw and /imu/mag
            "pub_rate_hz": 100,  # integer, default 50 in code, 200 here
            "temp_pub_rate_hz": 1.0,     # float, default 1.0
            "startup_calib_seconds": 5.0,     # default 3 seconds
            "gyro_calib_max_std_dps": 2.0,    # warning threshold - if std dev is too high during calibration; default 1.0
            "accel_calib_max_std_mps2": 0.35,  # same for accel; default 0.35
            "magnetometer_bias": [-4.24107093, -26.30849772, 25.83791568]  # Adafruit on Turtle, calibrated using tests/calibrate_mag.py
        }],
    )

    # Madgwick filter node to compute orientation quaternion *ONLY from raw IMU data* - do not run with "fusing" IMUs!
    # publishes to "imu/data" topic
    # https://github.com/CCNYRoboticsLab/imu_tools
    # sudo apt install ros-${ROS_DISTRO}-imu-tools
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            "stateless": False,
            "use_mag": True,
            "publish_tf": False,
            "reverse_tf": False,
            "fixed_frame": "imu_link",
            "constant_dt": 0.0,
            "publish_debug_topics": False,
            "world_frame": "enu",
            "gain": 0.03,
            "zeta": 0.0,
            "mag_bias_x": 0.0,
            "mag_bias_y": 0.0,
            "mag_bias_z": 0.0,
            "orientation_stddev": 0.0
        }],
        #remappings=[("imu/mag", "imu/mag"), ("imu/data_raw", "imu/data_raw"), ("imu/data", "imu/data")],
    )

    # We need to run an EKF filter here to ensure its output stabilizes before starting SLAM Toolbox or other Localizers.
    # Localizers/mappers only publish the map to odom transform. Robot needs EKF filter to publish odom to base_link transform.
    ekf_imu_odom = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    return LaunchDescription([
        xv_11_driver_node,
        #mpu9250_driver_node,
        bno055_driver_node,
        #icm20948_driver_node,
        #icm20948_driver_raw_node,
        #madgwick_filter_node,
        ekf_imu_odom
    ])
