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
                "magnetometer_scale": [1.0, 1.0, 1.0],  # should be around 1.0
                "magnetometer_bias": [1.672994523195427e-05, 1.777942953037992e-05, 3.2817091139903744e-05],
                "magnetometer_transform": [
                    1.0160951390293467, 0.008597352199034276, -0.008498487872556243,
                    0.008597352199034368, 1.0040890425158557, 0.014842492476619326,
                    -0.008498487872556252, 0.014842492476619368, 0.980515572473782],
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
            # driver defaults are used if parameters below are not provided - bno055/src/bno055/bno055/registers.py:255
            # see https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478
            'variance_acc': [0.017, 0.017, 0.017], # [m/s^2]      defaults: [0.017, 0.017, 0.017]
            'variance_angular_vel': [0.04, 0.04, 0.04], # [rad/s] defaults: [0.04, 0.04, 0.04]
            'variance_orientation': [0.0159, 0.0159, 0.0159], # [rad] - (roll, pitch, yaw)  defaults: [0.0159, 0.0159, 0.0159]
            'variance_mag': [0.0, 0.0, 0.0], # [Tesla]            defaults: [0.0, 0.0, 0.0]
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
            "magnetometer_bias": [-3.28, -25.93, 21.88],
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
            "magnetometer_bias": [-3.28, -25.93, 21.88]
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
        #bno055_driver_node,
        icm20948_driver_node,
        #icm20948_driver_raw_node,
        #madgwick_filter_node,
        ekf_imu_odom
    ])
