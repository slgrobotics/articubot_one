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

    # Keep interface compatible with being included from seggy.launch.py
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
            # driver defaults are used if parameters below are not provided - bno055/src/bno055/bno055/registers.py:255
            # see https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478
            'variance_acc': [0.017, 0.017, 0.017], # [m/s^2]      defaults: [0.017, 0.017, 0.017]
            'variance_angular_vel': [0.04, 0.04, 0.04], # [rad/s] defaults: [0.04, 0.04, 0.04]
            'variance_orientation': [0.0159, 0.0159, 0.0159], # [rad] - (roll, pitch, yaw)  defaults: [0.0159, 0.0159, 0.0159]
            'variance_mag': [0.0, 0.0, 0.0], # [Tesla]            defaults: [0.0, 0.0, 0.0]
        }],
        remappings=[("imu", "imu/data")]
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
        ekf_imu_odom
    ])
