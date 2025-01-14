from launch import LaunchDescription
from launch_ros.actions import Node

#
# See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1
#
# This file goes to ~/launch folder on Create 1 Turtlebot Raspberry Pi
#

def generate_launch_description():

    create_driver_node = Node(
        package='create_driver',
        namespace='',
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
            'latch_cmd_duration': 0.5,
            'loop_hz': 5.0,
            'publish_tf': False,
            'gyro_offset': 0.0,
            'gyro_scale': 1.19,
            'distance_scale': 1.02

        }],
        remappings=[('cmd_vel', 'diff_cont/cmd_vel'),('odom','diff_cont/odom')]
    )

    xv_11_driver_node = Node(
        package='xv_11_driver',
        namespace='',
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
        namespace='',
        executable='bno055',
        name='bno055',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            'ros_topic_prefix': '',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 0x28,
            'data_query_frequency': 20,
            'calib_status_frequency': 0.1,
            'frame_id': 'imu_link',
            'operation_mode': 0x0C,
            'placement_axis_remap': 'P2',
            'acc_factor': 100.0,
            'mag_factor': 16000000.0,
            'gyr_factor': 900.0,
            'set_offsets': False, # set to true to use offsets below
            'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
            'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
            'offset_gyr': [0x0002, 0xFFFF, 0xFFFF]
        }],
        remappings=[("imu", "imu/data")]
    )

    return LaunchDescription([

        create_driver_node,
        xv_11_driver_node,
        #mpu9250driver_node,
        bno055_driver_node

    ])
