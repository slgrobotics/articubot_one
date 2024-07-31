import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node



def generate_launch_description():

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )

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
          {'port_name': '/dev/ttyUSB0'},
          {'serial_baudrate' : 115200},
          {'laser_scan_dir': True},
          {'enable_angle_crop_func': False},
          {'angle_crop_min': 135.0},
          {'angle_crop_max': 225.0}
        ]
    )

    mpu9250driver_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        output='screen',
        respawn=True,
        respawn_delay=4,
        emulate_tty=True,
        parameters=[
          {'calibrate': True },
          {'gyro_range': 0 },     # Gyroscope range: 0 -> +-250°/s, 1 -> +-500°/s, 2 -> +-1000°/s, 3 -> +-2000°/s
          {'accel_range': 0 },    # Acceleration range: 0 -> +-2g, 1 -> +-4g, 2 -> +-8g, 3 -> +-16g
          {'dlpf_bandwidth': 2 },   # Digital low pass filter bandwidth [0-6]: 0 -> 260Hz, 1 -> 184Hz, 2 -> 94Hz, 3 -> 44Hz, 4 -> 21Hz, 5 -> 10Hz, 6 -> 5Hz
          {'gyro_x_offset':  2.0367700 },  # If "calibrate" is true, these values will be overriden by the calibration procedure
          {'gyro_y_offset': -0.0467939 },
          {'gyro_z_offset': -0.1288320 },
          {'accel_x_offset': 0.752060 },
          {'accel_y_offset': 0.278359 },
          {'accel_z_offset': 1.053610 },
          {'frequency': 50 }
        ]
    )

    # Launch them all!
    return LaunchDescription([
        # joystick,
        ldlidar_node,
        mpu9250driver_node,
    ])
