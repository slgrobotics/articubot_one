import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from seggy.launch.py
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
          {'use_sim_time': use_sim_time},
          {'product_name': 'LDLiDAR_LD14'},
          {'laser_scan_topic_name': 'scan'},
          {'point_cloud_2d_topic_name': 'pointcloud2d'},
          {'frame_id': 'laser_frame'},
          {'port_name': '/dev/ttyUSBLDR'},
          {'serial_baudrate' : 115200},
          {'laser_scan_dir': True},
          # Seggy has vertical bar behind the LiDAR, so we crop angles around 180 degrees:
          {'enable_angle_crop_func': True},
          {'angle_crop_min': 170.0},
          {'angle_crop_max': 190.0}
        ]
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
          {'use_sim_time': use_sim_time},
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

    # Face gesture sensor - https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/FaceGesture.md
    face_gesture_sensor = Node(
        package="face_gesture_sensor",
        namespace=namespace,
        executable="fgs_node",
        name="face_gesture_sensor",
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        ldlidar_node,
        mpu9250driver_node,
        face_gesture_sensor
    ])
