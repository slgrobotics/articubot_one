from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch

#
# Generate launch description for Seggy robot sensors
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'seggy'  # static per robot type

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
        remappings=[("imu", "imu/data"), ("magnetic_field","imu/mag")]
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

    # Face gesture perception adapter node - prepares data for Behavior Trees
    perception_adapter = Node(
        package="face_gesture_sensor",
        namespace=namespace,
        executable="perception_adapter",
        name="perception_adapter",
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        ldlidar_node,
        bno08x_driver_node,
        ekf_imu_odom,
        oakd_camera,
        face_gesture_sensor,
        perception_adapter
    ])
