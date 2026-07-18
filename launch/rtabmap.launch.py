"""Reusable RTAB-Map RGB-D launch for articubot_one robots.

This launch file does not start a camera or robot odometry. Include it after the
robot's sensors/localization are launched and override topics/frames as needed.
Defaults match launch/oakd.launch.py with an OAK-D Lite named ``oak``.

See:
 - https://introlab.github.io/rtabmap/
 - https://github.com/introlab/rtabmap_ros
 - https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch
 - https://wiki.ros.org/rtabmap_ros  (Tutorials)
 - https://wiki.ros.org/rtabmap_ros/Tutorials/MappingAndNavigationOnTurtlebot
 - https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="rtabmap",
            description="Namespace for RTAB-Map nodes.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock.",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="False: mapping; true: localization using an existing database.",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/rtabmap.db",
            description="RTAB-Map database to create or load.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="false",
            description="Delete the database at startup for a fresh map.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="base_link",
            description="Robot base frame used by RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "map_frame_id",
            default_value="map",
            description="Map frame published by RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="",
            description=(
                "Odometry TF frame. Leave empty to subscribe to odom_topic; "
                "set to odom to use TF instead."
            ),
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry/filtered",
            description="External robot odometry topic.",
        ),
        DeclareLaunchArgument(
            "visual_odometry",
            default_value="false",
            description=(
                "Start RTAB-Map RGB-D odometry. Normally false on a robot that "
                "already publishes fused wheel/IMU odometry."
            ),
        ),
        DeclareLaunchArgument(
            "publish_tf_odom",
            default_value="false",
            description="Publish odom->base TF from RTAB-Map visual odometry.",
        ),
        DeclareLaunchArgument(
            "publish_tf_map",
            default_value="true",
            description="Publish map->odom TF from RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/oak/rgb/image_rect",
            description="Rectified RGB image topic.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/oak/stereo/image_raw",
            description="Depth image aligned with the RGB image.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/oak/rgb/camera_info",
            description="CameraInfo corresponding to rgb_topic.",
        ),
        DeclareLaunchArgument(
            "approx_sync",
            default_value="true",
            description="Use approximate synchronization for RGB, depth, and odometry.",
        ),
        DeclareLaunchArgument(
            "approx_sync_max_interval",
            default_value="0.05",
            description="Maximum approximate-sync interval in seconds; 0 is unlimited.",
        ),
        DeclareLaunchArgument(
            "qos",
            default_value="2",
            description="Sensor input QoS: 0 system, 1 reliable, 2 best effort.",
        ),
        DeclareLaunchArgument(
            "topic_queue_size",
            default_value="10",
            description="Subscriber queue size.",
        ),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value="30",
            description="Message-filter synchronization queue size.",
        ),
        DeclareLaunchArgument(
            "rtabmap_viz",
            default_value="true",
            description="Start the RTAB-Map GUI.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RTAB-Map's RViz configuration.",
        ),
        DeclareLaunchArgument(
            "subscribe_scan",
            default_value="false",
            description="Also subscribe to a 2-D laser scan.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan topic used when subscribe_scan is true.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu/data",
            description="Optional filtered IMU topic.",
        ),
        DeclareLaunchArgument(
            "wait_imu_to_init",
            default_value="false",
            description="Wait for an IMU orientation before starting visual odometry.",
        ),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value=(
                "--RGBD/NeighborLinkRefining true "
                "--RGBD/ProximityBySpace true "
                "--Reg/Strategy 0 "
                "--Grid/FromDepth true "
                "--Grid/3D false "
                "--Grid/RayTracing true"
            ),
            description="Additional RTAB-Map command-line parameters.",
        ),
    ]

    # Convert the simple boolean launch argument into RTAB-Map's -d flag without
    # duplicating the upstream launch implementation. The expression evaluates
    # to either '-d <user args>' or just '<user args>'.
    from launch.substitutions import PythonExpression

    args = PythonExpression(
        [
            "('-d ' if '",
            LaunchConfiguration("delete_db_on_start"),
            "'.lower() == 'true' else '') + '''",
            LaunchConfiguration("rtabmap_args"),
            "'''",
        ]
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "localization": LaunchConfiguration("localization"),
            "database_path": LaunchConfiguration("database_path"),
            "frame_id": LaunchConfiguration("frame_id"),
            "map_frame_id": LaunchConfiguration("map_frame_id"),
            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "visual_odometry": LaunchConfiguration("visual_odometry"),
            "publish_tf_odom": LaunchConfiguration("publish_tf_odom"),
            "publish_tf_map": LaunchConfiguration("publish_tf_map"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "depth": "true",
            "stereo": "false",
            "rgbd_sync": "false",
            "subscribe_rgbd": "false",
            "approx_sync": LaunchConfiguration("approx_sync"),
            "approx_sync_max_interval": LaunchConfiguration(
                "approx_sync_max_interval"
            ),
            "qos": LaunchConfiguration("qos"),
            "topic_queue_size": LaunchConfiguration("topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": LaunchConfiguration("rviz"),
            "subscribe_scan": LaunchConfiguration("subscribe_scan"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init"),
            "args": args,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [rtabmap])
