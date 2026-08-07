from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from articubot_one.launch_utils.helpers import include_launch

#
# Generate launch description for Dragger's OAK-D camera
#
# This launch file can be used on a separate Raspberry Pi with the OAK-D connected to it ("stereo.local" in my case).
#

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'dragger'  # static per robot type

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from dragger.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # sensor nodes don't depend on robot_model and don't use package_name

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
        oakd_camera
    ])
