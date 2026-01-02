from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#
# A convenience/compatibility wrapper to launch stingray.launch.py with sim time enabled
#
# To launch stingray sim:
#
# cd ~/robot_ws; colcon build; ros2 launch articubot_one stingray_sim.launch.py
#

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    namespace=''

    package_name='articubot_one'

    robot_model='stingray'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    stingray_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), 'robots', robot_model, 'launch', 'stingray.launch.py'])
        ),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time}.items()
    )

    # start the demo autonomy task (script)
    # See /opt/ros/jazzy/lib/python3.12/site-packages/nav2_simple_commander/example_waypoint_follower.py
    #
    # waypoint_follower = Node(
    #    package='nav2_simple_commander',
    #    namespace=namespace,
    #    executable='example_waypoint_follower',
    #    emulate_tty=True,
    #    output='screen',
    # )

    return LaunchDescription([

        stingray_launch,
        #waypoint_follower    # or, "ros2 run articubot_one xy_waypoint_follower.py"
    ])

