from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

#
# Generate launch description for Dragger robot sensors
#
#   *** runs on real robot only, not in simulation ***
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # sensor nodes don't depend on robot_model and don't use package_name
    # they run on real robot only, not in simulation

    sonar_f_l_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_F_L", "--controller-ros-args", "--remap sonar_broadcaster_F_L/range:=sonar_F_L"]
    )

    sonar_f_r_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_F_R", "--controller-ros-args", "--remap sonar_broadcaster_F_R/range:=sonar_F_R"]
    )

    sonar_b_l_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_B_L", "--controller-ros-args", "--remap sonar_broadcaster_B_L/range:=sonar_B_L"]
    )

    sonar_b_r_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["sonar_broadcaster_B_R", "--controller-ros-args", "--remap sonar_broadcaster_B_R/range:=sonar_B_R"]
    )

    delayed_sonars_spawner = TimerAction(period=10.0, actions=[sonar_f_l_spawner, sonar_f_r_spawner, sonar_b_l_spawner, sonar_b_r_spawner])

    # We want to spawn the sonar broadcasters only after the diff drive controller is up, but we don't have diff_drive_spawner here.
    #delayed_sonars_spawner = RegisterEventHandler(
    #    event_handler=OnProcessStart(
    #        target_action=diff_drive_spawner,
    #        on_start=[sonar_f_l_spawner, sonar_f_r_spawner, sonar_b_l_spawner, sonar_b_r_spawner]
    #   )
    #)

    return LaunchDescription([
        delayed_sonars_spawner
    ])
