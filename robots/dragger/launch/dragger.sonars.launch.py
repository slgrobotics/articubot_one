from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
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

    def launch_sonar_controllers(context):
        controller_namespace = namespace.perform(context)
        controller_params = PathJoinSubstitution([
            FindPackageShare("articubot_one"), "robots", "dragger", "config", "controllers.yaml"
        ]).perform(context)

        sonar_f_l_spawner = Node(
            package="controller_manager",
            namespace=controller_namespace,
            executable="spawner",
            arguments=["sonar_broadcaster_F_L", "--param-file", controller_params, "--controller-ros-args", "--remap sonar_broadcaster_F_L/range:=sonar_F_L"]
        )

        sonar_f_r_spawner = Node(
            package="controller_manager",
            namespace=controller_namespace,
            executable="spawner",
            arguments=["sonar_broadcaster_F_R", "--param-file", controller_params, "--controller-ros-args", "--remap sonar_broadcaster_F_R/range:=sonar_F_R"]
        )

        sonar_b_l_spawner = Node(
            package="controller_manager",
            namespace=controller_namespace,
            executable="spawner",
            arguments=["sonar_broadcaster_B_L", "--param-file", controller_params, "--controller-ros-args", "--remap sonar_broadcaster_B_L/range:=sonar_B_L"]
        )

        sonar_b_r_spawner = Node(
            package="controller_manager",
            namespace=controller_namespace,
            executable="spawner",
            arguments=["sonar_broadcaster_B_R", "--param-file", controller_params, "--controller-ros-args", "--remap sonar_broadcaster_B_R/range:=sonar_B_R"]
        )

        delayed_sonars_spawner = TimerAction(period=10.0, actions=[sonar_f_l_spawner, sonar_f_r_spawner, sonar_b_l_spawner, sonar_b_r_spawner])

        return [delayed_sonars_spawner]

    # We want to spawn the sonar broadcasters only after the diff drive controller is up, but we don't have diff_drive_spawner here.
    #delayed_sonars_spawner = RegisterEventHandler(
    #    event_handler=OnProcessStart(
    #        target_action=diff_drive_spawner,
    #        on_start=[sonar_f_l_spawner, sonar_f_r_spawner, sonar_b_l_spawner, sonar_b_r_spawner]
    #   )
    #)

    return LaunchDescription([
        OpaqueFunction(function=launch_sonar_controllers)
    ])
