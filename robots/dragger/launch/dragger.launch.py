from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.launch_utils import (
    include_launch,
    delayed_include,
    namespace_wrap,
)

#
# Generate launch description for Dragger robot
#
# if launched with "use_sim_time=true", will be running in Gazebo simulation
#
# On the robot's Raspberry Pi:
#
#     ros2 launch articubot_one dragger.launch.py
#
# On the Desktop in simulation:
#
#     ros2 launch articubot_one dragger.launch.py use_sim_time:=true
#

def generate_launch_description():

    package_name = 'articubot_one'
    robot_model = 'dragger'  # static per this robot

    # Launch arguments (can be overridden by parent)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # -------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------
    robot_state_publisher = include_launch(
        package_name,
        ['launch', 'rsp.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
    )

    # -------------------------------------------------------
    # Drive & Sensors
    # -------------------------------------------------------
    drive_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.drive.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
    )

    sensors_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.sensors.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        },
        condition=UnlessCondition(use_sim_time),   # only for real robot
    )

    # -------------------------------------------------------
    # Sonars (real robot + simulation variants)
    # Dragger has four sonars in the corners, we have two separate launch files for them:
    # -------------------------------------------------------

    # Sonar broadcasters for *** real robot ***
    sonars_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.sonars.launch.py'],
        condition=UnlessCondition(use_sim_time)    # real robot only
    )

    # Sonar topic relays (scan->range) for *** Gazebo simulation ***
    sonars_sim_include = include_launch(
        package_name,
        ['launch', 'sonars_sim.launch.py'],
        condition=IfCondition(use_sim_time)        # simulation only
    )

    # -------------------------------------------------------
    # Timed includes (first localizers, then Nav2)
    #
    # Note: TimerAction does not work in included launch files:
    #   https://chatgpt.com/s/t_691df1a57c6c819194bea42f267a8570
    # -------------------------------------------------------

    loc_delay = 15.0   # seconds
    nav_delay = 18.0   # seconds

    localizers_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'dragger.localizers.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }
    )

    navigation_include = include_launch(
        package_name,
        ['launch', 'navigation.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model
        }
    )

    delayed_loc = delayed_include(loc_delay, "LOCALIZERS", localizers_include)
    delayed_nav = delayed_include(nav_delay, "NAVIGATION", navigation_include)

    # -------------------------------------------------------
    # Final Launch Description
    # -------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for dragger nodes'
        ),

        LogInfo(msg=[
            '============ starting DRAGGER (top-level)  namespace="', namespace,
            '"  use_sim_time=', use_sim_time, '  robot_model=', robot_model
        ]),

        #
        # Main robot stack
        #
        robot_state_publisher,
        drive_include,
        sensors_include,
        sonars_include,
        sonars_sim_include,

        #
        # Delayed launch blocks
        #
        delayed_loc,
        delayed_nav,
    ])
