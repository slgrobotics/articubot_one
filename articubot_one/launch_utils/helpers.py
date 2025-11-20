# ===================================================================
#  Reusable launch helper utilities for ROS 2 launch files.
#  Import into any launch file:
#
#      from articubot_one.launch_utils.helpers import (
#              include_launch,
#              delayed_include,
#              namespace_wrap,
#      )
#
# ===================================================================

from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from typing import List


def include_launch(package, path_elements, launch_arguments=None, condition=None):
    """
    Helper to generate an IncludeLaunchDescription action cleanly.

    Args:
        package (str): ROS 2 package name to search in.
        path_elements (list[str]): Path elements under package share.
        launch_arguments (dict): Launch arguments to pass.
        condition: Optional launch condition (IfCondition, UnlessCondition).

    Returns:
        IncludeLaunchDescription()
    """
    if launch_arguments is None:
        launch_arguments = {}

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), *path_elements])
        ),
        launch_arguments=launch_arguments.items(),
        condition=condition,
    )


def delayed_include(delay_seconds, description, include_action):
    """
    Wrap an include action inside a TimerAction with logging.

    Args:
        delay_seconds (float): Delay before the include is triggered.
        description (str): Text for logging.
        include_action: The IncludeLaunchDescription action to delay.

    Returns:
        TimerAction()
    """
    return TimerAction(
        period=delay_seconds,
        actions=[
            LogInfo(msg=[f"============ starting {description} after {delay_seconds} sec"]),
            include_action,
        ]
    )

# ---------------------------------------------------------------------------
# Wrap any actions into a namespace block
# ---------------------------------------------------------------------------
def namespace_wrap(namespace, actions: List):
    """
    Wrap a set of actions under a given namespace.

    Equivalent to:

        GroupAction([
            PushRosNamespace(namespace),
            <actions...>
        ])

    But more readable and reusable across files.
    """
    from launch_ros.actions import PushRosNamespace
    from launch.actions import GroupAction

    return GroupAction(
        [
            PushRosNamespace(namespace),
            *actions,
        ]
    )
