#!/usr/bin/env python3

"""
Start the ROS 2 adapter for the robot's battery broadcaster.

CMake installs this wrapper so ROS 2 can launch the implementation in
articubot_one.battery_state_compat. With the built workspace sourced and
the battery broadcaster running, use:

     ros2 run articubot_one run_battery_state_compat.py

Expect the node to publish normalized BatteryState messages on
battery/battery_state when the configured battery is uniquely present.
"""

from articubot_one.battery_state_compat import main

if __name__ == '__main__':
    main()
