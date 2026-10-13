#!/usr/bin/env python3

"""

It’s the executable entry point that lets ROS launch the adapter.
This wrapper exists because CMake currently installs scripts into lib/articubot_one, where ROS finds executables.

This packaging pattern separates the importable node implementation from its ROS executable entry point. 
CMake installs both: 
  - the Python module "articubot_one/battery_state_compat.py" for imports and
  - the launcher "scripts/run_battery_state_compat.py" where ros2 run and launch files can find it.

The two files serve different roles:
  - articubot_one/battery_state_compat.py contains the node logic and functions used by tests.
  - scripts/run_battery_state_compat.py calls main() and is installed by CMake into lib/articubot_one, where ros2 run and launch files find it.

This follows the package’s existing script layout. It runs onboard robots using the official broadcaster; it isn’t launched for Turtle or Gazebo.

Run it:
  ros2 run articubot_one run_battery_state_compat.py

"""

from articubot_one.battery_state_compat import main

if __name__ == '__main__':
    main()
