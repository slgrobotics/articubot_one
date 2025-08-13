#!/bin/bash
# Launch VS Code with ROS2 environment sourced

echo "Sourcing ROS2 environment..."

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source /home/ubuntu/ros_ws/install/setup.bash

echo "ROS_DISTRO: $ROS_DISTRO"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"

# Close any existing VS Code instances
echo "Closing existing VS Code instances..."
pkill -f "code.*articubot_one" || true

# Launch VS Code with the environment
echo "Launching VS Code with ROS2 environment and software rendering..."
code --disable-gpu --disable-software-rasterizer /home/ubuntu/ros_ws/src/articubot_one
