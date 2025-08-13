#!/bin/bash
# Generate URDF for preview
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros_ws/install/setup.bash

echo "Generating URDF for preview..."
xacro robots/stingray/description/robot.urdf.xacro sim_mode:=true > robots/stingray/description/robot_generated.urdf

if [ $? -eq 0 ]; then
    echo "✅ Generated robot_generated.urdf successfully"
    echo "You can now preview robots/stingray/description/robot_generated.urdf in VS Code"
else
    echo "❌ Failed to generate URDF"
fi
