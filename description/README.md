## Download code and build:
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/articubot_one.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ~/robot_ws/install/setup.bash
```

## This launches Gazebo + robot simulation:
```
    cd ~/robot_ws
    colcon build
    source ~/robot_ws/install/setup.bash
    ros2 launch articubot_one sim.launch.py
```

To re-generate robot.urdf.generated.xml manually for troubleshooting:
```
    cd ~/robot_ws
    colcon build
    xacro install/articubot_one/share/articubot_one/robots/sim/description/robot.urdf.xacro sim_mode:=true > src/articubot_one/robots/sim/description/robot.urdf.generated.xml
```
