Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# Seggy's Four Rooms Patrol

**Contents:**

- [What is this feature for?](#what-is-this-feature-for)
- [The Four Rooms world](#the-four-rooms-world)
- [Build the patrol package](#build-the-patrol-package)
- [Start Seggy with the saved map](#start-seggy-with-the-saved-map)
- [Start and monitor the patrol](#start-and-monitor-the-patrol)
- [Patrol points](#patrol-points)
- [Save a new map with orientation](#save-a-new-map-with-orientation)
- [Dock staging points](#dock-staging-points)
- [How the patrol works](#how-the-patrol-works)
- [Verification and remaining work](#verification-and-remaining-work)

-------------

## What is this feature for?

Seggy can visit the same places without someone driving it by hand. Give it a list of stops and it will use a map to find its way around walls and furniture. In the Four Rooms simulation, Seggy visits four indoor stops and two outdoor stops, then starts the route again.

<img alt="The Four Rooms world" src="https://github.com/user-attachments/assets/04be0637-5ecd-4947-826f-6fa5d206129b" />


## The Four Rooms world

[Four Rooms](assets/worlds/four_rooms.sdf) is a Gazebo practice site with four furnished rooms and an outdoor yard along the south and west sides. The wall colors are visual landmarks for the four compass directions. The coordinates below are in the Gazebo world frame.

| Wall | Color | Landmark |
| --- | --- | --- |
| North | Blue | Extends west along the yard to meet the arc. |
| South | Red | Doorway near `(2, -0.925)` leads into the yard. |
| West | Green | Doorway near `(-0.925, 2)` leads into the yard. |
| East | Yellow | Extends south along the yard to meet the arc. |
| Interior dividers and yard arc | Brown | Separate the rooms and mark the curved yard boundary. |

The walls are 2.5 m high, 0.15 m thick, and 50% transparent so you can see through them in Gazebo. Their collision geometry still blocks the robot. Four passages connect the rooms, two connect the building to the yard, and one sits at the midpoint of the yard arc. Each has 32 inches (0.8128 m) of clear width. The north and east walls extend to an 18 m radius quarter-circle built from 12 segments.

Each room has a table. The northwest room also has a sofa, the northeast an armchair, the southwest a bookcase flush with the interior divider, and the southeast a bed. The southeast table's edge is 5 cm from the south wall. Tabletops are 29.1 inches (0.74 m) high, with 27.6 inches (0.70 m) of clearance below. The table legs are spaced wide enough for Seggy to enter from a long side when centered.

Eight boxes, cylinders, and spheres give the camera objects to look for on the tables and floor. The northeast sphere is 60 cm across and weighs 50 g. The three floor cylinders weigh 200 g each. Seggy can push those four objects.

Six green circles on the muted green ground mark the patrol stops. The circles are 0.5 m in diameter and have no collision geometry. The yard also has two reverse-docking practice bays. The V-shaped bay is south of the building and opens west. The trapezoidal bay is west of the building and opens south. Painted crosses and arrows mark the approach and docking positions. See [Dock staging points](#dock-staging-points) for the coordinates.

## Build the patrol package

The C++ patrol commander lives in the separate `turtlebot3_autonomous_patrol` package under `~/robot_ws/src`. The `articubot_one` code is cloned there as well.

From a terminal:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# To run Seggy sim:
git clone https://github.com/slgrobotics/articubot_one.git
git clone https://github.com/slgrobotics/ros_battery_monitoring.git
git clone https://github.com/slgrobotics/scan_to_range.git
git clone https://github.com/slgrobotics/outdoors_loc_nav.git
git clone https://github.com/slgrobotics/rviz_attitude_plugin.git
git clone https://github.com/slgrobotics/dock_detector.git
git clone https://github.com/slgrobotics/dock_detector_interfaces.git
git clone https://github.com/slgrobotics/dock_detector_nav2.git
git clone https://github.com/slgrobotics/map_orientation_saver.git
git clone https://github.com/slgrobotics/slg_bt_plugins.git

# and, finally the patrol package:
git clone https://github.com/slgrobotics/turtlebot3-autonomous-patrol.git   # this pulls "lyrical" branch

cd ~/robot_ws

sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y --skip-keys ament_python

colcon build
source install/setup.bash
```

## Start Seggy with the saved map

Edit Seggy's simulation drive (`~robot_ws/src/articubot_one/robots/seggy/launch/seggy.drive.launch.py` ) launch to select [four_rooms.sdf](assets/worlds/four_rooms.sdf): `'robot_world': 'four_rooms',`

From a terminal with `~/robot_ws/install/setup.bash` sourced:

```bash
ros2 launch articubot_one seggy.launch.py \
  use_sim_time:=true \
  localizer_type:=amcl \
  map:="$HOME/robot_ws/src/articubot_one/assets/maps/four_rooms.yaml"
```

This command starts Gazebo, RViz2, Joystick driver node and all robot's nodes including ACML and Nav2.
The map was generated by driving Seggy in the *four_rooms* world, covering all areas and saving the map.

The robot spawns near `(0, 0)`, aimed about 19° north of east. In RViz, set the fixed frame to `map` and use **2D Pose Estimate** near that position and heading.

## Start and monitor the patrol

In a second terminal, source the same ROS environment and the patrol workspace. The commander starts sending goals as soon as it loads the tree, so check localization before running it.

```bash
source ~/robot_ws/install/setup.bash

ros2 run turtlebot3_autonomous_patrol patrol_commander --ros-args \
  -p use_sim_time:=true \
  -p bt_xml_path:="$HOME/robot_ws/src/articubot_one/robots/seggy/config/seggy_four_rooms_patrol_bt.xml"
```

This uses Seggy's running Nav2 stack. Leave the TurtleBot3 simulator and its navigation launch out of this setup. The patrol commander currently expects the global `/navigate_to_pose` action.

Watch the behavior tree in a separate terminal:

```bash
ros2 topic echo /bt_log --field data | grep --line-buffered -E 'NavigateToRoom|RecoveryRotate|ApproachObject|ChargeBattery'
```

`NavigateToRoom: heading to (x, y)` identifies the current stop, and `SUCCESS — arrived at room` confirms arrival.
The filter also shows failures, cancellations, and recovery.
A successful arrival at point 6 followed by a goal for point 1 marks a completed lap.
The commander has no lap counter.

For live distance remaining on the current leg:

```bash
ros2 topic echo /navigate_to_pose/_action/feedback --field feedback.distance_remaining
```

## Patrol points

The [Seggy patrol tree](robots/seggy/config/seggy_four_rooms_patrol_bt.xml) keeps the six points in the order selected in RViz. The table rounds the coordinates to three decimals. The XML stores the full values.

| Stop | X (m) | Y (m) |
| ---: | ---: | ---: |
| 1 | 0.647 | 2.707 |
| 2 | 2.529 | 5.791 |
| 3 | 7.030 | 6.963 |
| 4 | 6.157 | 1.648 |
| 5 | 2.814 | −4.647 |
| 6 | −4.457 | 3.534 |

To record a different route, use RViz **Publish Point** with fixed frame `map` while running `ros2 topic echo /clicked_point`. Copy X and Y in visit order and verify each message has `frame_id: map`. **Publish Point** records position only. The current `NavigateToRoom` action sets yaw to zero. Use **Navigate Through Poses** to test a route once you have selected the stops.

Gazebo world coordinates and ROS map coordinates have different origins. The green circles are 0.5 m diameter and use the measured map-to-world alignment. If you regenerate the map, verify the route in the new frame before reusing the XML or moving the circles.

## Save a new map with orientation

Use SLAM Toolbox to map the rooms, outdoor paths, and docking approaches. In a sourced terminal, run the orientation-aware saver while SLAM publishes `/map`:

```bash
mkdir -p ~/launch/maps
ros2 run map_orientation_saver map_saver_with_orientation \
  -f ~/launch/maps/four_rooms \
  --ros-args -p use_sim_time:=true \
  --params-file "$(ros2 pkg prefix --share map_orientation_saver)/config/map_saver_params.yaml"
```

The saved [map YAML](assets/maps/four_rooms.yaml) references the occupancy image at 5 cm per pixel. AMCL loads those two files. The [orientation sidecar](assets/maps/four_rooms.orientation.yaml) records the map's heading estimate. The `four_rooms_serialized.data` and `four_rooms_serialized.posegraph` files support resuming SLAM Toolbox mapping. [Map orientation guide](docs/map_orientation.md)

## Dock staging points

Each staging cross is 0.8 m from its target cross, toward the bay opening. The yaw points away from the bay so Seggy can reverse in.
Map coordinates are approximate, based on the live Gazebo pose and ROS transform measured at the fixed start.

| Bay | Gazebo X (m) | Gazebo Y (m) | Map X (m) | Map Y (m) | Map yaw (rad) |
| --- | ---: | ---: | ---: | ---: | ---: |
| V-shaped | 6.903 | −3.802 | 6.922 | −3.797 | 3.142 (west) |
| Trapezoidal | −4.475 | 6.903 | −4.456 | 6.908 | −1.571 (south) |

Both staging centers fall on free cells in the saved map. These are separate from the six patrol stops.

## How the patrol works

- The commander ticks a BehaviorTree.CPP tree at 10 Hz.
- The tree sends one Nav2 goal at a time.
- An object detection can interrupt a goal, causing the navigation action to cancel it and letting the approach branch run.
- A failed navigation can trigger a spin and retry.
- A battery check gates the route on each tick.

| Function | Current behavior |
| --- | --- |
| Navigation | Sends `NavigateToPose` goals in `map`, in the configured order. |
| Object detection | Demo subscriber listens for a string on `/detections` and supplies a fixed inspection pose. |
| Recovery | Demo spin publishes an unstamped `Twist` on `cmd_vel`, followed by a short wait and retry. |
| Battery check | Listens for a `Float32` on `/battery_state`, defaults to 100%, and branches to charging below 20%. |
| Charging | Demo action logs and returns success. |

## Verification and remaining work

In testing, Seggy completed numerous six-point laps with very few Nav2 failures. Each lap provides an end-to-end test of the route.

- **Camera inspection.** Connect Seggy's simulated RGB-D stream to a detector, transform detections into `map`, and plan reachable inspection poses beside objects on the tables. Track inspected objects so a repeated detection does not keep interrupting the route.
- **Recovery.** Use Seggy's Nav2 recovery actions or route recovery commands through its stamped velocity and collision-monitor path.
- **Battery and docking.** Read Seggy's `sensor_msgs/msg/BatteryState` on `battery/battery_state`. Connect the charge branch to Seggy's docking stack, configure the relocated bays for this map, and verify the transition from docking to charging and back to patrol. [Seggy docking configuration](robots/seggy/config/docking_sim.yaml)

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
