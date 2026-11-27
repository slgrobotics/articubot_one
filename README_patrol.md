Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# Seggy's Four Rooms Patrol Experiment

> **Note:** I am using a fork of [TurtleBot3 Autonomous Patrol package](https://github.com/slgrobotics/turtlebot3-autonomous-patrol) - all credit and copyright belong to the original [author](https://github.com/OmarAds2002).

**Contents:**

- [What are we doing here?](#what-are-we-doing-here)
- [The Four Rooms world](#the-four-rooms-world)
- [Build the patrol package](#build-the-patrol-package)
- [Start Seggy with the saved map](#start-seggy-with-the-saved-map)
- [Start and monitor the patrol](#start-and-monitor-the-patrol)
- [Sim Patrol points](#sim-patrol-points)
- [Save a new map with orientation](#save-a-new-map-with-orientation)
- [Dock staging points](#dock-staging-points)
- [How the patrol works](#how-the-patrol-works)
- [Run the office patrol on the real robot](#run-the-office-patrol-on-the-real-robot)
  - [Office patrol stops](#office-patrol-stops)
  - [Arrival and monitoring](#arrival-and-monitoring)
- [Verification and remaining work](#verification-and-remaining-work)

-------------

## What are we doing here?

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

Pass `robot_world:=four_rooms` to select [Four Rooms](assets/worlds/four_rooms.sdf) in Gazebo.

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

In a second terminal, source the shared robot workspace. The commander starts sending goals as soon as it loads the tree, so check localization before running it.

```bash
source ~/robot_ws/install/setup.bash

ros2 run turtlebot3_autonomous_patrol patrol_commander --ros-args \
  -p use_sim_time:=true \
  -p bt_xml_path:="$HOME/robot_ws/src/articubot_one/robots/seggy/behavior_trees/seggy_four_rooms_patrol_bt.xml"
```

This uses Seggy's running Nav2 stack. Leave the TurtleBot3 simulator and its navigation launch out of this setup. The patrol commander currently expects the global `/navigate_to_pose` action.

Watch the behavior tree in a separate terminal:

```bash
ros2 topic echo /bt_log --field data | grep --line-buffered -E 'NavigateToRoom|Running:|RecoveryRotate|ApproachObject|ChargeBattery'
```

`NavigateToRoom: heading to` shows the stop label, goal, and starting position with straight-line distance when TF is available, and `SUCCESS — arrived at` confirms arrival and reports the stop label, goal coordinates, latest map pose, heading, and XY error when TF is available.
The minute-by-minute `Running:` messages include the goal and current pose when TF is available, plus distance along the Nav2 path, estimated time remaining, and recovery count when fresh action feedback provides them. The filter also shows failures, cancellations, and recovery.
A successful arrival at point 6 followed by a goal for point 1 marks a completed lap.
The commander has no lap counter.

For automatic charging in Four Rooms, start Seggy with the [Four Rooms docking catalogue](robots/seggy/config/docking_four_rooms_sim.yaml). It uses the bays in this world. Verify the approximate dock poses against the active map before starting the patrol.

```bash
ros2 launch articubot_one seggy.launch.py \
  use_sim_time:=true robot_world:=four_rooms localizer_type:=amcl enable_docking:=true \
  map:="$HOME/robot_ws/src/articubot_one/assets/maps/four_rooms.yaml" \
  docking_catalogue:="$HOME/robot_ws/src/articubot_one/robots/seggy/config/docking_four_rooms_sim.yaml"
```

Then launch the commander with the profile from the workspace root:

```bash
cd ~/robot_ws
ros2 run turtlebot3_autonomous_patrol patrol_commander --ros-args \
  --params-file src/articubot_one/robots/seggy/config/patrol_sim.yaml
```

The profile selects `sim_v_bay` and enables Gazebo recharge. The commander logs docking stages on `/bt_log`, starts simulation recharge when docking requests charge confirmation (or after successful positioning if it skips that stage), and confirms from fresh battery status that recharge has stopped before undocking. Patrol resumes after undocking succeeds.

The simulation profile allows two retries after a recoverable docking failure. Before each retry, Nav2 drives to `dock_retry_pose`, a map-frame position and yaw about 2 m west of the V-bay staging pose. If pre-staging fails, the docking retry budget is exhausted, or charging or undocking fails, the commander holds patrol and logs the cause. Restart it after resolving the fault.

The hardware profile selects `real_v_bay` and leaves recharge to the physical dock. Use that profile with Seggy's hardware docking bringup enabled.

For live distance remaining on the current leg:

```bash
ros2 topic echo /navigate_to_pose/_action/feedback --field feedback.distance_remaining
```

## Sim Patrol points

The [Seggy patrol tree](robots/seggy/behavior_trees/seggy_four_rooms_patrol_bt.xml) keeps the six points in the order selected in RViz. The table rounds the coordinates to three decimals. The XML stores the full values.

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
| Battery check | Reads `sensor_msgs/msg/BatteryState` on `/battery/battery_state`. Set `-p low_battery_threshold:=0.25` in the commander’s ROS arguments for a 25% cutoff. The default is 20%. After a low reading, patrol waits for `full_battery_threshold` (default 1.0) or `FULL` status. For a 95% restart threshold, also pass `-p full_battery_threshold:=0.95`. Patrol can start before the first valid reading. |
| Charging | Calls `/dock_robot` for the configured `dock_id`, waits for the full battery condition, then calls `/undock_robot`. |

============================================

## Run the office patrol on the real robot

The [office patrol tree](robots/seggy/behavior_trees/seggy_office_patrol_bt.xml) uses three stops recorded on the physical Seggy. 
Use the saved office map from that run.
The commands below assume the map and workspace are in the same account as the terminal running them.

```bash
# On robot's Raspberry Pi 5:

source ~/robot_ws/install/setup.bash

ros2 launch articubot_one seggy.launch.py \
  use_sim_time:=false \
  localizer_type:=amcl \
  enable_docking:=true \
  map:="$HOME/launch/maps_/seggy_office.yaml"
```

If Seggy is already running with AMCL and the correct map, keep that bringup running. 
In RViz, set the fixed frame to `map`, use **2D Pose Estimate** at the robot's actual location and heading if needed, and check that the lidar scan aligns with the map.
Confirm `ros2 action list` shows `/navigate_to_pose`.

In a Workstation's terminal on the same ROS network, run the Patrol commander. Seggy starts moving as soon as the first goal is accepted.

```bash
cd ~/robot_ws
source install/setup.bash

ros2 run turtlebot3_autonomous_patrol patrol_commander --ros-args \
  --params-file src/articubot_one/robots/seggy/config/patrol_real.yaml
```

### Office patrol stops

| Stop | Map X (m) | Map Y (m) | Recorded yaw (rad) |
| ---: | ---: | ---: | ---: |
| 1 | −0.181 | 0.276 | 1.630 |
| 2 | −2.994 | 4.360 | 3.107 |
| 3 | −4.473 | 0.392 | 0.200 |

These poses were recorded while Seggy was stationary using `ros2 run tf2_ros tf2_echo map base_footprint`. 
The XML keeps the measured headings in comments.
The current `NavigateToRoom` action sends yaw zero, so Seggy faces the positive X direction of the map at each stop.
The [office simulation world](assets/worlds/seggy_office.sdf) marks the same stops with green floor circles.

This is how the office patrol stops are shown on the digital model of the office ([assets/worlds/seggy_office.sdf](https://github.com/slgrobotics/articubot_one/blob/lyrical/assets/worlds/seggy_office.sdf)):

<img alt="Office Patrol Stops" src="https://github.com/user-attachments/assets/1e31c7a1-44d5-4d47-9244-a98b06f174db" />

### Arrival and monitoring

Seggy's standard Nav2 goal checker uses a **7 cm XY radius** and **0.10 rad (about 5.7°) yaw tolerance**. XY is *latched*.
Once Seggy enters the position radius, it can finish turning even if it drifts beyond that radius.
(See [Seggy Nav2 parameters](https://github.com/slgrobotics/articubot_one/blob/lyrical/robots/seggy/config/nav2_params.yaml#L102))

Use the same [monitoring commands](#start-and-monitor-the-patrol) as in simulation. An arrival at point 3 followed by a goal for point 1 marks the start of the next lap.
If you stop the commander while Seggy is moving, check whether a Nav2 goal is still active and cancel it in RViz if needed.

The three poses were sampled on hardware. A complete autonomous office lap has yet to be verified. The patrol tree still uses demo object detection. Verify the docking and charging cycle on hardware before relying on automatic charge stops.

## Verification and remaining work

In testing, Seggy completed numerous six-point laps with very few Nav2 failures. Each lap provides an end-to-end test of the route.

- **Camera inspection.** Connect Seggy's simulated RGB-D stream to a detector, transform detections into `map`, and plan reachable inspection poses beside objects on the tables. Track inspected objects so a repeated detection does not keep interrupting the route.
- **Recovery.** Use Seggy's Nav2 recovery actions or route recovery commands through its stamped velocity and collision-monitor path.
- **Battery and docking.** Verify a full docking, charging, and undocking cycle in each environment and check that the active dock catalogue matches the map. [Seggy docking configuration](robots/seggy/config/docking_sim.yaml)

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
