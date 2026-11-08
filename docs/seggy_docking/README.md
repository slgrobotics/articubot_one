Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# Seggy Nav2 docking

**Contents:**

- [What is this feature for?](#what-is-this-feature-for)
- [Start the simulation](#start-the-simulation)
- [Enable docking on the real robot](#enable-docking-on-the-real-robot)
- [Configuration and command routing](#configuration-and-command-routing)
- [Dock and undock](#dock-and-undock)
- [Testing and results](#testing-and-results)

-------------

## What is this feature for?

Seggy needs to find its parking bay and drive into it carefully. This feature
connects its laser-based bay detector to Nav2, the software that guides the robot.
You choose a bay by name. Nav2 drives Seggy near it, asks the detector for a more
accurate location, and controls the final approach.

For example, Seggy can park at the V-shaped bay, then reverse out of it.
These are simulation parking tests. Successful parking does not mean the robot
has started charging.

## Start the simulation

Build and source the same Nav2 compatibility overlay used by the plugin:

```bash
source /opt/ros/lyrical/setup.bash
source ~/nav2_compat_ws/install/setup.bash
cd ~/robot_ws
colcon build --packages-select dock_detector_interfaces dock_detector dock_detector_nav2 articubot_one
source install/setup.bash
ros2 launch articubot_one seggy_sim.launch.py
```

Do not launch a second robot or navigation stack alongside an existing one.
Seggy simulation defaults to enable_docking:=true; hardware defaults to false.
To disable this integration:

```bash
ros2 launch articubot_one seggy_sim.launch.py enable_docking:=false
```

Localizers start after the existing 18-second delay; navigation/detector start
after 25 seconds. Wait for active navigation and a usable map before sending goals.

## Enable docking on the real robot

The hardware launch selects docking_real.yaml (real_v_bay) and
docking_nav2_real.yaml when use_sim_time is false. Hardware docking stays disabled
unless you add enable_docking:=true to your normal Seggy launch command.
This starts the named detector and merges the LiDAR plugin/database into the
normal docking_server; /dock_robot then recognizes real_v_bay. A separate trial
server is no longer needed.

Build all four packages on the computer that actually runs Seggy's Nav2 process,
using its matching Nav2 compatibility overlay. A build on a desktop does not
install the plugin on a remote robot. With updated source in that robot workspace:

```bash
source /opt/ros/lyrical/setup.bash
source ~/nav2_compat_ws/install/setup.bash
cd ~/robot_ws
colcon build --packages-select dock_detector_interfaces dock_detector dock_detector_nav2 articubot_one
source install/setup.bash
ros2 pkg prefix dock_detector_nav2
```

Stop the previous normal bringup, then restart with the same saved map and
localization arguments you already use, adding enable_docking:=true. For example,
replace the placeholder map path below with your actual saved map:

```bash
ros2 launch articubot_one seggy.launch.py localizer_type:=amcl \
  map:=/path/to/your/saved_map.yaml enable_docking:=true
```

After localization and lifecycle activation, verify the effective database/plugin:

```bash
ros2 param get /docking_server docks
ros2 param get /docking_server lidar_dock.plugin
ros2 param get /docking_server lidar_dock.dock_direction
ros2 param get /docking_server lidar_dock.staging_yaw_offset
ros2 lifecycle get /docking_server
```

Expected: real_v_bay, dock_detector_nav2::LidarDock, forward, 0.0, and active.
Then one action performs both Nav2 staging and the detector-guided approach:

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: real_v_bay, navigate_to_staging_pose: true}" --feedback
```

The real catalogue's map prior is (2.36116, 0.93915) m, inward yaw -10.79294 degrees,
from the saved real staging survey. Recalibrate it when changing maps or moving
the bay. Real approach limits remain 0.015 m/s, 0.05 rad/s and zero retries.
Arrival remains XY-only at 0.025 m. This integration fixes the missing database
entry; the previous TF control failure and arrival overshoot remain unresolved
and are recorded in the wiki. No hardware action has validated this new normal
bringup integration yet.

## Configuration and command routing

Persistent robot files are in robots/seggy/config:

- nav2_params.yaml: existing standard navigation settings.
- docking_sim.yaml: named bay priors and detector geometry; one source for the
  detector and Nav2 database. Values currently assume map axes match world axes.
- docking_nav2.yaml: simulation docking server/controller tuning and plugin settings.
- docking_real.yaml: real_v_bay map prior and V detector geometry.
- docking_nav2_real.yaml: forward docking with conservative real-robot limits.

At launch, the selected standard Nav2 file is merged with the catalogue and
docking overrides. A temporary runtime parameter file is generated automatically
and removed on shutdown. It is not a separately maintained snapshot; edits to the
standard file are included at the next launch. No manual /tmp configuration is
required. An explicit nav2_params_file override still chooses the base file.
Override docking_catalogue or docking_params_file to select other persistent files.

Catalogue headings point INTO the bay. Current world-derived references:

| Name | Parking X/Y | Inward dock yaw | Robot parked yaw | Staging X/Y |
|---|---|---|---|---|
| trapezoid_bay | -1, 3 | 180 degrees | 180 degrees | -0.5, 3 |
| v_bay | 0, 6 | 90 degrees | 90 degrees | 0, 5.5 |

Seggy now uses forward docking: `lidar_dock.dock_direction: forward` and
`staging_yaw_offset: 0.0` in docking_nav2.yaml. Both staging and parked robot
headings point into the bay. The staging position, detector geometry, dock pose
convention and velocity route are unchanged. `rotate_to_dock` remains false for
continuous LiDAR detection. To restore reverse docking, set the direction to
`backward` and staging yaw offset to pi (3.141592653589793) together.
The generic dock_detector config generator still defaults to backward docking;
Seggy's settings overlay explicitly selects forward docking.

These settings take effect after relaunching the docking server with newly merged
parameters. Building this workspace does not update a running remote robot.
With enable_docking:=true, hardware now selects its real-map catalogue and
replaces the root server docking settings with the LiDAR integration at launch.
Without that flag, the previous standard Nav2 docking settings remain in effect.

The blue floor crosses remain at 0.8 m; Nav2 stages at 0.5 m for more reliable
initial perception.

Calibrate map/world alignment before using these priors. Starting SLAM in another
pose may change the relationship. Detection is constrained by the selected prior;
identical nearby shapes are not uniquely identified by a physical marker.

Docking velocities go through cmd_vel_raw -> velocity smoother -> Collision Monitor
-> cmd_vel_nav -> twist_mux -> diff_cont/cmd_vel. No direct controller bypass is
introduced. The existing experimental dock_controller must not run concurrently.
Default approach limits are 0.035 m/s and 0.08 rad/s, with 90 s approach timeout.
The plugin is noncharging and inherits Nav2's XY-only arrival threshold.

## Dock and undock

Choose either named bay using the normal Nav2 action:

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: trapezoid_bay, navigate_to_staging_pose: true}" --feedback
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: v_bay, navigate_to_staging_pose: true}" --feedback
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot \
  "{dock_type: lidar_dock, max_undocking_time: 60.0}" --feedback
```

Send one goal at a time, allowing docking/undocking to finish before the next.
The plugin selects the detector after staging and accepts only fresh observations
with the matching dock ID and acquisition token. It fails rather than using a
previous bay's pose. Nav2 handles approach failures and configured retries.
Namespaced robots use correspondingly namespaced action names and isolated TF.
Shared-bay reservations remain outside this feature.

## Testing and results

The launch merge tests verify that navigation settings outside docking remain
unchanged, both database identities match the catalogue, robot namespace/clock
are propagated, and disabled docking reads no extra files or starts a detector.

```bash
python3 -m pytest -q src/articubot_one/test/test_docking_launch.py
python3 -m pytest -q src/dock_detector/test
```

The following results used the previous **backward** configuration on 2026-11-07
after a normal simulation restart. They do not validate the new forward approach:

| Trial | Result | Settled world position error | Heading error |
|---|---|---|---|
| Trapezoid dock | Success, one retry | 2.68 cm | 3.26 degrees |
| Trapezoid undock | Success | Forward departure | — |
| V dock | Success, no retries | 2.94 cm | 4.05 degrees |
| V undock | Success | Forward departure | — |
| V repeat dock | Success, no retries | 2.92 cm | 2.61 degrees |

The initial 0.8 m staging trial exhausted retries due to detection gaps. The final
configuration stages at 0.5 m and retains the 0.5-second freshness gate. General
Nav2 navigation still uses its existing 0.3 m XY goal tolerance; lateral staging
error can cause detection loss and retries. These trials demonstrate operation,
not guaranteed reliability across arbitrary starting poses or obstacles.

Ground truth was sampled from Gazebo after two seconds of settling, since motion
continued briefly after the action result. All four command topics ended at zero.
The repeat V trial had zero XY movement over the final half-second observation.
Evidence and reproduction scripts are in
`src/dock_detector/results/2026-11-07-seggy-nav2-sim/`; the wiki records the results.

Both the original and docking-enabled composed navigation containers exited with
code -6 during temporary shutdown/reload. That shutdown issue remains undiagnosed;
the subsequent normal full-simulation restart worked. The active action trials
had no navigation process crash. No hardware/charging validation is implied.

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
