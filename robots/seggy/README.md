# Seggy configuration

**Contents:**

- [Hardware settings](#hardware-settings)
- [Related settings](#related-settings)
  - [Wi-Fi V2: waiting for the first saved map](#wi-fi-v2-waiting-for-the-first-saved-map)
  - [Docking configuration](#docking-configuration)
- [Direct firmware square test](#direct-firmware-square-test)

-------------

## Hardware settings

`config/hardware.yaml` contains the serial connection, encoder calibration,
motor conversion factor, and battery settings for the Arduino driver.
The real-hardware branch of `description/ros2_control.xacro` loads this file.
Simulation does not load it.

The packaged file is used by default:

```bash
ros2 launch articubot_one seggy.launch.py
```

To use a machine-specific copy:

```bash
ros2 launch articubot_one seggy.launch.py \
  hardware_config:=/absolute/path/to/seggy_hardware.yaml
```

The same override is available when expanding the robot description directly:

```bash
xacro robots/seggy/description/robot.urdf.xacro sim_mode:=false \
  hardware_config:=/absolute/path/to/seggy_hardware.yaml
```

> Rebuild and source the workspace after adding the packaged YAML. Hardware
> settings are read during initialization; restart the hardware stack to apply changes.

## Related settings

- Wheel joint names, `main_battery`, and the unused nominal `loop_rate` remain in Xacro.
- Wheel geometry and the actual controller-manager update rate remain in `config/controllers.yaml`.
- Battery broadcaster metadata also lives in `config/controllers.yaml`. Keep its
  design capacity and chemistry consistent with `hardware.yaml`; its estimated
  full capacity is currently 90% of design capacity.
- Simulation battery properties remain in `description/robot.urdf.xacro`.


### Wi-Fi V2: waiting for the first saved map

A configured map path such as `/home/ros/launch/seggy_office.yaml` need not
exist when Seggy starts. After the normal Wi-Fi bringup delay, the V2 suite
checks for readable YAML and a complete PGM image every five seconds at INFO
level. Mapping/localization continues. Saving the map to the configured path
automatically starts the enabled Wi-Fi nodes, overlay, and browser viewer;
an orientation sidecar is not required. No launch restart is needed for this
first save. A subsequent map change requires restarting the Wi-Fi suite.

Set `map_check_interval` in the shared `/**` Wi-Fi YAML parameters to adjust
polling, or pass it when launching `wifi_survey_v2.launch.py` directly. Keep
`map_yaml_path` configured: the suite cannot guess the desired saved filename.


### Docking configuration

For **real Seggy’s `real_v_bay`**, the checked-in configuration puts the **parking pose 60 cm out from the V apex** and the **staging pose another 50 cm out**, both measured at `base_link` along the dock centerline. Seggy faces into the bay at both poses. These are configured distances, not measurements from the running robot.
- [Hardware catalogue](https://github.com/slgrobotics/articubot_one/tree/lyrical/robots/seggy/config/docking_real.yaml) 
- [Hardware docking settings](https://github.com/slgrobotics/articubot_one/tree/lyrical/robots/seggy/config/docking_nav2_real.yaml)

| Point in the sequence | Nominal `base_link` distance from the V apex |
|---|---:|
| Staging | **110 cm** |
| LiDAR parking target, before insertion | **60 cm** |
| Charging insertion | Starts from the actual aligned pose. A full **15 cm** from nominal parking would put `base_link` **45 cm** from the apex. Charging or the braking allowance can stop it earlier. |
| Undock target | **50 cm back from where undocking starts**. If it starts at the nominal full-insertion point, that puts the retreat target **95 cm** from the apex. |

The key distinction is that **Nav2’s XY tolerance is a radius around its goal**, while the **LiDAR staging gate checks lateral offset from the detected centerline**.

| Step | Current check |
|---|---|
| Skip initial staging navigation | `dock_prestaging_tolerance` **10 cm XY** from the configured staging point. This check can skip the navigation leg. |
| Complete initial staging navigation | General Nav2 goal checker **30 cm XY**, **5.7° yaw**. XY latches once accepted. NavFn separately has a **20 cm** planning tolerance for finding a reachable endpoint. |
| Pass the LiDAR staging gate | Within **5 cm lateral** and **5° heading** of the detected dock axis. Seggy must also be at least **20 cm outside the parking plane**. That is a minimum approach distance. |
| Complete a LiDAR re-staging move | Nav2 temporarily uses **7 cm XY** and **5° yaw**, with XY latching inherited from the normal goal checker. The fresh LiDAR staging gate runs again afterward. |
| Pass the near-dock guard | In the final **8 cm** before parking, lateral error of **2.5 cm or more** aborts the approach. At or beyond the parking plane, being outside the **2.5 cm XY** arrival radius also aborts. |
| Finish Nav2 positioning | Within **2.5 cm XY** of the filtered LiDAR target. The wrapper applies the final heading requirement in the next phase. |
| Finish yaw alignment | Within **4.5 cm XY** of the wrapper’s filtered target (**2.5 cm + 2 cm margin**) and **5° yaw**. |
| Stay within the insertion corridor | Relative to the insertion start, at most **1 cm lateral drift** and **5° heading error**. More than **5 mm backward** aborts. The **15 cm travel cap** is measured from insertion start, with **3 cm plus a velocity-based allowance** reserved for braking. |
| Finish undocking | Within **5 cm XY** and **5.7° yaw** of the retreat target. |

The detector has separate association limits. It searches within **95 cm** of the expected landmark, accepts a fitted pose within **45 cm** of its prior and **25°** of its expected heading, and allows **±11°** on the V opening angle. Those settings affect *which detection is accepted*. The staging and arrival checks use the accepted detection to decide whether Seggy can proceed. [Detector catalogue](https://github.com/slgrobotics/articubot_one/tree/lyrical/robots/seggy/config/docking_real.yaml) · [Approach checks](/home/sergei/robot_ws/src/dock_detector_nav2/src/lidar_dock.cpp) · [Alignment and insertion](/home/sergei/robot_ws/src/dock_detector/dock_detector/docking_alignment.py)

If you want to ease one value, I’d start with **staging heading**, from **5° to 6°**, and measure the effect. One logged rejection still had about **2 cm lateral error** but **5.9° heading error** after correction. I’d keep the **5 cm lateral gate** for now. A previous run passed it at **4.97 cm off-center** and still triggered the near-dock guard. Widening `docking_threshold` would also widen the guard’s allowed lateral error and the wrapper’s alignment radius, so it affects three decisions at once.

## Direct firmware square test

`scripts/seggy_firmware_square.py` runs without ROS or third-party Python packages.
Run it on Seggy with the wheel firmware available at `/dev/ttyACM0` and all ROS
hardware drivers and other serial clients stopped. It requires Linux's `fuser`
utility to check for an existing serial client. Direct commands bypass Nav2,
collision monitoring, and `twist_mux`; keep the complete square clear and the
physical stop control available. Firmware joystick input must remain inactive.

From the workspace root, preview the route and nominal speeds:

```bash
python3 src/articubot_one/scripts/seggy_firmware_square.py \
  --linear-percent 15 --angular-percent 20
```

Add `--execute` to open the port and move. This is a manual test command; the
script does not start as part of robot bringup.

```bash
python3 src/articubot_one/scripts/seggy_firmware_square.py \
  --port /dev/ttyACM0 --side 1.0 --cycles 1 --direction ccw \
  --linear-percent 15 --angular-percent 20 --execute
```

Percentages are integers from 1 to 100. Linear percent commands both wheels
forward equally. Angular percent commands the same wheel-speed magnitude in
opposite directions for in-place turns; it is a percentage of the maximum
in-place robot turn rate, not a rad/s value. At the firmware's 3.5 rad/s wheel
maximum, 15% linear gives a nominal 0.064 m/s and 20% angular gives 0.361 rad/s.
The ROS `wheels_pwm_factor` is not used by this direct percent interface.

Each circuit consists of four straight sides and four 90-degree turns. The
script uses cumulative encoder counts to end each segment, sends zero velocity,
and waits for unchanged counts before the next segment. Speeds are step commands
with no ROS smoothing or tapering. Inertia and whole-count resolution can cause
overshoot, which is included in segment results. There is no external localization
or correction to the original square corners.

Geometry defaults match Seggy's controller settings: wheel radius 0.122 m,
separation 0.473 m, and 60 counts per wheel revolution. Override these with
`--wheel-radius`, `--wheel-separation`, and `--counts-per-rev`; use
`--motor-max-rad-s` if the firmware maximum changes. The port defaults to
115200 baud (`--baud`). Commands and count samples default to 20 Hz (`--rate`).

The JSONL log includes parameters, raw commands/replies, cumulative wheel travel,
estimated local X/Y/yaw, measured linear/angular speeds, segment durations and
overshoot, stop confirmation, and final estimated closure error. Counts are coarse:
one count is about 1.28 cm at the wheel, so instantaneous speed samples are noisy.
Wheel odometry cannot independently measure slip or actual physical square accuracy.
Use `--output /absolute/path/new-run.jsonl` for a chosen log filename.

Missing replies, malformed replies, wrong-direction travel, no count changes for
five seconds, or segment timeouts abort the run. The script attempts zero commands
and encoder-based stop confirmation on errors, Ctrl+C, and SIGTERM. If serial
communication fails, stop confirmation is unavailable; the firmware's two-second
command watchdog is the fallback. Encoder silence cannot prove a physical stop if
the sensors have failed. `--stall-timeout`, `--segment-timeout`, `--serial-timeout`,
and `--settle` adjust the bounded checks; see `--help` for defaults.

Offline verification, with no robot access:

```bash
python3 -m unittest discover -s src/articubot_one/test \
  -p test_seggy_firmware_square.py
```

The offline checks exercise clockwise/counterclockwise square integration with
Hall-count quantization, 32-bit counter rollover, valid acknowledgments, malformed
serial replies, and missing-reply timeouts. Physical square accuracy and motor
response still require a live test.
