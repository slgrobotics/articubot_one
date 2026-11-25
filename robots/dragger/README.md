# Dragger configuration

## Contents

- [Hardware settings](#hardware-settings)
- [Related settings](#related-settings)

----------------------------------------

## Hardware settings

`config/hardware.yaml` contains the serial connection, encoder calibration,
motor conversion factor, and battery settings for the Arduino driver.
The real-hardware branch of `description/ros2_control.xacro` loads this file.
Simulation does not load it.

The packaged file is used by default:

```bash
ros2 launch articubot_one dragger.launch.py
```

To use a machine-specific copy:

```bash
ros2 launch articubot_one dragger.launch.py \
  hardware_config:=/absolute/path/to/dragger_hardware.yaml
```

The same override is available when expanding the robot description directly:

```bash
xacro robots/dragger/description/robot.urdf.xacro sim_mode:=false \
  hardware_config:=/absolute/path/to/dragger_hardware.yaml
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

> The existing sonar no-return workaround remains in `description/ros2_control.xacro`
> and is disabled by default. This extraction does not enable it.


## Wi-Fi monitoring: V1 and V2

Dragger starts V2 Wi-Fi monitoring, visualization, overlay, browser viewer
(port 8088), and stationary active surveys 40 seconds after bringup, on real
hardware only. This mirrors Seggy's V2 setup, with GPS enabled for Dragger.
Use the matching V1 or V2 build of `wifi_logger_visualizer`; the selection does
not install or switch the package itself.

```bash
ros2 launch articubot_one dragger.launch.py wifi_version:=v2
ros2 launch articubot_one dragger.launch.py wifi_version:=v1
ros2 launch articubot_one dragger.launch.py wifi_version:=off
```

V1 uses `config/wifi_survey_config.yaml` and `wifi_data.db`. V2 uses
`config/wifi_survey_v2_config.yaml` and `wifi_data_v2.db` (paths relative to the
launch directory). Set `wifi_interface` to Dragger's connected radio if more
than one wireless adapter is present. GPS collection defaults to enabled on
`/gps/filtered`, remapped into the robot namespace. A fresh map-to-base_link TF
is still required. Missing/stale GPS does not stop passive V2 collection.

V2 defaults to `outdoors_loc_nav/assets/maps/empty_map.yaml`: 600 x 600 cells,
0.25 m resolution, centered at (-75, -75), covering 150 x 150 m. In map_server
or AMCL mode, an explicit navigation `map` raster is also used for Wi-Fi.
Alternatively, select the captured YAML/PGM background independently:

```bash
ros2 launch articubot_one dragger.launch.py \
  wifi_map_yaml_path:=/home/ros/launch/backyard.yaml wifi_active_survey:=false
```

When using SLAM Toolbox or Cartographer, pass a saved raster YAML/PGM through
`wifi_map_yaml_path`; a serialized SLAM posegraph is not a Wi-Fi background.
Without this override those modes use the empty map. V2 stores the background
snapshot; it does not follow a live SLAM map. Keep the GPS localization datum
and map orientation consistent between runs to reuse a coverage map reliably.

Active surveys require an iperf3 server at the configured `target_host` and
create network traffic; disable with `wifi_active_survey:=false` for passive
monitoring only. Namespaced multi-robot deployments on one host must configure
separate database paths and browser ports.
