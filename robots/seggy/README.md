# Seggy configuration

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
