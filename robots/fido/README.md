# Fido configuration

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
ros2 launch articubot_one fido.launch.py
```

To use a machine-specific copy:

```bash
ros2 launch articubot_one fido.launch.py \
  hardware_config:=/absolute/path/to/fido_hardware.yaml
```

The same override is available when expanding the robot description directly:

```bash
xacro robots/fido/description/robot.urdf.xacro sim_mode:=false \
  hardware_config:=/absolute/path/to/fido_hardware.yaml
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
