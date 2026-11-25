# Turtle configuration

## Contents

- [Hardware settings](#hardware-settings)
- [Related settings](#related-settings)

----------------------------------------

## Hardware settings

`config/hardware.yaml` contains the Create 1 driver's serial connection,
frame names, command timeout, loop frequency, and calibration settings.
`turtle.drive.launch.py` passes this file directly to `create_driver` as ROS
node parameters. The `/**` selector supports robot namespaces.

The packaged file is used by default:

```bash
ros2 launch articubot_one turtle.launch.py
```

To use a machine-specific copy:

```bash
ros2 launch articubot_one turtle.launch.py \
  hardware_config:=/absolute/path/to/turtle_hardware.yaml
```

The drive-only launch accepts the same argument:

```bash
ros2 launch articubot_one turtle.drive.launch.py robot_model:=turtle \
  hardware_config:=/absolute/path/to/turtle_hardware.yaml
```

> Rebuild and source the workspace after adding the packaged YAML. Restart
> the driver after changing its startup settings.

## Related settings

- Real Turtle uses `create_driver`; its hardware settings do not belong in Xacro.
- Simulation uses `config/controllers.yaml` and does not load the hardware YAML.
- Command/odometry remappings and driver respawn settings remain in the drive launch.
- Sensor settings remain in `turtle.sensors.launch.py`.
