# Map orientation integration

`articubot_one` uses the standalone
[map_orientation_saver](https://github.com/slgrobotics/map_orientation_saver)
package. The estimator, frame math, default configuration, saver CLI, sidecar
format, and algorithm tests belong there. This project retains only robot
launch integration and its tests.

## Build

Place `map_orientation_saver` alongside this repository in the workspace.
The dependency is declared in `package.xml`:

```bash
cd ~/robot_ws
colcon build --packages-up-to articubot_one
source install/setup.bash
```

When replacing an older installation, use a clean build/install for the two
packages if obsolete `articubot_one` executables or copied Python modules remain.
A normal incremental colcon install does not prune previously installed files.
The old `ros2 run articubot_one map_orientation_node` and
`ros2 run articubot_one map_saver_with_orientation` entry points are no longer
provided by the source package.

## Bringup and robot settings

Generic localizers and Dragger/Seggy launch files retain the existing options:

```bash
ros2 launch articubot_one dragger.launch.py use_map_orientation_estimator:=true
ros2 launch articubot_one seggy.launch.py use_sim_time:=true
ros2 launch articubot_one seggy.launch.py use_map_orientation_estimator:=false
```

`launch/map_orientation.launch.py` is a thin project adapter. It starts
`map_orientation_saver`'s `map_orientation_node`, using the defaults installed
by that package, then applies:

1. On real hardware, the selected robot's `config/navsat_transform.yaml`, if
   present. `magnetic_declination_radians` is converted to the estimator's
   `magnetic_declination_degrees`; `yaw_offset` stays in radians.
2. In Gazebo simulation, true-ENU heading with zero declination/yaw correction,
   matching this project's `description/imu.xacro`. Hardware corrections are
   not used in simulation.
3. The caller's `orientation_config_filepath`, when provided, overriding the
   preceding settings. `use_sim_time` follows the launch argument.

Dragger therefore keeps its configured NavSat heading correction. Seggy's
hardware has no NavSat config: provide its local magnetic declination or a
verified true-ENU reference through an estimator YAML. Simulation needs no
extra heading override because the adapter knows this project's IMU convention.
The standalone package itself does not infer heading from the clock setting.

```bash
ros2 launch articubot_one seggy.launch.py \
  orientation_config_filepath:=/absolute/path/to/orientation.yaml
```

```yaml
/**/map_orientation_node:
  ros__parameters:
    heading_reference: magnetic
    magnetic_declination_degrees: 4.18  # Example: use your location's correction.
```

`use_map_orientation_estimator` defaults to true. With it false the adapter
starts no estimator and reads no estimator config files. The adapter can also
be launched directly with `robot_model`, `namespace`, and `use_sim_time`.
Existing namespacing and relative status/IMU topics are preserved. Do not start
another estimator in the same namespace while robot bringup is running one.

## Save a map

Use the executable from the standalone package:

```bash
ros2 run map_orientation_saver map_saver_with_orientation \
  -f ~/launch/maps/backyard \
  --ros-args \
  --params-file "$(ros2 pkg prefix --share map_orientation_saver)/config/map_saver_params.yaml"
```

For simulation, add `-p use_sim_time:=true` after `--ros-args`. For namespaced
robots, add `-r __ns:=/robot1` as appropriate. Existing project Nav2 saver
parameter files can also be supplied.

The command saves the raster YAML/image and matching `.orientation.yaml`.
Estimator status remains on `localization/map_orientation`; the JSON and
sidecar formats are unchanged by this migration. Read the standalone package's
README for input quality, frame conventions, parameter details, and failure
behavior. Geographic visualization is a separate consumer feature.

## Tests

`test/test_map_orientation_launch.py` covers this adapter, Dragger hardware
correction, Seggy's default, simulation and custom-config precedence,
namespace handling, disable behavior, and the existing robot launch arguments.
Estimator/saver/math tests now run only in `map_orientation_saver`.
