"""

See https://github.com/slgrobotics/articubot_one/wiki/Jazzy-to-Lyrical-migration#battery-monitoring

Compatibility node for legacy battery topic consumers.

This node:
 - listens to the official battery array topic published by the Lyrical battery broadcaster,
 - selects the configured battery by its frame_id, 
 - converts the report from the broadcaster's 0..100 percentage convention to the older sensor_msgs/BatteryState 0..1 range,
 - and republishes it on the legacy battery/battery_state topic used by the rest of the robot stack.

This keeps older nodes and launch files working while the system uses the newer official battery monitor output.

See scripts/run_battery_state_compat.py

Run it:
  ros2 run articubot_one run_battery_state_compat.py

"""

from copy import deepcopy
import math

import rclpy
from control_msgs.msg import BatteryStateArray
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


def normalize_battery_state(message, frame_id):
    """Convert the official 0..100 percentage to sensor_msgs' 0..1 fraction."""
    result = deepcopy(message)
    result.header.frame_id = frame_id
    result.percentage = (
        min(1.0, max(0.0, result.percentage / 100.0))
        if math.isfinite(result.percentage) else math.nan
    )
    return result


def select_battery(message, battery_name, frame_id):
    """Use the per-battery state, including its actual presence flag."""
    matches = [state for state in message.battery_states
               if state.header.frame_id == battery_name]
    if len(matches) != 1:
        return None
    return normalize_battery_state(matches[0], frame_id)


class BatteryStateCompat(Node):
    def __init__(self):
        super().__init__('battery_state_compat')

        self.battery_name = self.declare_parameter('battery_name', 'main_battery').value
        self.frame_id = self.declare_parameter('frame_id', 'battery_frame').value

        self.get_logger().info("BatteryStateCompat node started with parameters: "
                                f"battery_name={self.battery_name}, frame_id={self.frame_id}")

        self.publisher = self.create_publisher(BatteryState, 'battery/battery_state', 10)

        self.subscription = self.create_subscription(
            BatteryStateArray, 'battery/raw_battery_states', self.on_battery_states, 10,
        )

    def on_battery_states(self, message):
        state = select_battery(message, self.battery_name, self.frame_id)
        if state is None:
            self.get_logger().warning(
                f'Expected one raw state for battery {self.battery_name!r}',
                throttle_duration_sec=5.0,
            )
            return
        self.publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateCompat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
