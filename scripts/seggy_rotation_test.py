#!/usr/bin/env python3
"""Command Seggy to spin in place and record AMCL pose as CSV."""

import argparse
import csv
import math
import time
from datetime import datetime


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turns', type=int, default=10)
    parser.add_argument('--speed', type=float, default=0.2,
                        help='Signed angular velocity in rad/s (default 0.2)')
    parser.add_argument('--cmd-topic', default='/cmd_vel_raw')
    parser.add_argument('--odom-topic', default='/diff_cont/odom')
    parser.add_argument('--amcl-topic', default='/amcl_pose')
    parser.add_argument('--output', default=datetime.now().strftime(
        'seggy_rotation_%Y%m%d_%H%M%S.csv'))
    args = parser.parse_args()
    if args.turns < 1 or not 0 < abs(args.speed) <= 0.3:
        parser.error('Use at least one turn and a speed magnitude up to 0.3 rad/s')

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    previous_yaw = None
    odom_time = amcl_time = start = None
    rotation = angular_speed = 0.0
    origin = None

    def on_odom(msg):
        nonlocal previous_yaw, odom_time, rotation, angular_speed
        heading = yaw(msg.pose.pose)
        if start is not None and previous_yaw is not None:
            rotation += math.remainder(heading - previous_yaw, math.tau)
        previous_yaw = heading
        angular_speed = msg.twist.twist.angular.z
        odom_time = time.monotonic()

    def on_amcl(msg):
        nonlocal origin, amcl_time
        amcl_time = time.monotonic()
        pose = msg.pose.pose
        x, y = pose.position.x, pose.position.y
        if origin is None or start is None:
            origin = x, y
        if start is not None:
            stamp = msg.header.stamp
            writer.writerow([
                amcl_time - start, stamp.sec * 10**9 + stamp.nanosec,
                msg.header.frame_id, x, y, x - origin[0], y - origin[1],
                yaw(pose), rotation / math.tau,
            ])
            log.flush()

    with open(args.output, 'x', newline='') as log:
        writer = csv.writer(log)
        writer.writerow(['elapsed_s', 'stamp_ns', 'frame_id', 'x_m', 'y_m',
                         'dx_m', 'dy_m', 'amcl_yaw_rad', 'odom_turns'])
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('seggy_rotation_test')
        pub = node.create_publisher(TwistStamped, args.cmd_topic, 10)
        node.create_subscription(Odometry, args.odom_topic, on_odom, qos_profile_sensor_data)
        # AMCL publishes on motion and retains its last pose for late subscribers.
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PoseWithCovarianceStamped, args.amcl_topic, on_amcl, amcl_qos)

        def command(speed):
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.angular.z = speed
            pub.publish(msg)

        try:
            print(f'Waiting for AMCL, wheel odometry, and {args.cmd_topic}. CSV: {args.output}',
                  flush=True)
            deadline = time.monotonic() + 15
            while True:
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                missing = []
                if odom_time is None or now - odom_time >= 0.5:
                    missing.append(f'{args.odom_topic}: no recent odometry')
                if amcl_time is None:
                    missing.append(f'{args.amcl_topic}: no AMCL pose; check the initial pose in RViz')
                if not pub.get_subscription_count():
                    missing.append(f'{args.cmd_topic}: no compatible velocity subscriber')
                if not missing:
                    break
                if now >= deadline:
                    raise RuntimeError('Startup timed out. ' + '; '.join(missing))

            start = time.monotonic()
            target = args.turns * math.tau
            deadline = start + 1.5 * target / abs(args.speed) + 5
            direction = math.copysign(1, args.speed)
            next_command = start
            next_report = start + 10
            print(f'Rotating for {args.turns} turns at {args.speed:g} rad/s.', flush=True)
            while direction * rotation < target:
                now = time.monotonic()
                if now - odom_time > 0.5:
                    raise RuntimeError(f'Odometry updates stopped on {args.odom_topic}')
                if now - max(start, amcl_time) > 10:
                    raise RuntimeError(f'AMCL updates stopped on {args.amcl_topic}')
                if now >= deadline:
                    raise RuntimeError('Timed out before reaching the requested rotation')
                if now >= next_report:
                    print(f'{direction * rotation / math.tau:.2f}/{args.turns} turns '
                          f'after {now - start:.0f} s', flush=True)
                    next_report = now + 10
                if now >= next_command:
                    command(args.speed)
                    next_command = now + 0.05
                rclpy.spin_once(node, timeout_sec=0.025)
            print(f'Reached {rotation / math.tau:.3f} signed odometry turns. Sending stop.',
                  flush=True)
        except KeyboardInterrupt:
            return 130
        except RuntimeError as exc:
            parser.exit(1, f'{exc}\n')
        finally:
            try:
                if start is not None:
                    stop_started = time.monotonic()
                    stationary_since = None
                    while time.monotonic() - stop_started < 5:
                        command(0.0)
                        rclpy.spin_once(node, timeout_sec=0.05)
                        time.sleep(0.05)
                        now = time.monotonic()
                        if (odom_time > stop_started and now - odom_time < 0.5
                                and abs(angular_speed) < 0.02):
                            stationary_since = stationary_since or now
                            if now - stationary_since >= 1:
                                print(f'Stop confirmed by wheel odometry at '
                                      f'{rotation / math.tau:.3f} signed turns.', flush=True)
                                break
                        else:
                            stationary_since = None
                    else:
                        parser.exit(1, 'Stop could not be confirmed by wheel odometry. '
                                    'If Seggy is still moving, use its emergency stop and '
                                    'check for another active velocity source.\n')
            finally:
                node.destroy_node()
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
