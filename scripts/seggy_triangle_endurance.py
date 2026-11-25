#!/usr/bin/env python3
"""Run p1 -> p2 -> p3 repeatedly using Nav2 NavigateToPose.

With Seggy parked at p3 and localized in map, source the ROS workspace and run:
    python3 src/articubot_one/scripts/seggy_triangle_endurance.py --cycles 10

The line-buffered JSONL log contains leg events, Nav2 paths and action feedback,
map-frame trajectory samples, odometry, and velocity commands. Pose samples
include distance to the latest global path received during that leg, when
available. Angular velocity in odometry and the command streams can be used
to compare actual and commanded turns, including angular acceleration.
"""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time


P1 = (-0.164, 0.011)
P2 = (-0.988, 2.321)


class RunStopped(Exception):
    """End the test before dispatching another leg."""


def distance_to_path(x, y, points):
    """Return the shortest distance to the path polyline in meters."""
    first_x, first_y = points[0]
    closest = math.hypot(x - first_x, y - first_y)
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        if length_squared:
            fraction = ((x - ax) * dx + (y - ay) * dy) / length_squared
            fraction = max(0.0, min(1.0, fraction))
            closest = min(closest, math.hypot(x - ax - fraction * dx,
                                              y - ay - fraction * dy))
    return closest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles', type=int, default=10)
    parser.add_argument('--startup-timeout', type=float, default=30.0,
                        help='Seconds to wait for Nav2 and map localization')
    parser.add_argument('--leg-timeout', type=float, default=180.0,
                        help='Maximum seconds per NavigateToPose goal')
    parser.add_argument('--output', type=Path, default=Path(datetime.now().strftime(
        'seggy_triangle_%Y%m%d_%H%M%S.jsonl')))
    args = parser.parse_args()
    if args.cycles < 1 or any(not math.isfinite(value) or value <= 0 for value in
                              (args.startup_timeout, args.leg_timeout)):
        parser.error('Use at least one cycle and finite positive timeouts')

    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import TrackingFeedback
    from nav_msgs.msg import Odometry, Path as NavPath, Trajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from tf2_ros import Buffer, TransformException, TransformListener

    with args.output.open('x', buffering=1) as log:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('seggy_triangle_endurance')
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        transforms = Buffer()
        listener = TransformListener(transforms, node)
        started = time.monotonic_ns()
        cycle, waypoint = 0, None
        stop_signal = None
        latest_path = None
        path_id = 0
        counts = Counter()
        pending = handle = result_future = None
        completed_legs = 0

        def request_stop(signum, _frame):
            nonlocal stop_signal
            stop_signal = signum

        previous_handlers = {
            sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
        }

        def record(event, **data):
            log.write(json.dumps(dict(
                elapsed_ns=time.monotonic_ns() - started,
                ros_time_ns=node.get_clock().now().nanoseconds,
                cycle=cycle, waypoint=waypoint, event=event, **data,
            ), separators=(',', ':')) + '\n')

        def on_message(msg, topic):
            nonlocal latest_path, path_id
            counts[topic] += 1
            data = {}
            if topic == '/plan':
                path_id += 1
                data['path_id'] = path_id
                latest_path = None
                if waypoint and msg.header.frame_id == 'map' and msg.poses:
                    latest_path = (path_id, [(p.pose.position.x, p.pose.position.y)
                                             for p in msg.poses])
            record('topic', topic=topic, message=message_to_ordereddict(msg), **data)

        topics = {
            '/plan': NavPath,
            '/local_plan': NavPath,
            '/transformed_global_plan': NavPath,
            '/tracking_feedback': TrackingFeedback,
            '/controller_server/optimal_trajectory': Trajectory,
            '/amcl_pose': PoseWithCovarianceStamped,
            '/odometry/local': Odometry,
            '/diff_cont/odom': Odometry,
            '/cmd_vel_raw': TwistStamped,
            '/cmd_vel_smoothed': TwistStamped,
            '/cmd_vel_nav': TwistStamped,
            '/diff_cont/cmd_vel': TwistStamped,
        }
        for topic, msg_type in topics.items():
            node.create_subscription(
                msg_type, topic, lambda msg, name=topic: on_message(msg, name),
                qos_profile_sensor_data,
            )

        def map_pose():
            try:
                return transforms.lookup_transform('map', 'base_link', Time())
            except TransformException:
                return None

        def record_map_pose():
            pose = map_pose()
            if pose is None:
                counts['map_pose_missing'] += 1
                return
            counts['map_pose'] += 1
            path_data = {}
            if latest_path is not None:
                reference, points = latest_path
                position = pose.transform.translation
                path_data = dict(path_id=reference, path_error_m=distance_to_path(
                    position.x, position.y, points))
            record('map_pose', transform=message_to_ordereddict(pose), **path_data)

        pose_timer = node.create_timer(0.05, record_map_pose)

        def check_stop():
            if stop_signal is not None:
                raise RunStopped(f'Received {signal.Signals(stop_signal).name}')

        def wait_for(future, deadline, timeout_message, *, stopping=False):
            while not future.done():
                if not stopping:
                    check_stop()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RunStopped(timeout_message)
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
            return future.result()

        def feedback(msg):
            record('feedback', message=message_to_ordereddict(msg.feedback))

        print(f'Logging to {args.output.resolve()}', flush=True)
        outcome = 'failed'
        try:
            record('run_start', requested_cycles=args.cycles, topics=list(topics),
                   pose_sample_hz=20, action='/navigate_to_pose',
                   leg_timeout_s=args.leg_timeout)
            deadline = time.monotonic() + args.startup_timeout
            while True:
                check_stop()
                origin = map_pose()
                if origin is not None and client.server_is_ready():
                    break
                if time.monotonic() >= deadline:
                    raise RunStopped('Timed out waiting for /navigate_to_pose and map -> base_link')
                rclpy.spin_once(node, timeout_sec=0.1)

            position = origin.transform.translation
            p3 = (position.x, position.y)
            record('course', p1=P1, p2=P2, p3=p3, p3_transform=message_to_ordereddict(origin))
            print(f'Captured p3 at ({p3[0]:.3f}, {p3[1]:.3f}) in map', flush=True)
            for cycle in range(1, args.cycles + 1):
                for name, start, target in (('p1', p3, P1), ('p2', P1, P2), ('p3', P2, p3)):
                    check_stop()
                    waypoint = name
                    latest_path = None
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id = 'map'
                    goal.pose.header.stamp = node.get_clock().now().to_msg()
                    goal.pose.pose.position.x, goal.pose.pose.position.y = target
                    yaw = math.atan2(target[1] - start[1], target[0] - start[0])
                    goal.pose.pose.orientation.z = math.sin(yaw / 2)
                    goal.pose.pose.orientation.w = math.cos(yaw / 2)
                    print(f'Cycle {cycle}/{args.cycles}, goal {waypoint}', flush=True)
                    record('goal_sent', target=message_to_ordereddict(goal.pose), yaw_rad=yaw)
                    deadline = time.monotonic() + args.leg_timeout
                    pending = client.send_goal_async(goal, feedback_callback=feedback)
                    handle = wait_for(pending, deadline, 'Timed out waiting for goal acceptance')
                    pending = None
                    if not handle.accepted:
                        raise RunStopped(f'Nav2 rejected {waypoint}')
                    record('goal_accepted', goal_id=bytes(handle.goal_id.uuid).hex())
                    result_future = handle.get_result_async()
                    result = wait_for(result_future, deadline, 'Timed out navigating to goal')
                    record('goal_result', status=result.status,
                           result=message_to_ordereddict(result.result))
                    handle = result_future = None
                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                        raise RunStopped(f'Nav2 finished {waypoint} with status {result.status}')
                    completed_legs += 1
            outcome = 'succeeded'
        except (RunStopped, RuntimeError) as exc:
            record('run_error', detail=str(exc))
            print(f'Stopped: {exc}', flush=True)
        finally:
            try:
                if pending is not None or (handle is not None and handle.accepted):
                    deadline = time.monotonic() + 5
                    if pending is not None:
                        handle = wait_for(pending, deadline, 'Goal acceptance unconfirmed',
                                          stopping=True)
                    if handle.accepted:
                        if result_future is None:
                            result_future = handle.get_result_async()
                        if not result_future.done():
                            response = wait_for(handle.cancel_goal_async(), deadline,
                                                'Cancel response unconfirmed', stopping=True)
                            record('cancel_response', return_code=response.return_code)
                        result = wait_for(result_future, deadline, 'Goal termination unconfirmed',
                                          stopping=True)
                        record('goal_result', status=result.status,
                               result=message_to_ordereddict(result.result))
            except (RunStopped, RuntimeError) as exc:
                record('cleanup_error', detail=str(exc))
                print(f'Nav2 goal state unconfirmed: {exc}', flush=True)
            finally:
                record('run_end', outcome=outcome, completed_legs=completed_legs,
                       samples={name: counts[name] for name in
                                (*topics, 'map_pose', 'map_pose_missing')})
                print(f'{completed_legs}/{3 * args.cycles} legs completed', flush=True)
                pose_timer.cancel()
                listener.unregister()
                client.destroy()
                node.destroy_node()
                rclpy.shutdown()
                for sig, handler in previous_handlers.items():
                    signal.signal(sig, handler)
        return 0 if outcome == 'succeeded' else 1


if __name__ == '__main__':
    raise SystemExit(main())
