#!/usr/bin/env python3
"""Compare a reverse-heading NavigateToPose goal with DriveOnHeading then Spin.

Source the ROS workspace before running. By default the test sends a goal 1 m
ahead with a reversed final heading. Use --split for two separate actions.
Use --start-from-log to stage at a prior run's map pose before repeating it.
Each action has a named phase in the JSONL telemetry and a bounded timeout.
"""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time


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
    parser.add_argument('--split', action='store_true', help='Use separate drive and spin actions')
    parser.add_argument('--start-from-log', type=Path,
                        help='Navigate to the map start pose recorded in a prior run')
    parser.add_argument('--startup-timeout', type=float, default=30.0,
                        help='Seconds to wait for Nav2 and map localization')
    parser.add_argument('--leg-timeout', type=float, default=75.0,
                        help='Maximum seconds per action')
    parser.add_argument('--output', type=Path, default=Path(datetime.now().strftime(
        'seggy_nav2_turn_%Y%m%d_%H%M%S.jsonl')))
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0 for value in
                              (args.startup_timeout, args.leg_timeout)):
        parser.error('Use finite positive timeouts')

    reference = None
    if args.start_from_log:
        with args.start_from_log.open() as prior:
            reference = next((row for line in prior
                              if (row := json.loads(line)).get('event') == 'course'), None)
        if not reference or 'origin' not in reference:
            parser.error('The previous log has no recorded start pose')

    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
    from nav2_msgs.msg import TrackingFeedback
    from nav_msgs.msg import Odometry, Path as NavPath, Trajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import Imu, JointState
    from rclpy.parameter_client import AsyncParameterClient
    from tf2_ros import Buffer, TransformException, TransformListener

    with args.output.open('x', buffering=1) as log:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('seggy_nav2_turn_probe')
        clients = {'navigate': ActionClient(node, NavigateToPose, '/navigate_to_pose')}
        if args.split:
            clients['drive'] = ActionClient(node, DriveOnHeading, '/drive_on_heading')
            clients['spin'] = ActionClient(node, Spin, '/spin')
        transforms = Buffer()
        listener = TransformListener(transforms, node)
        started = time.monotonic_ns()
        waypoint = None
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
                phase=waypoint, event=event, **data,
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
            '/joint_states': JointState,
            '/imu/data': Imu,
            '/diff_cont/cmd_vel_out': TwistStamped,
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

        def planar(pose):
            position = pose.transform.translation
            q = pose.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            return position.x, position.y, yaw

        def require_map_pose():
            pose = map_pose()
            if pose is None:
                raise RunStopped('Map transform unavailable')
            return pose

        def navigation_goal(x, y, yaw):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            return goal

        def spin_goal(yaw):
            goal = Spin.Goal()
            goal.target_yaw = yaw
            goal.time_allowance.sec = math.ceil(args.leg_timeout)
            return goal

        def perform(phase, name, goal):
            nonlocal pending, handle, result_future, waypoint, latest_path, completed_legs
            check_stop()
            waypoint, latest_path = phase, None
            print(f'Sending Nav2 goal for {phase}', flush=True)
            record('goal_sent', action=name, goal=message_to_ordereddict(goal),
                   start_pose=message_to_ordereddict(require_map_pose()))
            deadline = time.monotonic() + args.leg_timeout
            pending = clients[name].send_goal_async(goal, feedback_callback=feedback)
            handle = wait_for(pending, deadline, 'Timed out waiting for goal acceptance')
            pending = None
            if not handle.accepted:
                raise RunStopped(f'Nav2 rejected {phase}')
            record('goal_accepted', goal_id=bytes(handle.goal_id.uuid).hex())
            result_future = handle.get_result_async()
            result = wait_for(result_future, deadline, 'Timed out navigating to goal')
            record('goal_result', status=result.status, result=message_to_ordereddict(result.result))
            handle = result_future = None
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RunStopped(f'Nav2 finished {phase} with status {result.status}'
                                 f' (code {result.result.error_code}: {result.result.error_msg})')
            completed_legs += 1
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                check_stop()
                rclpy.spin_once(node, timeout_sec=0.05)
            record('phase_end', end_pose=message_to_ordereddict(require_map_pose()))

        print(f'Logging to {args.output.resolve()}', flush=True)
        outcome = 'failed'
        try:
            record('run_start', mode='split' if args.split else 'combined',
                   reference=str(args.start_from_log) if reference is not None else None,
                   topics=list(topics), pose_sample_hz=20,
                   leg_timeout_s=args.leg_timeout)
            deadline = time.monotonic() + args.startup_timeout
            while True:
                check_stop()
                origin = map_pose()
                if origin is not None and all(c.server_is_ready() for c in clients.values()):
                    break
                if time.monotonic() >= deadline:
                    raise RunStopped('Timed out waiting for Nav2 actions and map -> base_link')
                rclpy.spin_once(node, timeout_sec=0.1)

            queries = {
                '/diff_cont': ['angular.z.' + name for name in (
                    'max_velocity', 'min_velocity', 'max_acceleration', 'max_deceleration',
                    'max_jerk', 'min_jerk')] + ['publish_limited_velocity'],
                '/controller_server': ['FollowPath.' + name for name in (
                    'wz_max', 'wz_std', 'PathAlignCritic.cost_weight',
                    'TwirlingCritic.twirling_cost_weight')],
                '/velocity_smoother': ['max_velocity', 'min_velocity', 'max_accel',
                                      'max_decel', 'smoothing_frequency'],
            }
            for remote, names in queries.items():
                parameters = AsyncParameterClient(node, remote)
                if not parameters.wait_for_services(timeout_sec=1.0):
                    record('params_unavailable', node_name=remote)
                    continue
                try:
                    response = wait_for(parameters.get_parameters(names), time.monotonic() + 1,
                                        'parameter read timeout')
                    record('parameters', node_name=remote, parameters={
                        key: repr(rclpy.parameter.parameter_value_to_python(value))
                        for key, value in zip(names, response.values)})
                except RunStopped:
                    check_stop()
                    record('params_unavailable', node_name=remote)
            if reference:
                start = reference['origin']['transform']['translation']
                yaw = reference['start_yaw']
                x, y, heading = planar(require_map_pose())
                position_error = math.hypot(x - start['x'], y - start['y'])
                heading_error = abs(math.remainder(heading - yaw, math.tau))
                if position_error > (.35 if args.split else .06) or heading_error > .15:
                    perform('staging', 'navigate', navigation_goal(start['x'], start['y'], yaw))
                x, y, heading = planar(require_map_pose())
                if math.hypot(x - start['x'], y - start['y']) > (.35 if args.split else .15):
                    raise RunStopped('Staging pose is too far from the recorded start pose')
                if args.split:
                    target_x, target_y = reference['target']
                    desired = math.atan2(target_y - y, target_x - x)
                    correction = math.remainder(desired - heading, math.tau)
                    if abs(correction) > .05:
                        perform('align_for_drive', 'spin', spin_goal(correction))
                elif abs(math.remainder(heading - yaw, math.tau)) > .15:
                    raise RunStopped('Staging heading differs from the recorded start heading')
            origin = require_map_pose()
            x, y, start_yaw = planar(origin)
            distance = 1.0
            target = (x + math.cos(start_yaw), y + math.sin(start_yaw))
            if args.split and reference:
                target = tuple(reference['target'])
                dx, dy = target[0] - x, target[1] - y
                distance = math.hypot(dx, dy)
                if not .65 <= distance <= 1.35 or abs(math.remainder(
                    math.atan2(dy, dx) - start_yaw, math.tau
                )) > .15:
                    raise RunStopped('Pose is unsuitable for a straight drive to the turn waypoint')
            record('course', origin=message_to_ordereddict(origin), target=target,
                   start_yaw=start_yaw)
            if args.split:
                goal = DriveOnHeading.Goal()
                goal.target.x = distance
                goal.speed = .10
                goal.time_allowance.sec = 30
                perform('drive', 'drive', goal)
                if reference:
                    x, y, _ = planar(require_map_pose())
                    if math.hypot(x - reference['target'][0], y - reference['target'][1]) > .25:
                        raise RunStopped('Drive ended too far from the reference turn location')
                perform('spin', 'spin', spin_goal(-math.pi))
            else:
                perform('combined', 'navigate',
                        navigation_goal(*target, math.remainder(start_yaw + math.pi, math.tau)))
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
                print(f'{completed_legs} actions completed', flush=True)
                pose_timer.cancel()
                listener.unregister()
                for client in clients.values():
                    client.destroy()
                node.destroy_node()
                rclpy.shutdown()
                for sig, handler in previous_handlers.items():
                    signal.signal(sig, handler)
        return 0 if outcome == 'succeeded' else 1


if __name__ == '__main__':
    raise SystemExit(main())
