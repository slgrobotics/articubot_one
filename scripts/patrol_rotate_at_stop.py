#!/usr/bin/env python3

r"""
Run a configured in-place rotation for one Seggy patrol stop.

See https://github.com/slgrobotics/turtlebot3-autonomous-patrol/blob/lyrical/README.md#run-custom-scripts-at-patrol-stops

The script reads a task at the named stop from the YAML, then commands its signed
angle at the requested angular rate. Positive angles turn left. It tracks
multiple revolutions with wheel odometry and sends velocity commands through
Seggy's velocity smoother and collision monitor.

    ros2 run articubot_one patrol_rotate_at_stop.py \
        "$HOME/robot_ws/src/articubot_one/robots/seggy/config/patrol_four_rooms_sim.tasks.yaml" \
        south_yard 1

The optional third argument selects a task in a stop's list, starting at 1.
Omit it to run the first entry or a legacy single-task mapping.

Run with the ROS workspace sourced, Seggy stationary, and patrol idle.
The Four Rooms profile requires a running simulation clock. Wheel odometry,
the velocity smoother, and the collision monitor must be available.

Expect startup output listing the CLI arguments and effective YAML settings,
local start and end times, and progress with elapsed time, ETA, and percentage.
Start, progress, and finish events are emitted as JSON lines on stdout,
with the human-readable message in each event's log field.
The finish event includes the exit code and, on failure, the error details.
The script exits successfully after reaching
the target angle and confirming that the robot has stopped. A timeout or
missing feedback produces an error and a nonzero exit status.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time

import yaml
import rclpy
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


class StopUnconfirmed(RuntimeError):
    """Wheel odometry did not confirm that the robot stopped."""


def local_timestamp():
    now = datetime.now().astimezone()
    return f'{now:%Y-%m-%d} {now.hour % 12 or 12}:{now:%M:%S %p %Z}'


def format_duration(seconds):
    minutes, seconds = divmod(int(seconds), 60)
    hours, minutes = divmod(minutes, 60)
    return f'{hours:02d}:{minutes:02d}:{seconds:02d}'


def emit_event(event, stop, name, log, **fields):
    record = {
        'event': event,
        'version': 1,
        'stop': stop,
        'name': name,
        'log': log,
        **fields,
    }
    print(json.dumps(record, separators=(',', ':'), allow_nan=False), flush=True)


def emit_progress(stop, name, log, elapsed_s, eta_s, percent_complete):
    emit_event('patrol_task_progress', stop, name, log, elapsed_s=round(elapsed_s, 3),
               eta_s=eta_s, percent_complete=round(percent_complete, 1))


def load_task(path, stop, task_index=1):
    try:
        task = yaml.safe_load(path.read_text())['stops'][stop]
        if isinstance(task, list):
            if not 1 <= task_index <= len(task):
                raise ValueError(f'task_index must be between 1 and {len(task)}')
            task = task[task_index - 1]
        elif task_index != 1:
            raise ValueError('a single-task mapping only supports task_index 1')
        name = task.get('name', stop)
        angle, rate, timeout = (float(task[key]) for key in
                                ('angle_deg', 'angular_speed_rad_s', 'timeout_s'))
        use_sim_time = task.get('use_sim_time', True)
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as error:
        raise ValueError(f'{path}, stop {stop!r}: {error}') from error
    if not math.isfinite(angle) or angle == 0:
        raise ValueError('angle_deg must be finite and nonzero')
    if not 0 < rate <= 0.5:
        raise ValueError('angular_speed_rad_s must be in (0, 0.5]')
    if not math.isfinite(timeout) or timeout <= abs(math.radians(angle)) / rate:
        raise ValueError('timeout_s must exceed the nominal rotation duration')
    if not isinstance(use_sim_time, bool):
        raise ValueError('use_sim_time must be true or false')
    if not isinstance(name, str) or not name.strip():
        raise ValueError('name must be a nonempty string')
    return name, angle, rate, timeout, use_sim_time


def rotate(node, stop, task_name, angle, rate, timeout):
    target = abs(math.radians(angle))
    direction = math.copysign(1, angle)
    previous_yaw = None
    odom_time = None
    rotation = measured_speed = 0.0
    rotating = False

    def on_odom(msg):
        nonlocal previous_yaw, odom_time, rotation, measured_speed
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        speed = msg.twist.twist.angular.z
        if not math.isfinite(yaw) or not math.isfinite(speed):
            return
        if rotating and previous_yaw is not None:
            rotation += math.remainder(yaw - previous_yaw, math.tau)
        previous_yaw, measured_speed, odom_time = yaw, speed, time.monotonic()

    publisher = node.create_publisher(TwistStamped, 'cmd_vel_raw', 10)
    node.create_subscription(Odometry, 'diff_cont/odom', on_odom, qos_profile_sensor_data)

    def command(speed):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.angular.z = speed
        publisher.publish(message)

    # Use Seggy's velocity smoother and collision monitor before commanding motion.
    for name in ('velocity_smoother', 'collision_monitor'):
        client = node.create_client(GetState, f'{name}/get_state')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'{name} is unavailable')
        state = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, state, timeout_sec=5)
        if not state.done() or state.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
            raise RuntimeError(f'{name} is not active')

    deadline = time.monotonic() + 10
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if (odom_time is not None and now - odom_time < 0.5 and abs(measured_speed) <= 0.02
                and publisher.get_subscription_count() and node.get_clock().now().nanoseconds > 0):
            break
        if now >= deadline:
            raise RuntimeError('Waiting for stationary wheel odometry, command subscriber, or clock')

    label = f'{stop}: {task_name}'
    emit_progress(stop, task_name, f'{label} — rotating {angle:g}° at {rate:g} rad/s',
                  0.0, math.ceil(target / rate), 0.0)
    rotating = True
    rotation_start = now
    next_command = now
    next_progress = now + 2
    deadline = now + timeout
    try:
        while direction * rotation < target:
            now = time.monotonic()
            if now - odom_time >= 0.5:
                raise RuntimeError('Wheel odometry stopped updating')
            if now >= deadline:
                raise RuntimeError(f'Rotation timed out at {math.degrees(rotation):.1f}°')
            if now >= next_progress:
                elapsed = now - rotation_start
                progress = max(0.0, direction * rotation)
                estimated_rate = progress / elapsed if progress > 0 else rate
                eta = math.ceil((target - progress) / estimated_rate)
                percent = 100 * progress / target
                log = (f'{label} — rotated {math.degrees(rotation):.1f}° of {angle:g}° | '
                       f'elapsed={format_duration(elapsed)} | ETA={format_duration(eta)} | '
                       f'complete={percent:.1f}%')
                emit_progress(stop, task_name, log, elapsed, eta, percent)
                next_progress = now + 2
            if now >= next_command:
                command(direction * rate)
                next_command = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        stop_started = time.monotonic()
        while time.monotonic() - stop_started < 3:
            command(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            if odom_time > stop_started and abs(measured_speed) <= 0.02:
                break
        else:
            raise StopUnconfirmed('Could not confirm that rotation stopped')
    elapsed = time.monotonic() - rotation_start
    log = (f'{label} — completed rotation ({math.degrees(rotation):.1f}°) | '
           f'elapsed={format_duration(elapsed)} | ETA=00:00:00 | complete=100.0%')
    emit_progress(stop, task_name, log, elapsed, 0, 100.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('tasks_file', type=Path)
    parser.add_argument('stop', help='Stop name under stops in the YAML')
    parser.add_argument('task_index', nargs='?', type=int, default=1,
                        help='One-based task position at the stop (default: 1)')
    args = parser.parse_args()
    cli_log = (f'CLI arguments: tasks_file={str(args.tasks_file)!r}, stop={args.stop!r}, '
               f'task_index={args.task_index}')
    try:
        name, angle, rate, timeout, use_sim_time = load_task(args.tasks_file, args.stop, args.task_index)
    except ValueError as error:
        emit_event('patrol_task_finish', args.stop, None, f'{cli_log}\n{args.stop}: {error}',
                   elapsed_s=0.0, exit_code=2,
                   error_code='invalid_configuration', error_message=str(error))
        return 2
    settings_log = (f'YAML settings for {args.stop!r}: name={name!r}, angle_deg={angle:g}, '
                    f'angular_speed_rad_s={rate:g}, timeout_s={timeout:g}, '
                    f'use_sim_time={str(use_sim_time).lower()}')

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    started = time.monotonic()
    start_log = f'{cli_log}\n{settings_log}\n{local_timestamp()} {args.stop}: started {name!r}'
    emit_event('patrol_task_start', args.stop, name, start_log, elapsed_s=0.0)
    node = None
    exit_code, error_code, error_message = 0, None, None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(
            'patrol_rotate_at_stop', parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        rotate(node, args.stop, name, angle, rate, timeout)
    except KeyboardInterrupt:
        exit_code, error_code, error_message = 130, 'canceled', 'rotation canceled'
    except StopUnconfirmed as error:
        exit_code, error_code, error_message = 3, 'motion_stop_unconfirmed', str(error)
    except Exception as error:
        exit_code, error_code, error_message = 1, 'execution_failed', str(error)
    finally:
        try:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
        except Exception as error:
            if exit_code == 0:
                exit_code, error_code, error_message = 1, 'cleanup_failed', str(error)
        elapsed = time.monotonic() - started
        end_log = (f'{local_timestamp()} {args.stop}: ended {name!r} | '
                   f'elapsed={format_duration(elapsed)}')
        if error_message is not None:
            end_log = f'{args.stop}: {error_message}\n{end_log}'
        emit_event('patrol_task_finish', args.stop, name, end_log, elapsed_s=round(elapsed, 3),
                   exit_code=exit_code, error_code=error_code, error_message=error_message)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
