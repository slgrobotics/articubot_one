#!/usr/bin/env python3
r"""
Wait for a configured duration at one patrol stop.

Set the task's executable to "articubot_one patrol_wait_at_stop.py" and give
it a duration_s in seconds. Run with the ROS workspace sourced:

    ros2 run articubot_one patrol_wait_at_stop.py /path/to/tasks.yaml south_yard 2

The optional third argument selects a one-based task position and defaults
to 1. The task uses elapsed real time and sends no motion commands. Expect
JSON start, progress, and finish records with human-readable log fields.
The script exits with 0 after the wait, 2 for invalid configuration, or
130 if canceled. Running it standalone requires no robot or simulator.
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


def timestamp():
    now = datetime.now().astimezone()
    return f'{now:%Y-%m-%d} {now.hour % 12 or 12}:{now:%M:%S %p %Z}'


def emit_event(event, stop, name, log, **fields):
    record = {'event': event, 'version': 1, 'stop': stop, 'name': name, 'log': log, **fields}
    print(json.dumps(record, separators=(',', ':'), allow_nan=False), flush=True)


def load_task(path, stop, index):
    task = yaml.safe_load(path.read_text())['stops'][stop]
    if isinstance(task, list):
        if not 1 <= index <= len(task):
            raise ValueError(f'task_index must be between 1 and {len(task)}')
        task = task[index - 1]
    elif index != 1:
        raise ValueError('a single-task mapping only supports task_index 1')
    name = task.get('name', stop)
    if not isinstance(name, str) or not name.strip():
        raise ValueError('name must be a nonempty string')
    duration = task['duration_s']
    if (type(duration) not in (int, float) or not math.isfinite(duration) or duration < 0):
        raise ValueError('duration_s must be a finite nonnegative number')
    return name, duration


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('tasks_file', type=Path)
    parser.add_argument('stop', help='Stop name under stops in the YAML')
    parser.add_argument('task_index', nargs='?', type=int, default=1,
                        help='One-based task position at the stop (default: 1)')
    args = parser.parse_args()
    cli_log = (f'CLI arguments: tasks_file={str(args.tasks_file)!r}, stop={args.stop!r}, '
               f'task_index={args.task_index}')
    try:
        name, duration = load_task(args.tasks_file, args.stop, args.task_index)
    except (AttributeError, KeyError, OSError, OverflowError, TypeError, ValueError, yaml.YAMLError) as error:
        message = f'{args.tasks_file}, stop {args.stop!r}: {error}'
        emit_event('patrol_task_finish', args.stop, None, f'{cli_log}\n{message}',
                   elapsed_s=0.0, exit_code=2, error_code='invalid_configuration',
                   error_message=message)
        return 2

    signal.signal(signal.SIGTERM, interrupt)
    label = f'{args.stop}: {name}'
    started = time.monotonic()
    exit_code, error_code, error_message = 0, None, None
    try:
        log = (f'{cli_log}\nYAML settings: name={name!r}, duration_s={duration:g}\n'
               f'{timestamp()} {label} — started waiting')
        emit_event('patrol_task_start', args.stop, name, log, elapsed_s=0.0)
        while True:
            elapsed = time.monotonic() - started
            remaining = max(0.0, duration - elapsed)
            percent = 100.0 * (1.0 - remaining / duration) if duration else 100.0
            log = (f'{label} — waited {elapsed:.1f} s of {duration:g} s | '
                   f'ETA={math.ceil(remaining)} s | complete={percent:.1f}%')
            emit_event('patrol_task_progress', args.stop, name, log,
                       elapsed_s=round(elapsed, 3), eta_s=math.ceil(remaining),
                       percent_complete=round(percent, 1))
            if remaining == 0.0:
                break
            time.sleep(min(2.0, remaining))
    except KeyboardInterrupt:
        exit_code, error_code, error_message = 130, 'canceled', 'wait canceled'

    elapsed = time.monotonic() - started
    result = error_message if error_message is not None else 'wait complete'
    log = f'{timestamp()} {label} — {result} | elapsed={elapsed:.1f} s'
    emit_event('patrol_task_finish', args.stop, name, log, elapsed_s=round(elapsed, 3),
               exit_code=exit_code, error_code=error_code, error_message=error_message)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
