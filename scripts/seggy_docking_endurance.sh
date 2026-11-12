#!/usr/bin/env bash
# Run repeated real Seggy docking cycles with a ROS action client that owns cancellation.
set -euo pipefail

exec python3 - "$@" <<'PY'
import argparse
import csv
import json
import math
from pathlib import Path
import random
import secrets
import signal
import tempfile
import time


def count(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return number


parser = argparse.ArgumentParser(description="Run Seggy real_v_bay docking endurance cycles.")
parser.add_argument("iterations", type=count)
parser.add_argument("--retries", type=count, default=2, help="extra attempts per cycle (default 2)")
parser.add_argument("--seed", type=int, help="replay the same pre-staging goals")
parser.add_argument("--log-dir", type=Path, default=Path.cwd(), help="parent directory for run logs")
parser.add_argument("--dry-run", action="store_true", help="print sampled goals without connecting to ROS")
parser.add_argument(
    "--allow-unconfirmed-charge", action="store_true",
    help="continue after insertion stopped without confirmed charging",
)
args = parser.parse_args()
if args.iterations == 0:
    parser.error("iterations must be positive")

seed = args.seed if args.seed is not None else secrets.randbits(64)
rng = random.Random(seed)
start = (0.07070043528482488, 0.07923359411452124)
end = (0.12089444929517279, 2.6351817500767054)
start_yaw = 2 * math.atan2(0.20215634834670823, 0.9793532615063496)
end_yaw = 2 * math.atan2(-0.3969423006361257, 0.9178435650837781)
yaw_span = math.remainder(end_yaw - start_yaw, 2 * math.pi)


def sample_goal():
    fraction = rng.random()
    yaw = start_yaw + fraction * yaw_span
    return dict(
        t=fraction, x=start[0] + fraction * (end[0] - start[0]),
        y=start[1] + fraction * (end[1] - start[1]), yaw_rad=yaw,
        qz=math.sin(yaw / 2), qw=math.cos(yaw / 2),
    )


print(f"Seed {seed}", flush=True)
if args.dry_run:
    for cycle in range(1, args.iterations + 1):
        print(json.dumps(dict(cycle=cycle, frame_id="map", **sample_goal())))
    raise SystemExit(0)

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import BatteryState


class RunStopped(Exception):
    """A condition that requires ending the run before sending another goal."""


stop_signal = None


def request_stop(signum, _frame):
    global stop_signal
    stop_signal = signum


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
args.log_dir.mkdir(parents=True, exist_ok=True)
directory = Path(tempfile.mkdtemp(prefix="seggy-docking-", dir=args.log_dir))
print(f"Logs: {directory}", flush=True)

rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node("seggy_docking_endurance")
clients = {
    "navigate": ActionClient(node, NavigateToPose, "/navigate_to_pose"),
    "dock": ActionClient(node, DockRobot, "/dock_robot"),
    "undock": ActionClient(node, UndockRobot, "/undock_robot"),
}
events = (directory / "events.jsonl").open("w", buffering=1)
summary = (directory / "attempts.csv").open("w", newline="", buffering=1)
csv_rows = csv.DictWriter(summary, fieldnames=[
    "cycle", "attempt", "t", "x", "y", "yaw_rad", "outcome", "phase",
    "status", "error_code", "elapsed_s", "nav_s", "dock_s", "undock_s",
    "charging_dwell_s", "noncharging_dwell_s", "unknown_dwell_s", "charge_confirmation_waived",
    "detail",
])
csv_rows.writeheader()
cycle = attempt = 0
odometry = battery = None


def record(event, **data):
    events.write(json.dumps(dict(
        wall_time=time.time(), cycle=cycle, attempt=attempt, event=event, **data,
    )) + "\n")


def on_odometry(msg):
    global odometry
    odometry = (time.monotonic(), msg.twist.twist)


def on_battery(msg):
    global battery
    battery = (time.monotonic(), msg)
    measurements = {}
    for name in ("voltage", "current", "percentage"):
        value = getattr(msg, name)
        measurements[name] = value if math.isfinite(value) else None
    record("battery", status=msg.power_supply_status, **measurements)


def on_log(msg):
    if ("LiDAR dock " in msg.msg or msg.msg.startswith("ALIGN:")) and "id=real_v_bay" in msg.msg:
        record("dock_trace", logger=msg.name, stamp=message_to_ordereddict(msg.stamp),
               message=msg.msg)
        if " arrival " in msg.msg or msg.msg.startswith("ALIGN: result"):
            print(msg.msg, flush=True)


subscriptions = [
    node.create_subscription(Odometry, "/odometry/local", on_odometry, qos_profile_sensor_data),
    node.create_subscription(BatteryState, "/battery/battery_state", on_battery, qos_profile_sensor_data),
    node.create_subscription(Log, "/rosout", on_log, 100),
]


def spin():
    rclpy.spin_once(node, timeout_sec=.05)


def check_stop():
    if stop_signal is not None:
        raise RunStopped("Interrupted by signal")


def wait_stationary():
    deadline = time.monotonic() + 8
    still_since = None
    while time.monotonic() < deadline:
        check_stop()
        spin()
        now = time.monotonic()
        if odometry is not None and now - odometry[0] < .5:
            twist = odometry[1]
            if math.hypot(twist.linear.x, twist.linear.y) <= .005 and abs(twist.angular.z) <= .01:
                still_since = still_since or now
                if now - still_since >= .5:
                    return
                continue
        still_since = None
    raise RunStopped("Fresh odometry did not confirm a stationary robot")


def action(phase, goal, timeout):
    check_stop()
    client = clients[phase]
    started = time.monotonic()
    deadline = started + timeout
    handle = result_future = None
    cancel_deadline = None
    last_feedback = 0.0
    last_state = None

    def feedback(message):
        nonlocal last_feedback, last_state
        now = time.monotonic()
        state = getattr(message.feedback, "state", None)
        if state != last_state or now - last_feedback >= 1:
            record("feedback", phase=phase, data=message_to_ordereddict(message.feedback))
            last_state, last_feedback = state, now

    print(f"Cycle {cycle}/{args.iterations}, attempt {attempt}: {phase}", flush=True)
    record("goal", phase=phase, data=message_to_ordereddict(goal))
    pending = client.send_goal_async(goal, feedback_callback=feedback)
    cancel_requested = False
    rejections = 0
    while True:
        spin()
        now = time.monotonic()
        if handle is None and pending.done():
            handle = pending.result()
            if not handle.accepted:
                rejections += 1
                record("goal_rejected", phase=phase, rejection=rejections)
                check_stop()
                if cancel_deadline is not None or rejections > args.retries:
                    raise RunStopped(f"{phase}: action goal rejected and retry budget exhausted")
                print(f"{phase}: action goal rejected. Retrying in 10 seconds "
                      f"({rejections}/{args.retries}).", flush=True)
                retry_at = time.monotonic() + 10
                while time.monotonic() < retry_at:
                    check_stop()
                    spin()
                check_stop()
                if hasattr(goal, "pose"):
                    goal.pose.header.stamp = node.get_clock().now().to_msg()
                record("goal", phase=phase, retry_after_rejection=rejections,
                       data=message_to_ordereddict(goal))
                pending = client.send_goal_async(goal, feedback_callback=feedback)
                deadline = time.monotonic() + timeout
                handle = None
                continue
            result_future = handle.get_result_async()
        if result_future is not None and result_future.done():
            response = result_future.result()
            result = response.result
            outcome = dict(
                phase=phase, status=response.status, error_code=result.error_code,
                detail=result.error_msg, elapsed_s=round(now - started, 3),
                success=(response.status == GoalStatus.STATUS_SUCCEEDED
                         and result.error_code == 0 and getattr(result, "success", True)),
                timed_out=cancel_deadline is not None and stop_signal is None,
            )
            record("result", **outcome, result=message_to_ordereddict(result))
            print(f"{phase}: status={outcome['status']} error={outcome['error_code']} "
                  f"elapsed={outcome['elapsed_s']:.1f}s {outcome['detail']}", flush=True)
            return outcome
        if cancel_deadline is None and (stop_signal is not None or now >= deadline):
            cancel_deadline = now + 15
            record("cancel_pending", phase=phase, reason="signal" if stop_signal else "deadline")
            print(f"{phase}: requesting cancellation and waiting for terminal result", flush=True)
        if cancel_deadline is not None:
            if handle is not None and not cancel_requested:
                handle.cancel_goal_async()
                cancel_requested = True
            if now >= cancel_deadline:
                raise RunStopped(f"{phase}: action termination could not be confirmed")


def dwell():
    start = previous = time.monotonic()
    counts = dict(charging=0., noncharging=0., unknown=0.)
    while previous < start + 60:
        check_stop()
        spin()
        now = min(time.monotonic(), start + 60)
        status = "unknown"
        if battery is not None and now - battery[0] <= 3:
            supply = battery[1].power_supply_status
            if supply != BatteryState.POWER_SUPPLY_STATUS_UNKNOWN:
                status = ("charging" if supply == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                          else "noncharging")
        counts[status] += now - previous
        previous = now
    return {f"{status}_dwell_s": round(seconds, 3) for status, seconds in counts.items()}


completed = first_attempt = 0
exit_code = 0
try:
    record("run_start", seed=seed, iterations=args.iterations, retries=args.retries,
           allow_unconfirmed_charge=args.allow_unconfirmed_charge)
    for phase, client in clients.items():
        deadline = time.monotonic() + 10
        while not client.wait_for_server(timeout_sec=.2):
            check_stop()
            if time.monotonic() >= deadline:
                raise RunStopped(f"{phase}: action server unavailable")
    for cycle in range(1, args.iterations + 1):
        pose = sample_goal()
        for attempt in range(1, args.retries + 2):
            row = dict(cycle=cycle, attempt=attempt, **{
                key: pose[key] for key in ("t", "x", "y", "yaw_rad")
            })
            try:
                wait_stationary()
                nav = NavigateToPose.Goal()
                nav.pose.header.frame_id = "map"
                nav.pose.header.stamp = node.get_clock().now().to_msg()
                nav.pose.pose.position.x, nav.pose.pose.position.y = pose["x"], pose["y"]
                nav.pose.pose.orientation.z, nav.pose.pose.orientation.w = pose["qz"], pose["qw"]
                result = action("navigate", nav, 120)
                row["nav_s"] = result["elapsed_s"]
                if result["success"] and not result["timed_out"] and stop_signal is None:
                    goal = DockRobot.Goal(
                        use_dock_id=True, dock_id="real_v_bay",
                        navigate_to_staging_pose=True, max_staging_time=90.,
                    )
                    result = action("dock", goal, 240)
                    row["dock_s"] = result["elapsed_s"]
                row.update({key: result[key] for key in (
                    "phase", "status", "error_code", "elapsed_s", "detail"
                )})
                check_stop()
                charge_confirmation_waived = (
                    args.allow_unconfirmed_charge
                    and result["phase"] == "dock"
                    and not result["timed_out"]
                    and result["status"] == GoalStatus.STATUS_ABORTED
                    and result["error_code"] == DockRobot.Result.FAILED_TO_CHARGE
                    and result["detail"] == "Charging not confirmed after stopping insertion"
                )
                row["charge_confirmation_waived"] = charge_confirmation_waived
                if charge_confirmation_waived:
                    wait_stationary()
                    record("charge_confirmation_waived", **result)
                    print("Insertion stopped. Charging confirmation waived for this cycle.",
                          flush=True)
                if (result["success"] and not result["timed_out"]) or charge_confirmation_waived:
                    print("Monitoring charging for 60 seconds.", flush=True)
                    row["phase"] = "dwell"
                    row.update(dwell())
                    row["phase"] = "undock"
                    result = action("undock", UndockRobot.Goal(
                        dock_type="lidar_dock", max_undocking_time=60.
                    ), 75)
                    row["undock_s"] = result["elapsed_s"]
                    row.update({key: result[key] for key in (
                        "status", "error_code", "elapsed_s", "detail"
                    )})
                    check_stop()
                    if not result["success"] or result["timed_out"]:
                        raise RunStopped("Undocking failed. Inspect Seggy before starting another cycle.")
                    wait_stationary()
                    row["outcome"] = "completed"
                    completed += 1
                    first_attempt += attempt == 1
                    break
                # These dock error codes are emitted before charging insertion.
                retryable = result["phase"] == "navigate" or (
                    not result["timed_out"] and result["status"] == GoalStatus.STATUS_ABORTED
                    and result["error_code"] in (
                        DockRobot.Result.FAILED_TO_STAGE, DockRobot.Result.FAILED_TO_DETECT_DOCK
                    )
                )
                if not retryable:
                    raise RunStopped("Docking stopped in an uncertain phase. Inspect Seggy at the bay.")
                if attempt > args.retries:
                    raise RunStopped("Retry budget exhausted")
                row["outcome"] = "retry"
            except RunStopped as error:
                row["outcome"] = "stopped"
                row["detail"] = f"{row.get('detail', '')} {error}".strip()
                raise
            finally:
                csv_rows.writerow(row)
except (RunStopped, KeyboardInterrupt) as error:
    print(f"Stopped: {error}", flush=True)
    record("run_stopped", reason=str(error))
    exit_code = 128 + stop_signal if stop_signal is not None else 1
finally:
    record("run_end", completed=completed, first_attempt=first_attempt, scheduled=args.iterations)
    print(f"Completed {completed}/{args.iterations} cycles, {first_attempt} on first attempt.",
          flush=True)
    events.close()
    summary.close()
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(exit_code)
PY
