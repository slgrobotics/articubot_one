#!/usr/bin/env python3
"""Measure Seggy firmware response while driving an encoder-controlled square.

Run on Seggy with ROS and other serial clients stopped, a clear test area,
and the physical stop control available. Preview without opening the port:

    python3 src/articubot_one/scripts/seggy_firmware_square.py \
        --linear-percent 15 --angular-percent 15

Add --execute to move. Each side and turn ends using cumulative Hall counts;
these are wheel-odometry estimates, not independent physical measurements.
The JSONL log records commands, replies, samples, segment results, and errors.
Expect four completed sides and turns followed by a confirmed encoder stop.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import time
from datetime import datetime


def count_delta(current, previous):
    """Unwrap a signed 32-bit wire counter."""
    return (current - previous + 2**31) % 2**32 - 2**31


class SerialLink:
    """Exclusive, bounded CR-framed serial transactions; no ROS dependency."""

    def __init__(self, port, baud, timeout):
        baud_constant = getattr(termios, f'B{baud}', None)
        if baud_constant is None:
            raise ValueError(f'Unsupported baud rate: {baud}')
        # TIOCEXCL blocks subsequent opens, but cannot evict an existing client.
        probe = subprocess.run(['fuser', port], capture_output=True, text=True)
        if probe.returncode != 1:
            raise RuntimeError(f'Port is busy or cannot be checked: {probe.stdout} {probe.stderr}')
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.timeout = timeout
        self.buffer = b''
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = baud_constant
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            # Do not toggle DTR or request encoder reset.
            time.sleep(1)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except BaseException:
            os.close(self.fd)
            raise

    def request(self, command):
        deadline = time.monotonic() + self.timeout
        packet = (command + '\r').encode('ascii')
        while packet:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError(f'Serial write timed out: {command}')
            packet = packet[os.write(self.fd, packet):]
        while b'\r' not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError(f'Serial reply timed out: {command}')
            block = os.read(self.fd, 256)
            if not block:
                raise RuntimeError('Serial device disconnected')
            self.buffer += block
            if len(self.buffer) > 1024:
                raise RuntimeError('Oversized serial reply')
        frame, self.buffer = self.buffer.split(b'\r', 1)
        reply = frame.decode('ascii').strip()
        prefix = command[0] if command else '*'
        if not reply.startswith(prefix + ' '):
            raise RuntimeError(f'Unexpected reply to {command!r}: {reply!r}')
        return reply

    def motors(self, left, right):
        reply = self.request(f'm {left} {right}')
        if reply != 'm OK':
            raise RuntimeError(f'Motor command rejected: {reply}')
        return reply

    def encoders(self):
        reply = self.request('e')
        parts = reply.split()
        if len(parts) != 3:
            raise RuntimeError(f'Malformed encoder reply: {reply}')
        counts = tuple(int(p) for p in parts[1:])
        if any(not -2**31 <= value < 2**31 for value in counts):
            raise RuntimeError(f'Encoder count outside signed 32-bit range: {reply}')
        return counts, reply

    def close(self):
        os.close(self.fd)


class SquareTest:
    """Integrate raw wheel counts and execute bounded straight/spin segments."""

    def __init__(self, args, link, log):
        self.args, self.link, self.log = args, link, log
        self.started = time.monotonic()
        self.previous = None
        self.previous_time = None
        self.left = self.right = self.x = self.y = self.heading = 0.0
        self.cycle, self.side, self.phase = 0, 0, 'startup'
        self.command = (0, 0)

    def record(self, event, **data):
        self.log.write(json.dumps(dict(elapsed_s=time.monotonic() - self.started,
                                      cycle=self.cycle, side=self.side, phase=self.phase,
                                      event=event, **data), allow_nan=False) + '\n')
        self.log.flush()

    def motors(self, left, right):
        self.record('command', left_percent=left, right_percent=right)
        reply = self.link.motors(left, right)
        self.command = left, right
        self.record('reply', reply=reply)

    def sample(self):
        counts, reply = self.link.encoders()
        now = time.monotonic()
        dl = dr = velocity = angular = 0.0
        if self.previous is not None:
            scale = math.tau * self.args.wheel_radius / self.args.counts_per_rev
            dl, dr = (count_delta(c, p) * scale for c, p in zip(counts, self.previous))
            dt = now - self.previous_time
            distance = (dl + dr) / 2
            angle = (dr - dl) / self.args.wheel_separation
            # Exact constant-curvature integration, unwrapped heading for turns.
            half = angle / 2
            chord = distance * (math.sin(half) / half if abs(half) > 1e-12 else 1)
            self.x += chord * math.cos(self.heading + half)
            self.y += chord * math.sin(self.heading + half)
            self.heading += angle
            velocity, angular = distance / dt, angle / dt
        self.left += dl
        self.right += dr
        self.previous, self.previous_time = counts, now
        self.record('sample', reply=reply, left_count=counts[0], right_count=counts[1],
                    left_percent=self.command[0], right_percent=self.command[1],
                    left_m=self.left, right_m=self.right, x_m=self.x, y_m=self.y,
                    yaw_deg=math.degrees(self.heading), linear_mps=velocity, angular_rad_s=angular)
        return counts

    def stop(self):
        """Refresh zero until counts remain unchanged for the settling interval."""
        start = unchanged = time.monotonic()
        previous = self.previous
        while time.monotonic() - start < 10:
            self.motors(0, 0)
            counts = self.sample()
            if counts != previous:
                unchanged = time.monotonic()
            previous = counts
            if time.monotonic() - unchanged >= self.args.settle:
                self.record('stop_confirmed', x_m=self.x, y_m=self.y,
                            yaw_deg=math.degrees(self.heading))
                return
            time.sleep(1 / self.args.rate)
        raise RuntimeError('Wheel counts did not settle after stop')

    def segment(self, phase, left, right, target, nominal_speed):
        self.phase = phase
        origin = (self.left + self.right) / 2 if phase == 'straight' else self.heading
        start = last_motion = time.monotonic()
        previous = self.previous
        deadline = start + (self.args.segment_timeout or 3 * target / nominal_speed + 10)
        self.record('segment_start', target=target, nominal_speed=nominal_speed)
        print(f'Cycle {self.cycle}, side {self.side}: {phase} ({left:+d}%, {right:+d}%)', flush=True)
        while True:
            tick = time.monotonic()
            if tick >= deadline:
                raise TimeoutError(f'{phase} segment timed out')
            self.motors(left, right)
            counts = self.sample()
            if counts != previous:
                last_motion = time.monotonic()
            previous = counts
            if time.monotonic() - last_motion > self.args.stall_timeout:
                raise RuntimeError(f'No encoder movement for {self.args.stall_timeout:g} seconds')
            value = (self.left + self.right) / 2 if phase == 'straight' else self.heading
            progress = value - origin
            if phase == 'turn' and self.args.direction == 'cw':
                progress = -progress
            if progress < -.05:
                raise RuntimeError('Encoder movement is opposite to the commanded direction')
            if progress >= target:
                break
            time.sleep(max(0, 1 / self.args.rate - (time.monotonic() - tick)))
        self.stop()
        final = (self.left + self.right) / 2 if phase == 'straight' else self.heading
        actual = final - origin
        if phase == 'turn' and self.args.direction == 'cw':
            actual = -actual
        self.record('segment_end', target=target, actual=actual,
                    unit='m' if phase == 'straight' else 'rad', duration_s=time.monotonic() - start)
        print(f'  stopped: {actual:.3f} {"m" if phase == "straight" else "rad"}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--linear-percent', type=int, default=15)
    parser.add_argument('--angular-percent', type=int, default=15)
    parser.add_argument('--side', type=float, default=1.0, help='Side length, metres')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--direction', choices=('ccw', 'cw'), default='ccw')
    parser.add_argument('--wheel-radius', type=float, default=.122)
    parser.add_argument('--wheel-separation', type=float, default=.473)
    parser.add_argument('--counts-per-rev', type=int, default=60)
    parser.add_argument('--motor-max-rad-s', type=float, default=3.5)
    parser.add_argument('--rate', type=float, default=20, help='Command/sample rate, Hz (1..50)')
    parser.add_argument('--settle', type=float, default=1, help='Unchanged-count interval after stop, seconds (1..5)')
    parser.add_argument('--serial-timeout', type=float, default=.25)
    parser.add_argument('--stall-timeout', type=float, default=5)
    parser.add_argument('--segment-timeout', type=float, default=0, help='Seconds; 0 calculates from speed and distance')
    parser.add_argument('--output', type=Path, default=Path(datetime.now().strftime('seggy_firmware_square_%Y%m%d_%H%M%S.jsonl')))
    parser.add_argument('--execute', action='store_true', help='Open serial port and move; otherwise preview only')
    args = parser.parse_args()
    positive = [args.side, args.wheel_radius, args.wheel_separation, args.motor_max_rad_s,
                args.rate, args.settle, args.serial_timeout, args.stall_timeout]
    if any(not math.isfinite(x) or x <= 0 for x in positive):
        parser.error('Dimensions, speeds, rates, and timeouts must be finite and positive')
    if not 1 <= args.rate <= 50 or not 1 <= args.settle <= 5 or not 0 < args.serial_timeout <= .5:
        parser.error('Use rate 1..50 Hz, settle 1..5 s, and serial timeout up to 0.5 s')
    if not math.isfinite(args.segment_timeout) or args.segment_timeout < 0:
        parser.error('Segment timeout must be finite and nonnegative')
    if args.cycles < 1 or args.counts_per_rev < 1 or any(not 1 <= p <= 100 for p in (args.linear_percent, args.angular_percent)):
        parser.error('Cycles/counts must be positive; speed percentages must be integers 1..100')
    v = args.motor_max_rad_s * args.wheel_radius * args.linear_percent / 100
    w = 2 * args.motor_max_rad_s * args.wheel_radius / args.wheel_separation * args.angular_percent / 100
    print(f'{args.cycles} square(s), {args.side:g} m sides, {args.direction}; nominal v={v:.3f} m/s, turn={w:.3f} rad/s')
    if not args.execute:
        print('Preview only. Add --execute to open the port and move.')
        return 0

    def interrupt(signum, _frame):
        raise KeyboardInterrupt(f'Received {signal.Signals(signum).name}')

    previous_handlers = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    link = test = None
    success = False
    try:
        with args.output.open('x', buffering=1) as log:
            link = SerialLink(args.port, args.baud, args.serial_timeout)
            test = SquareTest(args, link, log)
            test.record('run_start', parameters={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()})
            try:
                test.stop()
                test.record('health', reply=link.request('h'))
                for test.cycle in range(1, args.cycles + 1):
                    for test.side in range(1, 5):
                        test.segment('straight', args.linear_percent, args.linear_percent, args.side, v)
                        p = args.angular_percent * (1 if args.direction == 'ccw' else -1)
                        test.segment('turn', -p, p, math.pi / 2, w)
                success = True
            except (KeyboardInterrupt, Exception) as exc:
                test.record('error', message=str(exc), type=type(exc).__name__)
                print(f'Stopped: {exc}', flush=True)
            finally:
                # A second interrupt must not interrupt the bounded stop attempt.
                for s in previous_handlers:
                    signal.signal(s, signal.SIG_IGN)
                try:
                    test.stop()
                except Exception as exc:
                    success = False
                    test.record('stop_failed', message=str(exc))
                    print(f'STOP NOT CONFIRMED: {exc}. Use physical stop. Firmware serial watchdog is 2 seconds.', flush=True)
                test.record('run_end', success=success, x_m=test.x, y_m=test.y,
                            closure_error_m=math.hypot(test.x, test.y), yaw_deg=math.degrees(test.heading))
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Test could not complete: {exc}', flush=True)
    finally:
        if link is not None:
            link.close()
        for s, handler in previous_handlers.items():
            signal.signal(s, handler)
    print(f'Logs: {args.output.resolve()}', flush=True)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
