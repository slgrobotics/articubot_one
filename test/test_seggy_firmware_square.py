"""Verify direct firmware square geometry, stop handling, and serial framing.

The tests emulate wheel counters and a CR-framed serial peer. No robot,
serial device, ROS installation, or motor command is required.

    python3 -m unittest discover -s src/articubot_one/test -p test_seggy_firmware_square.py

Run from the workspace root. Expect all tests to pass, including a full square,
encoder rollover, malformed replies, and a bounded missing-reply timeout.
"""
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import pty
import threading
import tempfile
import tty
from types import SimpleNamespace
import unittest
from unittest import mock

path = Path(__file__).parents[1] / 'scripts/seggy_firmware_square.py'
spec = importlib.util.spec_from_file_location('firmware_square', path)
square = importlib.util.module_from_spec(spec)
spec.loader.exec_module(square)


class FakeFirmware:
    def __init__(self):
        self.command = (0, 0)
        self.counts = [2**31 - 3, 2**31 - 3]

    def motors(self, left, right):
        self.command = left, right
        return 'm OK'

    def encoders(self):
        for i, command in enumerate(self.command):
            self.counts[i] += (1 if command > 0 else -1) if command else 0
        values = tuple((c + 2**31) % 2**32 - 2**31 for c in self.counts)
        return values, f'e {values[0]} {values[1]}'


class SquareTests(unittest.TestCase):
    def test_wraparound_both_directions(self):
        self.assertEqual(square.count_delta(-2**31, 2**31 - 1), 1)
        self.assertEqual(square.count_delta(2**31 - 1, -2**31), -1)

    def test_complete_square_both_directions_and_stops(self):
        for direction in ('cw', 'ccw'):
            args = SimpleNamespace(wheel_radius=.122, wheel_separation=.473,
                                   counts_per_rev=60, settle=.001, rate=1000,
                                   segment_timeout=1, stall_timeout=.1, direction=direction)
            peer, log = FakeFirmware(), io.StringIO()
            run = square.SquareTest(args, peer, log)
            run.sample()
            with mock.patch('builtins.print'):
                for side in range(4):
                    run.segment('straight', 15, 15, 1, .064)
                    p = 20 if direction == 'ccw' else -20
                    run.segment('turn', -p, p, math.pi / 2, .36)
            self.assertEqual(peer.command, (0, 0))
            events = [json.loads(l) for l in log.getvalue().splitlines()]
            self.assertEqual(sum(e['event'] == 'segment_end' for e in events), 8)
            # Whole Hall-count steps overshoot each segment; bound their accumulation.
            tick = math.tau * args.wheel_radius / args.counts_per_rev
            self.assertLess(math.hypot(run.x, run.y), .25)
            self.assertLess(abs(abs(run.heading) - math.tau), 4 * 2 * tick / args.wheel_separation)
            for event in events:
                if event['event'] == 'segment_end':
                    quantum = tick if event['unit'] == 'm' else 2 * tick / args.wheel_separation
                    self.assertGreaterEqual(event['actual'], event['target'])
                    self.assertLess(event['actual'] - event['target'], quantum + 1e-9)

    def test_main_stops_after_serial_error(self):
        class FailingFirmware(FakeFirmware):
            def __init__(self, *args):
                super().__init__()
                self.failed = self.closed = False

            def encoders(self):
                if self.command != (0, 0) and not self.failed:
                    self.failed = True
                    raise TimeoutError('injected missing encoder reply')
                return super().encoders()

            def request(self, command):
                return 'h 13000 0 0 0'

            def close(self):
                self.closed = True

        peer = FailingFirmware()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'run.jsonl'
            argv = ['square', '--execute', '--output', str(output)]
            with mock.patch.object(square, 'SerialLink', return_value=peer), \
                    mock.patch.object(square.sys, 'argv', argv), \
                    mock.patch('builtins.print'):
                self.assertEqual(square.main(), 1)
            events = [json.loads(l) for l in output.read_text().splitlines()]
        self.assertEqual(peer.command, (0, 0))
        self.assertTrue(peer.closed)
        self.assertTrue(any(e['event'] == 'error' for e in events))
        self.assertEqual(events[-2]['event'], 'stop_confirmed')
        self.assertFalse(events[-1]['success'])

    def transaction(self, reply):
        master, slave = pty.openpty()
        tty.setraw(slave)
        link = square.SerialLink.__new__(square.SerialLink)
        link.fd, link.timeout, link.buffer = slave, .05, b''
        def peer():
            os.read(master, 128)
            if reply is not None:
                os.write(master, reply)
        thread = threading.Thread(target=peer)
        thread.start()
        try:
            return link.request('m 15 15')
        finally:
            thread.join(timeout=1)
            os.close(master)
            os.close(slave)

    def test_firmware_acknowledgment(self):
        self.assertEqual(self.transaction(b'm OK\r'), 'm OK')

    def test_mismatched_reply_rejected(self):
        with self.assertRaises(RuntimeError):
            self.transaction(b'e 10 10\r')

    def test_missing_reply_times_out(self):
        with self.assertRaises(TimeoutError):
            self.transaction(None)


if __name__ == '__main__':
    unittest.main()
