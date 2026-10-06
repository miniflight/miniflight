"""Real UDP controller checks on ephemeral ports, without an Unreal process."""

import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
import unittest

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.controllers.r1_gates import Controller as PositionController
from target.aigp.controllers.r1_body_rates import Controller as RatesController
from target.aigp.experiments.recording import RecordedClient, RecordedController, recording, replay
from target.aigp.simulator import AIGPSimulator


class ControllerSmokeTest(unittest.TestCase):
    def test_undefined_gate_approach_stops_before_arming(self):
        for make in (PositionController, RatesController):
            with self.subTest(controller=make.__module__):
                updates, received = self.flight(make)
                self.assertEqual(len(updates), 1)
                self.assertFalse(any(message.get_type() in ("COMMAND_LONG", "SET_ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED")
                                     for message in received))

    def test_active_gate_replacement_changes_target_and_replays(self):
        for make in (PositionController, RatesController):
            with self.subTest(controller=make.__module__):
                updates, received = self.flight(make, replace_geometry=True)
                before = [row for row in updates if row["gates"][0]["center"] == [-2, 0, 0]]
                after = [row for row in updates if row["gates"][0]["center"] == [0, -2, 0]]
                self.assertGreaterEqual(len(before), 3)
                self.assertGreater(len(after), 1)
                self.assertTrue(all(row["gate_index"] == 0 for row in updates))
                self.assertNotEqual(before[-1]["command"], after[0]["command"])
                if make is PositionController:
                    targets = [tuple(row["command"][name] for name in ("north", "east", "down")) for row in after]
                    self.assertEqual(len(set(targets)), 1)  # Identical geometry keeps the chosen point while the drone moves.
                    self.assertGreater(targets[0][0], -.1)
                    self.assertLess(targets[0][1], -2.9)
                    positions = [message for message in received if message.get_type() == "SET_POSITION_TARGET_LOCAL_NED"]
                    self.assertAlmostEqual(positions[-1].x, targets[0][0], delta=1e-6)
                    self.assertAlmostEqual(positions[-1].y, targets[0][1], delta=1e-6)
                    self.assertEqual(positions[-1].type_mask, 3576)
                else:
                    self.assertTrue(all(row["command"]["roll_rate"] < -.5 for row in after))
                    rates = [message for message in received if message.get_type() == "SET_ATTITUDE_TARGET"]
                    self.assertGreater(rates[-2].body_roll_rate, .5)  # FRD rates are negated on the wire.
                    self.assertEqual(rates[-2].type_mask, 144)
                self.assertEqual([message.param1 for message in received if message.get_type() == "COMMAND_LONG"], [1, 0])
                neutral = [message for message in received if message.get_type() == "SET_ATTITUDE_TARGET"][-1]
                self.assertEqual((neutral.body_roll_rate, neutral.body_pitch_rate, neutral.body_yaw_rate, neutral.thrust), (0, 0, 0, 0))

    def flight(self, make, replace_geometry=False):
        directory = self.enterContext(tempfile.TemporaryDirectory())
        path = Path(directory) / "trace.jsonl"
        stop = threading.Event()
        failures, received = [], []
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, recording(path) as record:
            server.bind(("127.0.0.1", 0))
            server.setblocking(False)
            client = RecordedClient(record, port=0, camera_port=None)
            encoder = mavlink.MAVLink(None, srcSystem=42, srcComponent=7)
            decoder = mavlink.MAVLink(None)

            def drain():
                while True:
                    try:
                        packet, _ = server.recvfrom(65536)
                    except BlockingIOError:
                        return
                    received.extend(decoder.parse_buffer(packet) or ())

            def serve():
                sequence = 0
                try:
                    while not stop.wait(.005):
                        sock = client._socket
                        if sock is None:
                            continue
                        try:
                            peer = sock.getsockname()
                        except OSError:
                            continue
                        drain()
                        commands = sum(message.get_type() in ("SET_ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED") for message in received)
                        replaced = replace_geometry and commands >= 3
                        center = ((0, -2, 0) if replaced else (-2, 0, 0)) if replace_geometry else (0, 0, 0)
                        transfer = 8 if replaced else 7
                        # Identity rotation and height 2: the base is one metre below the opening center.
                        track = struct.pack("<H", 1) + struct.pack("<H9f", 0, center[0], center[1], center[2] + 1, 1, 0, 0, 0, 2, 2)
                        sequence += 1
                        stamp = sequence * 5000
                        north = .2 * stamp * 1e-6 if replace_geometry else 0
                        finish = 1000000000 if commands >= 10 else -1
                        race = struct.pack("<BQqqIq", 1, stamp // 1000, 0, finish, 0, 0).ljust(253, b"\0")
                        messages = (
                            mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 4, 3),
                            mavlink.MAVLink_data_transmission_handshake_message(0, len(track), transfer, 0, 1, 253, 1),
                            mavlink.MAVLink_encapsulated_data_message(0, (struct.pack("<BH", 2, transfer) + track).ljust(253, b"\0")),
                            mavlink.MAVLink_encapsulated_data_message(0, race),
                            mavlink.MAVLink_local_position_ned_message(stamp // 1000, north, 0, 0, .2 if replace_geometry else 0, 0, 0),
                            mavlink.MAVLink_attitude_message(stamp // 1000, 0, 0, 0, 0, 0, 0),
                            mavlink.MAVLink_highres_imu_message(stamp, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xffff),
                        )
                        for message in messages:
                            server.sendto(message.pack(encoder), peer)
                except BaseException as error:
                    failures.append(error)

            worker = threading.Thread(target=serve)
            worker.start()
            simulator = AIGPSimulator(RecordedController(make(), record), startup_timeout=3, client=client)
            try:
                if replace_geometry:
                    simulator.rollout(attach=True)
                else:
                    with self.assertRaisesRegex(ValueError, "gate 0 has no approach direction"):
                        simulator.rollout(attach=True)
            finally:
                stop.set()
                worker.join(timeout=2)
                client.disconnect()
            self.assertFalse(worker.is_alive())
            self.assertEqual(failures, [])
            drain()
            self.assertIsNone(client._socket)
            self.assertFalse(client.connected)
        updates = [json.loads(line) for line in path.read_text().splitlines() if json.loads(line)["event"] == "update"]
        self.assertEqual(replay(make(), path), len(updates))
        return updates, received


if __name__ == "__main__":
    unittest.main()
