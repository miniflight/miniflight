"""Real UDP controller checks on ephemeral ports, without an Unreal process."""

import json
import base64
from dataclasses import asdict, replace
from pathlib import Path
import socket
import struct
import tempfile
import threading
import unittest

from pymavlink.dialects.v20 import common as mavlink

from common.math import Vector3D
from miniflight import Ned
from target.aigp.controllers import Gate
from target.aigp.controllers.r1_gates import Controller as PositionController
from target.aigp.controllers.r1_body_rates import Controller as RatesController
from target.aigp.controllers.r1_beautiful import Controller as PIDController
from target.aigp.experiments.recording import RecordedClient, RecordedController, recording, replay
from target.aigp.simulator import AIGPSimulator
from target.aigp.client import Packet, SimulatorClient, TrackInfo
from target.aigp.native import NativeRaceState
from test.aigp_fake_simulator import write_race
from test.aigp_udp_smoke import imu


class ControllerSmokeTest(unittest.TestCase):
    def test_pid_uses_native_time_when_imu_clock_advances(self):
        controller = PIDController(kp=0, ki=1, kd=0)
        gate = Gate(0, Ned(2, 0, 1), (1, 0, 0, 0), 2, 2)
        packets = (
            Packet(mavlink.MAVLink_local_position_ned_message(5000, 0, 0, 0, 0, 0, 0), 10),
            Packet(mavlink.MAVLink_attitude_message(5000, 0, 0, 0, 0, 0, 0), 10),
            Packet(mavlink.MAVLink_encapsulated_data_message(0, bytes([2]).ljust(253, b"\0")),
                   10, TrackInfo(7, (gate,))),
        )
        race = NativeRaceState(started=True, valid=True, completed=False, time_seconds=0,
                               active_gate_index=0, finish_time_seconds=-1, received_at=10)
        self.assertIsNotNone(controller.update(packets, (), race))
        for native_time, imu_time, receipt, integral in ((.5, 50000000, 10.1, 1.5), (.5, 100000000, 10.2, 1.5)):
            controller.update((Packet(imu(imu_time), receipt),), (),
                              replace(race, time_seconds=native_time, received_at=receipt))
            self.assertEqual(tuple(controller.integral.v), (integral, 0, 0))
        with self.assertRaisesRegex(TimeoutError, "privileged pose is stale"):
            controller.update((), (), replace(race, time_seconds=.5, received_at=10.5))
        # Native completion can arrive after the final sensor packet.
        self.assertIsNotNone(controller.update((), (), replace(race, completed=True, time_seconds=.5,
                             active_gate_index=1, finish_time_seconds=.5, received_at=11)))

    def test_pid_integral_uses_time_and_unwinds_from_its_limit(self):
        controller = PIDController(kp=0, ki=1, kd=0)
        zero, target = Vector3D(), Vector3D(1, 0, 0)
        for dt, expected in ((.5, .5), (0, .5), (100, 3)):
            self.assertEqual(controller.desired_acceleration(zero, zero, target, dt).v[0], expected)
        self.assertEqual(controller.desired_acceleration(zero, zero, -target, 1).v[0], 2)

    def test_undefined_gate_approach_stops_before_arming(self):
        for make in (PositionController, RatesController, PIDController):
            with self.subTest(controller=make.__module__):
                updates, received = self.flight(make)
                self.assertTrue(all(row.get("command") is None for row in updates))
                self.assertEqual(updates[-1]["error"]["type"], "ValueError")
                self.assertFalse(any(message.get_type() in ("COMMAND_LONG", "SET_ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED")
                                     for message in received))

    def test_active_gate_replacement_changes_target_and_replays(self):
        for make in (PositionController, RatesController, PIDController):
            with self.subTest(controller=make.__module__):
                updates, received = self.flight(make, replace_geometry=True)
                before = [row for row in updates if row["gates"][0]["position"] == [-2, 0, 1]]
                after = [row for row in updates if row["gates"][0]["position"] == [0, -2, 1]]
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
        native_path = Path(directory) / "native.jsonl"
        native = self.enterContext(native_path.open("w"))
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
                        write_race(native, completed=finish >= 0)
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
            simulator = AIGPSimulator(RecordedController(make, record), startup_timeout=3, client=client,
                                      race_status_path=native_path)
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
        updates, observed, decoder = [], SimulatorClient(camera_port=None), mavlink.MAVLink(None)
        for line in path.read_text().splitlines():
            row = json.loads(line)
            if row["event"] != "update":
                continue
            for saved in row["telemetry"]:
                message, = decoder.parse_buffer(base64.b64decode(saved["wire"]))
                observed._receive(message, ("test", 0), saved["received_at"])
            row["gates"] = None if observed.gates is None else json.loads(json.dumps([asdict(gate) for gate in observed.gates]))
            row["gate_index"] = None if row["native_race"] is None else row["native_race"]["active_gate_index"]
            updates.append(row)
        self.assertEqual(replay(make, path), len(updates))
        return updates, received


if __name__ == "__main__":
    unittest.main()
