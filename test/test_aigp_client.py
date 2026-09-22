import math
import struct
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from miniflight import Control, PositionNed, Vehicle
from target.aigp import SimulatorClient
from target.aigp.client import _Camera as Camera


PEER = ("127.0.0.1", 14560)


def packet(message, system=42, component=7):
    return message.pack(mavlink.MAVLink(None, srcSystem=system, srcComponent=component))


def heartbeat():
    return mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 4, 3)


def imu(stamp=1000000, xacc=1.0):
    return mavlink.MAVLink_highres_imu_message(
        stamp, xacc, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0xffff,
    )


class Socket:
    def __init__(self, packets=()):
        self.packets = list(packets)
        self.sent = []
        self.closed = False

    def recvfrom(self, size):
        return self.packets.pop(0)

    def sendto(self, data, peer):
        self.sent.append((bytes(data), peer))
        return len(data)

    def close(self):
        self.closed = True


class ClientTest(unittest.TestCase):
    def setUp(self):
        self.wire = Socket([(packet(heartbeat()), PEER)])
        self.camera = Socket()
        self.sim = SimulatorClient()
        self.enterContext(patch.object(SimulatorClient, "_bind", side_effect=[self.wire, self.camera]))
        self.enterContext(patch("target.aigp.client.select.select", side_effect=
                                lambda sockets, *args: ([s for s in sockets if s.packets], [], [])))
        self.enterContext(patch("target.aigp.client.time.monotonic", return_value=10.0))
        self.sim.connect()
        self.addCleanup(self.sim.disconnect)

    def feed(self, message, **kwargs):
        self.wire.packets.append((packet(message, **kwargs), PEER))

    def output(self):
        return [mavlink.MAVLink(None).parse_buffer(data)[0] for data, _ in self.wire.sent]

    def test_connect_is_passive_and_discovers_target(self):
        self.assertEqual(self.sim._target, (42, 7))
        self.assertEqual(self.wire.sent, [])
        with self.assertRaisesRegex(RuntimeError, "already connected"):
            self.sim.connect()

    def test_vq2_needs_only_imu_not_privileged_pose(self):
        self.feed(imu())
        state = self.sim.read()
        self.assertEqual((state.time, state.dt), (1.0, 0.0))
        self.assertEqual(state.acceleration, (1, 2, 3))
        self.assertEqual(state.gyro, (4, 5, 6))
        self.assertIsNone(state.frame)
        self.assertIsNone(state.race)
        self.assertNotIn("LOCAL_POSITION_NED", state.telemetry)
        self.assertEqual(state.received_at["HIGHRES_IMU"], 10.0)
        self.feed(imu(1020000))
        second = self.sim.read()
        self.assertAlmostEqual(second.dt, .02)
        self.assertEqual(len(second.messages), 1)
        self.assertEqual(state.telemetry["HIGHRES_IMU"].time_usec, 1000000)
        with self.assertRaises(TypeError):
            state.telemetry["fake"] = 1

    def test_vq1_pose_is_preserved_without_changing_common_input(self):
        self.feed(mavlink.MAVLink_local_position_ned_message(1, 1, 2, 3, 4, 5, 6))
        self.feed(imu())
        state = self.sim.read()
        self.assertEqual(state.telemetry["LOCAL_POSITION_NED"].vx, 4)
        self.assertEqual(state.gyro, (4, 5, 6))

    def test_sensor_component_can_differ_from_heartbeat(self):
        self.feed(imu(), component=8)
        self.assertEqual(self.sim.read().time, 1.0)

    def test_unrelated_system_and_peer_are_ignored(self):
        self.feed(imu(2000000), system=99)
        self.wire.packets.append((packet(imu(3000000)), ("127.0.0.1", 20000)))
        self.feed(imu())
        self.assertEqual(self.sim.read().time, 1.0)

    def test_old_duplicate_and_nonfinite_imu_samples_are_ignored(self):
        self.feed(imu())
        self.sim.read()
        for sample in (imu(1000000), imu(900000), imu(1100000, math.nan)):
            self.feed(sample)
        self.feed(imu(1020000))
        state = self.sim.read()
        self.assertEqual(len(state.messages), 1)
        self.assertAlmostEqual(state.dt, .02)

    def test_startup_clock_reset_accepts_the_new_imu_stream(self):
        for boot, start in ((2000, -1), (3000, 4000)):
            with self.subTest(start=start):
                payload = struct.pack("<BQqqIq", 1, boot, start, -1, 0, 0).ljust(253, b"\0")
                self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
                self.feed(imu(boot * 1000))
                self.sim.read(timeout=0)
                payload = struct.pack("<BQqqIq", 1, 100, start, -1, 0, 0).ljust(253, b"\0")
                self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
                self.feed(imu(100000))
                state = self.sim.read(timeout=0)
                self.assertAlmostEqual(state.time, .1)
                self.assertEqual(state.dt, 0.0)
                self.assertEqual(state.race.sim_boot_time_ms, 100)
                self.feed(imu(120000))
                self.assertAlmostEqual(self.sim.read(timeout=0).dt, .02)

    def test_clock_reset_after_go_does_not_rebase_the_imu(self):
        payload = struct.pack("<BQqqIq", 1, 2000, 1000, -1, 0, 0).ljust(253, b"\0")
        self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
        self.feed(imu(2000000))
        self.sim.read(timeout=0)
        payload = struct.pack("<BQqqIq", 1, 100, -1, -1, 0, 0).ljust(253, b"\0")
        self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
        self.feed(imu(100000))
        with self.assertRaisesRegex(TimeoutError, "fresh IMU"):
            self.sim.read(timeout=0)
        self.assertEqual(self.sim.race.sim_boot_time_ms, 100)

    def test_read_requires_fresh_data(self):
        self.feed(imu())
        self.sim.read()
        with self.assertRaisesRegex(TimeoutError, "fresh IMU"):
            self.sim.read(timeout=0)

    def test_race_and_events_are_available(self):
        payload = struct.pack("<BQqqIq", 1, 1000, -1, -1, 3, 0).ljust(253, b"\0")
        self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
        self.feed(mavlink.MAVLink_collision_message(0, 1001, 0, 2, 0, 0, 8))
        self.feed(imu())
        state = self.sim.read()
        self.assertEqual(state.race.active_gate_index, 3)
        self.assertEqual(state.race.race_start_boot_time_ms, -1)
        self.assertEqual(state.race.received_at, 10.0)
        self.assertIn("COLLISION", [m.get_type() for m in state.messages])

    def test_race_finish_is_available_without_imu(self):
        self.assertIsNone(self.sim.race)
        payload = struct.pack("<BQqqIq", 1, 2000, 500, 123, 6, 0).ljust(253, b"\0")
        self.feed(mavlink.MAVLink_encapsulated_data_message(0, payload))
        with self.assertRaisesRegex(TimeoutError, "fresh IMU"):
            self.sim.read(timeout=0)
        self.assertEqual(self.sim.race.race_finish_time_ns, 123)
        self.assertEqual(self.sim.race.active_gate_index, 6)
        self.assertEqual(self.sim.race.received_at, 10.0)
        # Other packet types must not refresh the race packet's own age.
        with patch("target.aigp.client.time.monotonic", return_value=10.5):
            self.feed(heartbeat())
            self.sim.poll()
        self.assertEqual(self.sim.race.received_at, 10.0)

    def test_body_rates_use_radians_extension_and_discovered_target(self):
        self.sim.send(Control(.1, -.2, .3, .4))
        message, = self.output()
        self.assertEqual(message.get_type(), "SET_ATTITUDE_TARGET")
        self.assertEqual((message.target_system, message.target_component), (42, 7))
        self.assertEqual(message.type_mask, 144)
        self.assertAlmostEqual(message.body_pitch_rate, -.2)
        self.assertAlmostEqual(message.thrust, .4)
        self.assertEqual(self.wire.sent[0][1], PEER)

    def test_arming_and_disarming_use_standard_commands(self):
        self.sim.arm()
        self.sim.disarm()
        for message, armed in zip(self.output(), (1, 0)):
            self.assertEqual(message.command, mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            self.assertEqual(message.param1, armed)

    def test_position_command_uses_builtin_position_control(self):
        self.sim.send(PositionNed(-23, 2, -1.5))
        message, = self.output()
        self.assertEqual(message.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(message.coordinate_frame, mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(message.type_mask, 3576)
        self.assertEqual((message.target_system, message.target_component), (42, 7))
        self.assertEqual((message.x, message.y, message.z), (-23, 2, -1.5))
        self.assertEqual((message.vx, message.vy, message.vz), (0, 0, 0))

    def test_invalid_position_command_is_rejected(self):
        for values in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                PositionNed(*values)
        self.assertEqual(self.wire.sent, [])

    def test_vehicle_api_uses_shared_body_rates_and_ned_encoding(self):
        vehicle = Vehicle(self.sim)
        vehicle.body_rates(.1, .2, .3, .4)
        vehicle.position_ned(1, 2, -3)
        vehicle.velocity_ned(4, 5, -6)
        rates, position, velocity = self.output()
        self.assertEqual(rates.type_mask, 144)
        self.assertEqual(position.coordinate_frame, mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(position.type_mask, 3576)
        self.assertEqual((position.x, position.y, position.z), (1, 2, -3))
        self.assertEqual(velocity.type_mask, 3527)
        self.assertEqual((velocity.vx, velocity.vy, velocity.vz), (4, 5, -6))

    def test_position_and_velocity_remain_callable(self):
        self.feed(mavlink.MAVLink_local_position_ned_message(1, 1, 2, 3, 4, 5, 6))
        self.assertEqual(self.sim.position(), (1, 2, 3))
        self.feed(mavlink.MAVLink_local_position_ned_message(2, 7, 8, 9, 10, 11, 12))
        self.assertEqual(self.sim.velocity(), (10, 11, 12))

    def test_disconnect_closes_both_sockets_and_is_idempotent(self):
        self.sim.disconnect()
        self.sim.disconnect()
        self.assertTrue(self.wire.closed)
        self.assertTrue(self.camera.closed)

    def test_invalid_control_does_not_send(self):
        for values in ((math.nan, 0, 0, 0), (0, math.inf, 0, 0), (0, 0, 0, -1), (0, 0, 0, 1.01)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Control(*values)
        with self.assertRaises(TypeError):
            self.sim.send((0, 0, 0, 0))
        self.assertEqual(self.wire.sent, [])


class ConnectionFailureTest(unittest.TestCase):
    def test_ports_can_be_reserved_before_the_simulator_starts(self):
        sock = Socket()
        sim = SimulatorClient(camera_port=None)
        self.addCleanup(sim.disconnect)
        with patch.object(sim, "_bind", return_value=sock) as bind:
            sim.open()
            self.assertFalse(sim.connected)
            self.assertEqual(sock.sent, [])
            self.assertFalse(sock.closed)
            with self.assertRaisesRegex(RuntimeError, "already open"):
                sim.open()
            sock.packets.append((packet(heartbeat()), PEER))
            with patch("target.aigp.client.select.select", side_effect=
                       lambda *args: ([sock] if sock.packets else [], [], [])):
                sim.connect()
            self.assertTrue(sim.connected)
            bind.assert_called_once()

    def test_no_heartbeat_times_out_and_closes(self):
        sock = Socket()
        sim = SimulatorClient(camera_port=None)
        with patch.object(sim, "_bind", return_value=sock):
            with self.assertRaisesRegex(TimeoutError, "heartbeat"):
                sim.connect(timeout=0)
        self.assertTrue(sock.closed)
        self.assertEqual(sock.sent, [])

    def test_camera_port_conflict_closes_telemetry_socket(self):
        sock = Socket()
        sim = SimulatorClient()
        with patch.object(sim, "_bind", side_effect=[sock, OSError("port in use")]):
            with self.assertRaises(OSError):
                sim.connect()
        self.assertTrue(sock.closed)


class CameraTest(unittest.TestCase):
    def setUp(self):
        self.camera = Camera()
        image = np.full((12, 16, 3), (10, 80, 160), dtype=np.uint8)
        ok, data = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        self.jpeg = data.tobytes()

    def chunks(self, frame=1, timestamp=1000000000):
        middle = len(self.jpeg) // 2
        pieces = (self.jpeg[:middle], self.jpeg[middle:])
        return [Camera.HEADER.pack(frame, i, 2, len(self.jpeg), len(piece), timestamp) + piece
                for i, piece in enumerate(pieces)]

    def test_out_of_order_chunks_and_duplicates_decode_once(self):
        first, second = self.chunks()
        self.camera.receive(second, 10)
        self.camera.receive(second, 10)
        self.assertIsNone(self.camera.latest)
        self.camera.receive(first, 10.1)
        frame = self.camera.latest
        self.assertEqual((frame.id, frame.time_ns, frame.received_at), (1, 1000000000, 10.1))
        self.assertEqual(frame.bgr.shape, (12, 16, 3))
        np.testing.assert_allclose(frame.bgr[0, 0], (10, 80, 160), atol=2)
        self.assertFalse(frame.bgr.flags.writeable)
        for chunk in self.chunks():
            self.camera.receive(chunk, 10.2)
        self.assertIs(self.camera.latest, frame)

    def test_incomplete_frames_expire_and_memory_is_bounded(self):
        for i in range(10):
            self.camera.receive(self.chunks(frame=i)[0], 10)
        self.assertEqual(len(self.camera.pending), Camera.MAX_FRAMES)
        self.camera.receive(b"", 11)
        self.assertEqual(self.camera.pending, {})
        self.assertIsNone(self.camera.latest)

    def test_invalid_headers_payloads_and_jpeg_are_ignored(self):
        packets = [b"short", self.chunks()[0][:-1],
                   Camera.HEADER.pack(1, 2, 2, 1, 1, 1) + b"x",
                   Camera.HEADER.pack(1, 0, 1, Camera.MAX_BYTES + 1, 1, 1) + b"x",
                   Camera.HEADER.pack(1, 0, 1, 1, 1, 1) + b"x"]
        for data in packets:
            self.camera.receive(data, 10)
        self.assertIsNone(self.camera.latest)

    def test_inconsistent_chunk_metadata_discards_frame(self):
        self.camera.receive(self.chunks()[0], 10)
        self.camera.receive(self.chunks(timestamp=2000000000)[1], 10)
        self.assertEqual(self.camera.pending, {})
        self.assertIsNone(self.camera.latest)


if __name__ == "__main__":
    unittest.main()
