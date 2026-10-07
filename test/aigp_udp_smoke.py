"""Real UDP round-trip using ephemeral loopback ports, never the simulator ports.

Run explicitly: python -m test.aigp_udp_smoke
"""

from contextlib import contextmanager
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
import json
import math
import time
from unittest.mock import patch

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from target.aigp.simulator import AIGPSimulator
from target.aigp.native import _stop_process
from target.aigp.controllers import BaseController
from target.aigp.client import RaceStatus
from target.aigp.controllers.r1_gates import Controller as Gates
from miniflight import BodyRates, State, VelocityNed
from target.aigp.simulator import SimulatorClient
from target.aigp.client import _Camera as Camera
from target.aigp.experiments.recording import RecordedClient, RecordedController, read_state, recording, replay


def packet(message, system=42, component=7):
    return message.pack(mavlink.MAVLink(None, srcSystem=system, srcComponent=component))


def heartbeat():
    return mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 4, 3)


def imu(stamp=1000000):
    return mavlink.MAVLink_highres_imu_message(
        stamp, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0xffff)


class UDPSmokeTest(unittest.TestCase):
    def test_poll_preserves_native_packets_and_receipt_times(self):
        server = self.enterContext(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        server.bind(("127.0.0.1", 0))
        client = SimulatorClient(port=0, camera_port=0)
        self.addCleanup(client.disconnect)
        client.open()
        address, camera_address = client._socket.getsockname(), client._vision.getsockname()
        invalid = imu(1000002)
        invalid.xacc = math.nan
        course = struct.pack("<HH9f", 1, 0, 1, 2, 3, 1, 0, 0, 0, 2, 2)
        messages = (
            heartbeat(),
            mavlink.MAVLink_data_transmission_handshake_message(0, len(course), 7, 0, 1, 253, 1),
            mavlink.MAVLink_encapsulated_data_message(0, (struct.pack("<BH", 2, 7) + course).ljust(253, b"\0")),
            imu(1000001), imu(1000001), imu(1000000), invalid,
        )
        packets = [packet(message) for message in messages]
        for data in packets:
            server.sendto(data, address)
        arrivals = client.poll(.1)
        self.assertEqual([bytes(item.data.get_msgbuf()) for item in arrivals], packets)
        self.assertEqual([item.data.time_usec for item in arrivals[3:]], [1000001, 1000001, 1000000, 1000002])
        self.assertTrue(math.isnan(arrivals[-1].data.xacc))
        self.assertTrue(all(item.received_at <= time.monotonic() for item in arrivals))
        self.assertEqual(client.telemetry["HIGHRES_IMU"].time_usec, 1000001)
        self.assertEqual(struct.unpack_from("<H9f", bytes(arrivals[2].data.data), 5)[1:4], (1, 2, 3))
        self.assertEqual(arrivals[2].decoded, client.gates)
        self.assertEqual(client.poll(), ())

        # Incomplete, repeated and old camera packets remain visible byte for byte.
        camera_packets = [Camera.HEADER.pack(8, 1, 2, 400, 200, stamp) + b"x" * 200
                          for stamp in (123456789, 123456789, 123456788)]
        for data in camera_packets:
            server.sendto(data, camera_address)
        camera_arrivals = client.poll(.1)
        self.assertEqual([item.data for item in camera_arrivals], camera_packets)
        self.assertTrue(all(item.received_at >= arrivals[-1].received_at for item in camera_arrivals))
        self.assertIsNone(client._camera.latest)
        self.assertEqual(client.poll(), ())

        race = struct.pack("<BQqqIq", 1, 5000, 4500, -1, 2, -1).ljust(253, b"\0")
        server.sendto(packet(mavlink.MAVLink_encapsulated_data_message(0, race)), address)
        _, jpeg = cv2.imencode(".jpg", np.zeros((2, 3, 3), dtype=np.uint8))
        camera = Camera.HEADER.pack(9, 0, 1, len(jpeg), len(jpeg), 123456900) + jpeg.tobytes()
        server.sendto(camera, camera_address)
        arrivals = client.poll(.1)
        status = next(item.decoded for item in arrivals if isinstance(item.decoded, RaceStatus))
        self.assertEqual((status.sim_boot_time_ms, status.active_gate_index), (5000, 2))
        frame_packet = next(item for item in arrivals if isinstance(item.data, bytes))
        self.assertEqual(frame_packet.data, camera)
        self.assertEqual((frame_packet.decoded.id, frame_packet.decoded.time_ns), (9, 123456900))
        self.assertEqual(frame_packet.decoded.bgr.shape, (2, 3, 3))
        self.assertEqual(frame_packet.decoded.received_at, frame_packet.received_at)
        self.assertEqual(client.poll(), ())

    def test_track_arrival_is_independent_of_imu_reads(self):
        server = self.enterContext(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        server.bind(("127.0.0.1", 0))
        client = SimulatorClient(port=0, camera_port=None)
        self.addCleanup(client.disconnect)
        client.open()
        address = client._socket.getsockname()

        def send(message):
            server.sendto(packet(message), address)

        def announce(transfer, track):
            send(mavlink.MAVLink_data_transmission_handshake_message(0, len(track), transfer, 0, (len(track) + 249) // 250, 253, 1))

        def fragment(transfer, track, index=0):
            payload = (struct.pack("<BH", 2, transfer) + track[index * 250:(index + 1) * 250]).ljust(253, b"\0")
            send(mavlink.MAVLink_encapsulated_data_message(index, payload))

        track = struct.pack("<HH9f", 1, 0, 1, 2, 3, .9999, 0, 0, 0, 2, 2)
        send(heartbeat())
        announce(7, track)
        client.poll(.1)
        self.assertIsNone(client.gates)
        self.assertIsNone(client.gates_received_at)
        fragment(7, track)
        client.poll(.1)
        gates, received_at = client.gates, client.gates_received_at
        self.assertEqual(gates[0].position, (1, 2, 3))
        self.assertEqual(gates[0].orientation, struct.unpack_from("<4f", track, 16))
        self.assertFalse(hasattr(gates[0], "center"))
        self.assertLessEqual(received_at, time.monotonic())

        # A missing or arriving IMU changes neither course nor its receipt time.
        with self.assertRaises(TimeoutError):
            client.read(.02)
        send(imu())
        state = client.read(.1)
        self.assertIsNone(state.motion)
        self.assertIs(client.gates, gates)
        self.assertEqual(client.gates_received_at, received_at)

        # An announcement alone retains the previous complete course.
        announce(8, track)
        client.poll(.1)
        self.assertIs(client.gates, gates)
        self.assertEqual(client.gates_received_at, received_at)
        fragment(8, track)
        client.poll(.1)
        self.assertEqual(client.gates, gates)
        self.assertGreater(client.gates_received_at, received_at)

        # Redacted geometry cannot replace the usable course or its timestamp.
        redacted = struct.pack("<HH9f", 1, 0, *([0] * 9))
        announce(9, redacted)
        fragment(9, redacted)
        retained_at = client.gates_received_at
        arrivals = client.poll(.1)
        self.assertEqual(arrivals[-1].decoded[0].orientation, (0, 0, 0, 0))
        self.assertEqual(arrivals[-1].decoded[0].width, 0)
        self.assertEqual(client.gates, gates)
        self.assertEqual(client.gates_received_at, retained_at)

        # Indexed fragments can arrive twice or backwards, with a repeated announcement.
        track = struct.pack("<H", 7) + b"".join(struct.pack("<H9f", i, i, 2, 3, 1, 0, 0, 0, 2, 2) for i in range(7))
        announce(10, track)
        fragment(10, track, 1)
        announce(10, track)
        fragment(11, track)
        fragment(10, track, 2)
        fragment(10, track, 1)
        client.poll(.1)
        self.assertEqual(client.gates, gates)
        self.assertEqual(client.gates_received_at, retained_at)
        fragment(10, track)
        client.poll(.1)
        self.assertEqual([gate.position for gate in client.gates], [(i, 2, 3) for i in range(7)])
        self.assertGreater(client.gates_received_at, retained_at)

    def test_shared_runner_with_and_without_pose_telemetry(self):
        for with_pose, invalid_optional in ((True, False), (False, False), (True, True)):
            with self.subTest(with_pose=with_pose, invalid_optional=invalid_optional):
                self.round_trip(with_pose, invalid_optional=invalid_optional)

    def test_raw_mavlink_inputs_and_outputs_round_trip(self):
        self.round_trip(True, wire_io=True)

    def test_camera_capture_policy_round_trip(self):
        for frames in (False, True):
            with self.subTest(frames=frames):
                self.round_trip(True, record_frames=frames)

    def test_velocity_command_round_trip(self):
        self.round_trip(True, VelocityNed(-1, 0, 0))

    def test_closed_transport_reports_cleanup_failure(self):
        self.round_trip(False, closed_on_exit=True)

    def round_trip(self, with_pose, command=BodyRates(.1, -.2, .3, .4), invalid_optional=False, closed_on_exit=False, wire_io=False, record_frames=None):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(server.close)
        server.bind(("127.0.0.1", 0))
        server.setblocking(False)
        stop = threading.Event()
        received, failures, states = [], [], []
        _, encoded = cv2.imencode(".jpg", np.zeros((12, 16, 3), dtype=np.uint8))
        jpeg = encoded.tobytes()

        def drain():
            while True:
                try:
                    data, _ = server.recvfrom(65536)
                except BlockingIOError:
                    return
                received.extend(mavlink.MAVLink(None).parse_buffer(data) or ())

        def serve():
            sequence = 0
            try:
                while not stop.wait(.005):
                    sock, vision = client._socket, client._vision
                    if sock is None or vision is None:
                        continue
                    try:
                        address, camera_address = sock.getsockname(), vision.getsockname()
                    except OSError:  # the runner has just closed its sockets
                        continue
                    sequence += 1
                    stamp = 1000000 + sequence * 5000
                    server.sendto(packet(heartbeat()), address)
                    race = struct.pack("<BQqqIq", 1, stamp // 1000, 0, -1, 0, 0).ljust(253, b"\0")
                    server.sendto(packet(mavlink.MAVLink_encapsulated_data_message(0, race)), address)
                    if with_pose:
                        invalid = invalid_optional and bool(states)
                        pose = mavlink.MAVLink_local_position_ned_message(stamp // 1000, math.nan if invalid else 1, 2, 3, 4, 5, 6)
                        attitude = mavlink.MAVLink_attitude_message(stamp // 1000, math.nan if invalid else .1, .2, .3, 0, 0, 0)
                        motors = mavlink.MAVLink_actuator_output_status_message(stamp, 15, [.1, .2, .3, .4] + [0] * 28)
                        for observation in (pose, attitude, motors):
                            server.sendto(packet(observation), address)
                    if wire_io:
                        for observation in (
                            mavlink.MAVLink_timesync_message(234567890, 123456789),
                            mavlink.MAVLink_command_ack_message(400, 0, 100),
                            mavlink.MAVLink_collision_message(0, 1001, 0, 2, 0, 0, 3),
                            mavlink.MAVLink_odometry_message(stamp, 1, 8, 1, 2, 3, [1, 0, 0, 0],
                                                            4, 5, 6, 0, 0, 0, [0] * 21, [0] * 21),
                        ):
                            server.sendto(packet(observation), address)
                        server.sendto(packet(mavlink.MAVLink_timesync_message(0, 0), system=43), address)
                    server.sendto(packet(imu(stamp)), address)
                    chunks = [jpeg[i:i + 200] for i in range(0, len(jpeg), 200)]
                    incomplete = Camera.HEADER.pack(sequence * 2, 0, len(chunks), len(jpeg), len(chunks[0]), stamp * 1000 - 1)
                    server.sendto(incomplete + chunks[0], camera_address)
                    for i in reversed(range(len(chunks))):
                        header = Camera.HEADER.pack(sequence * 2 + 1, i, len(chunks), len(jpeg), len(chunks[i]), stamp * 1000)
                        server.sendto(header + chunks[i], camera_address)
                        server.sendto(header + chunks[i], camera_address)
                    server.sendto(incomplete + chunks[0], camera_address)
                    drain()
            except BaseException as error:
                failures.append(error)

        class Controller(BaseController):
            def update(self, state, gate_index, gates):
                if not isinstance(state, State) or gate_index != 0 or gates is not None:
                    raise AssertionError("controller did not receive the observations and active gate index")
                states.append(state)
                if wire_io and len(states) == 1:
                    assert client.target_ids == (42, 7)
                    client.mav.timesync_send(123456789, 0)
                    client.mav.set_actuator_control_target_send(1234567, 0, *client.target_ids,
                                                                 [.1, .2, .3, .4, 0, 0, 0, 0])
                    client.mav.command_long_send(*client.target_ids, 31000, 0, 0, 0, 0, 0, 0, 0, 0)
                if len(states) == 5:
                    if closed_on_exit:
                        client._socket.close()
                        raise StopIteration
                    raise KeyboardInterrupt
                return command

        controller = Controller()
        if record_frames is None:
            client = SimulatorClient(port=0, camera_port=0)
        else:
            directory = self.enterContext(tempfile.TemporaryDirectory())
            trace = Path(directory) / "camera.jsonl"
            record = self.enterContext(recording(trace, dict(recorded_frames=record_frames)))
            controller = RecordedController(controller, record, frames=record_frames)
            client = RecordedClient(record, port=0, camera_port=0)
        worker = threading.Thread(target=serve)
        worker.start()
        try:
            with self.assertRaises(OSError if closed_on_exit else KeyboardInterrupt):
                AIGPSimulator(controller, client=client).rollout(attach=True)
        finally:
            stop.set()
            worker.join(timeout=2)
            client.disconnect()
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        drain()
        self.assertEqual(len(states), 5)
        if record_frames is not None:
            updates = [json.loads(line) for line in trace.read_text().splitlines()
                       if json.loads(line)["event"] == "update"]
            self.assertEqual(len(updates), len(states))
            for actual, row in zip(states, updates):
                restored = read_state(row["state"])
                self.assertEqual(restored.gyro, actual.gyro)
                if actual.frame is None:
                    continue
                if record_frames:
                    np.testing.assert_array_equal(restored.frame.bgr, actual.frame.bgr)
                    self.assertFalse(restored.frame.bgr.flags.writeable)
                else:
                    self.assertIsNone(restored.frame)
                    self.assertEqual(row["frame_info"]["id"], actual.frame.id)
                    self.assertEqual(row["frame_info"]["shape"], [12, 16, 3])
                    self.assertNotIn('"png":', json.dumps(row))
        self.assertEqual(states[0].dt, 0)
        self.assertTrue(all(s.dt > 0 for s in states[1:]))
        self.assertTrue(any(s.frame is not None for s in states))
        self.assertTrue(all(s.frame.bgr.shape == (12, 16, 3) and s.frame.bgr.dtype == np.uint8
                            and not s.frame.bgr.flags.writeable for s in states if s.frame is not None))
        self.assertEqual(states[-1].motion is not None, with_pose and not invalid_optional)
        self.assertEqual(states[-1].attitude is not None, with_pose and not invalid_optional)
        self.assertEqual(states[-1].motors is not None, with_pose)
        self.assertEqual(states[-1].acceleration, (1, 2, 3))
        self.assertEqual(states[-1].gyro, (-4, -5, -6))
        if with_pose:
            first = states[0]
            self.assertEqual(first.motion.position, (1, 2, 3))
            self.assertEqual(first.motion.velocity, (4, 5, 6))
            for value, expected in zip((first.attitude.roll, first.attitude.pitch, first.attitude.yaw), (.1, -.2, -.3)):
                self.assertAlmostEqual(value, expected)
            self.assertEqual(first.motors.active, 15)
            self.assertEqual(len(first.motors.outputs), 32)
            for value, expected in zip(first.motors.outputs[:4], (.1, .2, .3, .4)):
                self.assertAlmostEqual(value, expected)
            for sample in (first.motion, first.attitude, first.motors):
                self.assertAlmostEqual(sample.time, first.time)
                self.assertLessEqual(sample.received_at, first.received_at)
            if invalid_optional:
                self.assertTrue(math.isnan(client.telemetry["LOCAL_POSITION_NED"].x))
                self.assertTrue(math.isnan(client.telemetry["ATTITUDE"].roll))
        raw = client.telemetry["HIGHRES_IMU"]
        self.assertEqual(bytes(raw.get_msgbuf()), packet(imu(raw.time_usec)))
        self.assertNotIn("_host_received_at", raw.to_dict())
        self.assertTrue(all(not hasattr(s, "telemetry") and not hasattr(s, "race") for s in states))
        rates = [m for m in received if m.get_type() == "SET_ATTITUDE_TARGET"]
        self.assertTrue(all(m.type_mask == 144 and m.target_system == 42 for m in rates))
        if isinstance(command, BodyRates):
            self.assertEqual(len(rates), 4 if closed_on_exit else 5)
            self.assertAlmostEqual(rates[0].thrust, .4)
        else:
            self.assertEqual(len(rates), 1)
            velocities = [m for m in received if m.get_type() == "SET_POSITION_TARGET_LOCAL_NED"]
            self.assertEqual(len(velocities), 4)
            self.assertTrue(all(m.type_mask == 3527 and m.coordinate_frame == 1 and m.target_system == 42
                                and (m.x, m.y, m.z) == (0, 0, 0) and (m.vx, m.vy, m.vz) == (-1, 0, 0)
                                for m in velocities))
        if not closed_on_exit:
            self.assertEqual(rates[-1].thrust, 0)
        if wire_io:
            requests = [m for m in received if m.get_type() == "TIMESYNC"]
            self.assertEqual([(m.tc1, m.ts1) for m in requests], [(123456789, 0)])
            outputs = [m for m in received if m.get_type() == "SET_ACTUATOR_CONTROL_TARGET"]
            self.assertEqual(len(outputs), 1)
            output, = outputs
            self.assertEqual((output.time_usec, output.group_mlx, output.target_system, output.target_component),
                             (1234567, 0, 42, 7))
            for value, expected in zip(output.controls, (.1, .2, .3, .4, 0, 0, 0, 0)):
                self.assertAlmostEqual(value, expected)
            resets = [m for m in received if m.get_type() == "COMMAND_LONG" and m.command == 31000]
            self.assertEqual([(m.target_system, m.target_component) for m in resets], [(42, 7)])
            reply = client.telemetry["TIMESYNC"]
            self.assertEqual((reply.tc1, reply.ts1), (234567890, 123456789))
            self.assertEqual(client.telemetry["COMMAND_ACK"].command, 400)
            self.assertEqual(client.telemetry["COLLISION"].id, 1001)
            self.assertEqual(client.telemetry["ODOMETRY"].x, 1)
        arms = [m.param1 for m in received if m.get_type() == "COMMAND_LONG"
                and m.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM]
        self.assertEqual(arms, [1] if closed_on_exit else [1, 0])
        self.assertFalse(client.connected)
        self.assertIsNone(client._socket)
        self.assertIsNone(client._vision)

    def test_owned_process_countdown_six_gates_finish_and_cleanup(self):
        result = self.owned_session()
        self.assertTrue(result["finish_sent"])
        self.assertGreater(result["imu_after_last_gate"], 0)

    def test_waits_for_track_input_after_go_before_arming(self):
        result = self.owned_session("--track-after-go")
        self.assertTrue(result["track_sent"])
        self.assertTrue(result["finish_sent"])

    def test_track_fragments_and_rejected_replacements_during_flight(self):
        result = self.owned_session("--fragmented-track")
        self.assertTrue(result["track_replaced"])
        self.assertTrue(result["finish_sent"])

    def test_owned_process_finish_without_final_imu(self):
        result = self.owned_session("--finish-without-imu")
        self.assertTrue(result["finish_sent"])
        self.assertEqual(result["imu_after_last_gate"], 0)

    def test_owned_process_race_packet_gap_is_not_a_sensor_failure(self):
        result = self.owned_session("--race-gap")
        self.assertGreater(result["race_packets_skipped"], 0)
        self.assertTrue(result["finish_sent"])

    def test_owned_process_waits_for_delayed_finish_after_disarming(self):
        result = self.owned_session("--finish-without-imu", "--delayed-finish")
        self.assertTrue(result["finish_sent"])
        self.assertTrue(result["disarmed_before_finish"])
        self.assertEqual(result["imu_after_last_gate"], 0)

    def test_owned_process_no_finish_remains_a_timeout(self):
        result = self.owned_session("--finish-without-imu", "--no-finish", expect_timeout=True)
        self.assertFalse(result["finish_sent"])
        self.assertTrue(result["disarmed_before_finish"])
        self.assertEqual(result["imu_after_last_gate"], 0)

    def test_recovered_imu_does_not_resume_control_during_finish_wait(self):
        result = self.owned_session("--finish-without-imu", "--delayed-finish", "--recover-imu")
        self.assertTrue(result["finish_sent"])
        self.assertTrue(result["disarmed_before_finish"])
        self.assertGreater(result["recovered_imu"], 0)
        self.assertEqual(result["positions_after_disarm"], 0)

    def owned_session(self, *fixture_args, expect_timeout=False):
        # Real child-process ownership and UDP; the child is a test fixture, not Unreal.
        directory = self.enterContext(tempfile.TemporaryDirectory())
        path = Path(directory) / "trace.jsonl"
        with recording(path) as record:
            controller = RecordedController(Gates(), record)
            client = RecordedClient(record, port=0, camera_port=None)
            children = []

            @contextmanager
            def launch(target, simulator_args=(), attach=False):
                self.assertFalse(attach)
                self.assertEqual(target, "vq1.r1")
                port = client._socket.getsockname()[1]
                child = subprocess.Popen([sys.executable, "-m", "test.aigp_fake_simulator", str(port), *fixture_args],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
                children.append(child)
                try:
                    yield child
                finally:
                    _stop_process(child)

            with patch("target.aigp.simulator.launch", side_effect=launch):
                if expect_timeout:
                    with self.assertRaisesRegex(TimeoutError, "fresh IMU.*race_finish_time_ns=-1.*active_gate_index=6"):
                        AIGPSimulator(controller, "vq1.r1", startup_timeout=8, client=client).rollout()
                else:
                    AIGPSimulator(controller, "vq1.r1", startup_timeout=8, client=client).rollout()
        self.assertGreater(replay(Gates(), path), 6)
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        count = 10 if "--fragmented-track" in fixture_args else 6
        self.assertTrue(all(len(row["gates"]) == count for row in rows if row["event"] == "update" and row["gates"]))
        self.assertEqual([row["armed"] for row in rows if row["event"] == "arm_request"], [True, False])
        self.assertTrue(any(row["event"] == "update" and row["gates"] for row in rows))
        child, = children
        output, error = child.communicate(timeout=2)
        self.assertEqual(child.returncode, 0, error)
        result = json.loads(output)
        self.assertEqual(result["too_soon"], [])
        self.assertEqual(result["gates"], list(range(1, count + 1)))
        self.assertEqual(result["arms"], [1, 0])
        self.assertGreater(result["positions"], 6)
        self.assertEqual(result["position_masks"], [3576])
        self.assertTrue(result["stopped_by_parent"])
        self.assertIsNone(client._socket)
        self.assertFalse(client.connected)
        return result


if __name__ == "__main__":
    unittest.main()
