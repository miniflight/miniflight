"""Real UDP round-trip using ephemeral loopback ports, never the simulator ports.

Run explicitly: python -m unittest test.aigp_udp_smoke -v
"""

import socket
import struct
import subprocess
import sys
import threading
import unittest
import json
from unittest.mock import patch

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from sim.aigp._runtime.controller_runner import run, run_session
from controllers.r1_gates import Controller as Gates
from miniflight import Control
from target.aigp import SimulatorClient, _Camera as Camera
from test.test_aigp_client import heartbeat, imu, packet


class UDPSmokeTest(unittest.TestCase):
    def test_shared_runner_with_and_without_pose_telemetry(self):
        for with_pose in (True, False):
            with self.subTest(with_pose=with_pose):
                self.round_trip(with_pose)

    def round_trip(self, with_pose):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(server.close)
        server.bind(("127.0.0.1", 0))
        server.setblocking(False)
        sim = SimulatorClient(port=0, camera_port=0)
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
                    sock, vision = sim._socket, sim._vision
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
                        pose = mavlink.MAVLink_local_position_ned_message(stamp // 1000, 1, 2, 3, 4, 5, 6)
                        server.sendto(packet(pose), address)
                    server.sendto(packet(imu(stamp)), address)
                    header = Camera.HEADER.pack(sequence, 0, 1, len(jpeg), len(jpeg), stamp * 1000)
                    server.sendto(header + jpeg, camera_address)
                    drain()
            except BaseException as error:
                failures.append(error)

        class Controller:
            def update(self, state):
                states.append(state)
                if len(states) == 5:
                    raise KeyboardInterrupt
                return Control(.1, -.2, .3, .4)

        worker = threading.Thread(target=serve)
        worker.start()
        try:
            with self.assertRaises(KeyboardInterrupt):
                run(Controller(), sim)
        finally:
            stop.set()
            worker.join(timeout=2)
            sim.disconnect()
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        drain()
        self.assertEqual(len(states), 5)
        self.assertEqual(states[0].dt, 0)
        self.assertTrue(all(s.dt > 0 for s in states[1:]))
        self.assertTrue(any(s.frame is not None for s in states))
        self.assertEqual("LOCAL_POSITION_NED" in states[-1].telemetry, with_pose)
        rates = [m for m in received if m.get_type() == "SET_ATTITUDE_TARGET"]
        self.assertEqual(len(rates), 5)
        self.assertTrue(all(m.type_mask == 144 and m.target_system == 42 for m in rates))
        self.assertAlmostEqual(rates[0].thrust, .4)
        self.assertEqual(rates[-1].thrust, 0)
        arms = [m.param1 for m in received if m.get_type() == "COMMAND_LONG"]
        self.assertEqual(arms, [1, 0])

    def test_owned_process_countdown_six_gates_finish_and_cleanup(self):
        result = self.owned_session()
        self.assertTrue(result["finish_sent"])
        self.assertGreater(result["imu_after_last_gate"], 0)

    def test_owned_process_finish_without_final_imu(self):
        result = self.owned_session("--finish-without-imu")
        self.assertTrue(result["finish_sent"])
        self.assertEqual(result["imu_after_last_gate"], 0)

    def test_owned_process_no_finish_remains_a_timeout(self):
        result = self.owned_session("--finish-without-imu", "--no-finish", expect_timeout=True)
        self.assertFalse(result["finish_sent"])
        self.assertEqual(result["imu_after_last_gate"], 0)

    def owned_session(self, *fixture_args, expect_timeout=False):
        # Real child-process ownership and UDP; the child is a test fixture, not Unreal.
        sim = SimulatorClient(port=0, camera_port=None)
        popen = subprocess.Popen
        children = []

        def launch(command, **kwargs):
            self.assertTrue(command[0].endswith("run_vq1.sh"))
            port = sim._socket.getsockname()[1]
            child = popen([sys.executable, "-m", "test.aigp_fake_simulator", str(port), *fixture_args],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, **kwargs)
            children.append(child)
            return child

        with patch("sim.aigp._runtime.controller_runner.SimulatorClient", return_value=sim), \
                patch("sim.aigp._runtime.controller_runner._check_simulator_ports"), \
                patch("sim.aigp._runtime.controller_runner.subprocess.Popen", side_effect=launch):
            if expect_timeout:
                with self.assertRaisesRegex(TimeoutError, "fresh IMU.*gate_index=6.*finish_ns=-1"):
                    run_session(Gates(), "vq1.r1", startup_timeout=8)
            else:
                run_session(Gates(), "vq1.r1", startup_timeout=8)
        child, = children
        output, error = child.communicate(timeout=2)
        self.assertEqual(child.returncode, 0, error)
        result = json.loads(output)
        self.assertEqual(result["too_soon"], [])
        self.assertEqual(result["gates"], [1, 2, 3, 4, 5, 6])
        self.assertEqual(result["arms"], [1, 0])
        self.assertGreater(result["positions"], 6)
        self.assertEqual(result["position_masks"], [3576])
        self.assertTrue(result["stopped_by_parent"])
        self.assertIsNone(sim._socket)
        return result


if __name__ == "__main__":
    unittest.main()
