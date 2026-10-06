"""Real UDP controller checks on ephemeral ports, without an Unreal process."""

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
            with self.subTest(controller=make.__name__, module=make.__module__):
                directory = self.enterContext(tempfile.TemporaryDirectory())
                path = Path(directory) / "trace.jsonl"
                stop = threading.Event()
                failures, received = [], []
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, recording(path) as record:
                    server.bind(("127.0.0.1", 0))
                    server.setblocking(False)
                    client = RecordedClient(record, port=0, camera_port=None)
                    encoder = mavlink.MAVLink(None, srcSystem=42, srcComponent=7)
                    # Identity rotation and height 2 put the opening center at the drone's origin.
                    track = struct.pack("<H", 1) + struct.pack("<H9f", 0, 0, 0, 1, 1, 0, 0, 0, 2, 2)

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
                                sequence += 1
                                stamp = sequence * 5000
                                race = struct.pack("<BQqqIq", 1, stamp // 1000, 0, -1, 0, 0).ljust(253, b"\0")
                                messages = (
                                    mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 4, 3),
                                    mavlink.MAVLink_data_transmission_handshake_message(0, len(track), 7, 0, 1, 253, 1),
                                    mavlink.MAVLink_encapsulated_data_message(0, (struct.pack("<BH", 2, 7) + track).ljust(253, b"\0")),
                                    mavlink.MAVLink_encapsulated_data_message(0, race),
                                    mavlink.MAVLink_local_position_ned_message(stamp // 1000, 0, 0, 0, 0, 0, 0),
                                    mavlink.MAVLink_attitude_message(stamp // 1000, 0, 0, 0, 0, 0, 0),
                                    mavlink.MAVLink_highres_imu_message(stamp, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xffff),
                                )
                                for message in messages:
                                    server.sendto(message.pack(encoder), peer)
                        except BaseException as error:
                            failures.append(error)

                    worker = threading.Thread(target=serve)
                    worker.start()
                    try:
                        with self.assertRaisesRegex(ValueError, "gate 0 has no approach direction"):
                            AIGPSimulator(RecordedController(make(), record), startup_timeout=3, client=client).rollout(attach=True)
                    finally:
                        stop.set()
                        worker.join(timeout=2)
                        client.disconnect()
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(failures, [])
                    while True:
                        try:
                            packet, _ = server.recvfrom(65536)
                        except BlockingIOError:
                            break
                        received.extend(mavlink.MAVLink(None).parse_buffer(packet) or ())
                    self.assertFalse(any(message.get_type() in ("COMMAND_LONG", "SET_ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED")
                                         for message in received))
                    self.assertIsNone(client._socket)
                    self.assertFalse(client.connected)
                self.assertEqual(replay(make(), path), 1)


if __name__ == "__main__":
    unittest.main()
