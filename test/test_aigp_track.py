from dataclasses import FrozenInstanceError
import json
import math
from pathlib import Path
import struct
import unittest

from pymavlink.dialects.v20 import common as mavlink

from miniflight import Ned
from target.aigp.simulator import _Track


def course(count=8, north=10.0):
    return struct.pack("<H", count) + b"".join(
        struct.pack("<H9f", i, north + 10 * i, 20, 30, 1, 0, 0, 0, 2, 4)
        for i in range(count)
    )


def handshake(data, transfer_id=7):
    return mavlink.MAVLink_data_transmission_handshake_message(
        0, len(data), transfer_id, 0, (len(data) + 249) // 250, 253, 1)


def packets(data, transfer_id=7):
    return [mavlink.MAVLink_encapsulated_data_message(
        i // 250, (struct.pack("<BH", 2, transfer_id) + data[i:i + 250]).ljust(253, b"\0"))
        for i in range(0, len(data), 250)]


class TrackTest(unittest.TestCase):
    def setUp(self):
        self.track = _Track()

    def deliver(self, data, transfer_id=7, now=10):
        self.track.start(handshake(data, transfer_id), now)
        for packet in packets(data, transfer_id):
            self.track.receive(packet, now)

    def test_captured_vq1_packet_reconstructs_the_verified_gate_centers(self):
        fixture = json.loads((Path(__file__).parent / "fixtures/vq1_track.json").read_text())
        fields = {key: value for key, value in fixture["handshake"].items() if key != "mavpackettype"}
        self.track.start(mavlink.MAVLink_data_transmission_handshake_message(**fields), 10)
        for packet in fixture["chunks"]:
            self.track.receive(mavlink.MAVLink_encapsulated_data_message(
                packet["seqnr"], bytes.fromhex(packet["data_hex"])), 10)
        self.assertEqual(len(self.track.gates), len(fixture["expected_centers"]))
        for index, (gate, expected) in enumerate(zip(self.track.gates, fixture["expected_centers"])):
            self.assertEqual(gate.id, index)
            self.assertLess(math.dist(gate.center, expected), 1e-10)
            self.assertIsInstance(gate.origin, Ned)
            self.assertAlmostEqual(math.hypot(*gate.orientation), 1, places=14)
            self.assertLess(math.dist(gate.orientation, (
                .7071067513842201, 0, -8.657316443846768e-8, .7071068109888684)), 1e-14)
            self.assertAlmostEqual(gate.width, 2.72, places=6)
            self.assertAlmostEqual(gate.height, 2.72, places=6)
        self.assertEqual(self.track.gates[0].origin,
                         Ned(-23.2979679107666, -.39990234375, -.03195800632238388))
        with self.assertRaises(FrozenInstanceError):
            self.track.gates[0].center = (0, 0, 0)
        with self.assertRaises(FrozenInstanceError):
            self.track.gates[0].origin = Ned(0, 0, 0)

    def test_rotated_gate_preserves_origin_and_normalizes_orientation(self):
        q = .995 * math.sqrt(.5)
        data = struct.pack("<H", 1) + struct.pack("<H9f", 0, 10, 20, 30, q, 0, q, 0, 2, 4)
        self.deliver(data)
        gate = self.track.gates[0]
        self.assertEqual(gate.origin, Ned(10, 20, 30))
        self.assertLess(math.dist(gate.orientation, (math.sqrt(.5), 0, math.sqrt(.5), 0)), 1e-14)
        self.assertLess(math.dist(gate.center, (8, 20, 30)), 1e-6)

    def test_chunks_can_arrive_out_of_order_with_duplicates(self):
        data = course()
        first, last = packets(data)
        self.track.start(handshake(data), 10)
        self.track.receive(last, 10)
        self.track.receive(last, 10.1)
        self.track.start(handshake(data), 10.2)
        self.assertIsNone(self.track.gates)
        self.track.receive(first, 10.3)
        self.assertEqual(len(self.track.gates), 8)
        self.assertEqual(self.track.gates[0].center, (10, 20, 28))

    def test_partial_replacement_keeps_the_last_complete_course(self):
        self.deliver(course(1))
        original = self.track.gates
        replacement = course(north=100)
        self.track.start(handshake(replacement, 8), 11)
        first, last = packets(replacement, 8)
        self.track.receive(first, 11)
        self.assertIs(self.track.gates, original)
        self.track.receive(last, 11.1)
        self.assertEqual(self.track.gates[0].center, (100, 20, 28))

    def test_interleaved_transfers_do_not_mix_courses(self):
        first, second = course(north=10), course(north=100)
        self.track.start(handshake(first, 7), 10)
        self.track.start(handshake(second, 8), 10)
        a, b = packets(first, 7), packets(second, 8)
        self.track.receive(a[0], 10)
        self.track.receive(b[1], 10)
        self.assertIsNone(self.track.gates)
        self.track.receive(a[1], 10)
        self.assertEqual(self.track.gates[0].center, (10, 20, 28))
        self.track.receive(b[0], 10)
        self.assertEqual(self.track.gates[0].center, (100, 20, 28))

    def test_unknown_expired_or_out_of_range_chunks_do_not_publish(self):
        data = course()
        first, last = packets(data)
        self.track.receive(first, 10)
        self.assertIsNone(self.track.gates)
        self.track.start(handshake(data), 10)
        first.seqnr = 2
        self.track.receive(first, 10)
        first.seqnr = 0
        self.track.receive(last, 10)
        self.assertIsNone(self.track.gates)
        self.track.receive(first, 16)
        self.assertIsNone(self.track.gates)
        self.assertEqual(self.track.pending, {})

    def test_pending_transfers_and_sizes_are_bounded(self):
        for transfer_id in range(20):
            self.track.start(handshake(course(), transfer_id), 10)
        self.assertEqual(len(self.track.pending), self.track.MAX_TRANSFERS)
        bad = handshake(course())
        bad.size = 1_000_000
        self.track.start(bad, 10)
        self.assertNotIn(7, self.track.pending)
        bad.size, bad.packets = 230, 2
        self.track.start(bad, 10)
        self.assertNotIn(7, self.track.pending)

    def test_missing_or_invalid_geometry_is_unavailable(self):
        valid = (0, 10, 20, 30, 1, 0, 0, 0, 2, 4)
        records = [
            (0,) + (0,) * 9,
            (0, math.nan, *valid[2:]),
            (*valid[:4], 0, 0, 0, 0, 2, 4),
            (*valid[:8], 0, 4),
            (1, *valid[1:]),
        ]
        for row in records:
            with self.subTest(row=row):
                self.deliver(course(1))
                self.assertIsNotNone(self.track.gates)
                self.deliver(struct.pack("<H", 1) + struct.pack("<H9f", *row))
                self.assertIsNone(self.track.gates)
        self.assertIsNone(_Track.decode(b""))
        self.assertIsNone(_Track.decode(struct.pack("<H", 2) + struct.pack("<H9f", *valid)))


if __name__ == "__main__":
    unittest.main()
