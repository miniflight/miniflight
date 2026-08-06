"""Read the metric pose and gate centers published by AIGP VQ1."""

from collections import Counter
import math
from pathlib import Path
import struct
import time

from pymavlink import mavutil


VERSION = Path(__file__).with_name("VERSION").read_text().strip()
CHECK_SECONDS = 5.0
HEARTBEAT_TIMEOUT_S = 10.0
TRACK_PACKET_ID = 2
TRACK_GATE = struct.Struct("<Hfffffffff")
REQUESTS = (
    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 60),
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 60),
    (getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ODOMETRY", 331), 60),
)


class TrackTransfer:
    """Reassemble the official one-time VQ1 gate transfer."""

    def __init__(self):
        self.chunks = {}
        self.expected = {}
        self.gates = []

    def add(self, message):
        kind = message.get_type()
        if kind == "DATA_TRANSMISSION_HANDSHAKE":
            transfer_id = int(message.width)
            self.chunks[transfer_id] = {}
            self.expected[transfer_id] = int(message.packets)
            return
        if kind != "ENCAPSULATED_DATA":
            return
        payload = bytes(message.data)
        if len(payload) < 3 or payload[0] != TRACK_PACKET_ID:
            return
        _, transfer_id = struct.unpack_from("<BH", payload)
        if transfer_id not in self.expected:
            return
        self.chunks[transfer_id][int(message.seqnr)] = payload[3:]
        if len(self.chunks[transfer_id]) != self.expected[transfer_id]:
            return
        joined = b"".join(
            self.chunks[transfer_id][index]
            for index in range(self.expected[transfer_id])
        )
        del self.chunks[transfer_id]
        del self.expected[transfer_id]
        self.gates = parse_track(joined)


def parse_track(payload):
    if len(payload) < 2:
        return []
    gate_count, = struct.unpack_from("<H", payload)
    offset = 2
    gates = []
    for _ in range(gate_count):
        if offset + TRACK_GATE.size > len(payload):
            return []
        values = TRACK_GATE.unpack_from(payload, offset)
        offset += TRACK_GATE.size
        gate_id = values[0]
        position = values[1:4]
        orientation = values[4:8]
        width, height = values[8:10]
        finite = all(math.isfinite(value) for value in values[1:])
        quaternion_norm = math.sqrt(sum(value * value for value in orientation))
        valid = finite and width > 0.0 and height > 0.0 and 0.99 <= quaternion_norm <= 1.01
        gates.append((gate_id, position, orientation, width, height, valid))
    gate_ids = {gate[0] for gate in gates}
    if len(gate_ids) != gate_count:
        return []
    return gates


def request(link, message_id, rate_hz):
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        1_000_000 / rate_hz,
        0,
        0,
        0,
        0,
        0,
    )


def run():
    link = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    counts = Counter()
    track = TrackTransfer()
    heartbeat = None
    deadline = time.monotonic() + HEARTBEAT_TIMEOUT_S
    while heartbeat is None and time.monotonic() < deadline:
        message = link.recv_match(blocking=False)
        if message is None:
            time.sleep(0.001)
            continue
        kind = message.get_type()
        counts[kind] += 1
        track.add(message)
        if kind == "HEARTBEAT":
            heartbeat = message
    if heartbeat is None:
        raise RuntimeError("no simulator heartbeat")
    for message_id, rate_hz in REQUESTS:
        request(link, message_id, rate_hz)

    deadline = time.monotonic() + CHECK_SECONDS
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=False)
        if message is None:
            time.sleep(0.001)
            continue
        counts[message.get_type()] += 1
        track.add(message)

    odometry = counts["ODOMETRY"] > 0
    split_state = counts["LOCAL_POSITION_NED"] > 0 and counts["ATTITUDE"] > 0
    metric_state = odometry or split_state
    valid_gates = bool(track.gates) and all(gate[-1] for gate in track.gates)
    ready = metric_state and valid_gates
    print(f"patch v{VERSION} {'PASS' if ready else 'BLOCKED'}")
    print("telemetry " + " ".join(f"{name}={count}" for name, count in sorted(counts.items())))
    print(f"metric_state={metric_state} valid_gate_centers={len(track.gates) if valid_gates else 0}")
    for gate_id, position, _orientation, width, height, valid in track.gates:
        print(
            f"gate {gate_id} center_ned_m "
            f"{position[0]:.3f} {position[1]:.3f} {position[2]:.3f} "
            f"size_m {width:.3f} {height:.3f} valid={valid}"
        )
    if not metric_state:
        print("missing metric position velocity and attitude")
    if not valid_gates:
        print("missing valid gate centers")
    if not ready:
        print("flight not armed")
        raise SystemExit(1)


if __name__ == "__main__":
    run()
