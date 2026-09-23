"""Bounded loopback fixture for session tests. No Unreal process or real flight model."""

import json
import math
import signal
import socket
import struct
import sys
import time

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.controllers.r1_gates import GATES


def main():
    peer = ("127.0.0.1", int(sys.argv[1]))
    finish_without_imu = "--finish-without-imu" in sys.argv[2:]
    no_finish = "--no-finish" in sys.argv[2:]
    race_gap = "--race-gap" in sys.argv[2:]
    delayed_finish = "--delayed-finish" in sys.argv[2:]
    result = {"too_soon": [], "gates": [], "arms": [], "positions": 0,
              "position_masks": [], "stopped_by_parent": False,
              "finish_sent": False, "imu_after_last_gate": 0,
              "disarmed_before_finish": False, "race_packets_skipped": 0}

    def stop(signum, frame):
        result["stopped_by_parent"] = True

    signal.signal(signal.SIGTERM, stop)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        sock.setblocking(False)
        decoder = mavlink.MAVLink(None)
        encoder = mavlink.MAVLink(None, srcSystem=42, srcComponent=7)
        position, target, index = (0.0, 0.0, 0.0), None, 0
        armed, boot, start, count = False, 0, -1, 0
        started = time.monotonic()
        gap_started = ended_at = None

        def send(message):
            sock.sendto(message.pack(encoder), peer)

        def receive():
            nonlocal target, armed
            while True:
                try:
                    packet, _ = sock.recvfrom(65536)
                except BlockingIOError:
                    return
                for message in decoder.parse_buffer(packet) or ():
                    kind = message.get_type()
                    if kind == "COMMAND_LONG" and message.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        armed = bool(message.param1)
                        result["arms"].append(int(armed))
                        if not armed and index == len(GATES) and not result["finish_sent"]:
                            result["disarmed_before_finish"] = True
                        if armed and (start < 0 or boot < start):
                            result["too_soon"].append("arm")
                    elif kind == "SET_POSITION_TARGET_LOCAL_NED":
                        target = (message.x, message.y, message.z)
                        result["positions"] += 1
                        if message.type_mask not in result["position_masks"]:
                            result["position_masks"].append(message.type_mask)
                        if start < 0 or boot < start:
                            result["too_soon"].append("position")

        while not result["stopped_by_parent"] and time.monotonic() - started < 12:
            boot = int((time.monotonic() - started) * 1000)
            start = -1 if boot < 100 else 500
            receive()
            if target is not None and armed and index < len(GATES):
                distance = math.dist(position, target)
                scale = min(1, .5 / distance) if distance else 0
                next_position = tuple(p + (t - p) * scale for p, t in zip(position, target))
                center = GATES[index]
                if position[0] >= center[0] > next_position[0]:
                    fraction = (center[0] - position[0]) / (next_position[0] - position[0])
                    crossing = tuple(p + (n - p) * fraction for p, n in zip(position, next_position))
                    if math.dist(crossing, center) < .75:
                        index += 1
                        result["gates"].append(index)
                position = next_position
            count += 1
            send(mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 4, 3))
            at_end = index == len(GATES)
            now = time.monotonic()
            if at_end and ended_at is None:
                ended_at = now
            if index >= 1 and gap_started is None:
                gap_started = now
            finish_ready = at_end and (not delayed_finish or now - ended_at >= 2.5)
            finish = 1000000000 if finish_ready and not no_finish else -1
            result["finish_sent"] |= finish >= 0
            data = struct.pack("<BQqqIq", 1, boot, start, finish, index, 0).ljust(253, b"\0")
            if race_gap and gap_started is not None and now - gap_started < 3:
                result["race_packets_skipped"] += 1
            else:
                send(mavlink.MAVLink_encapsulated_data_message(0, data))
            if not at_end or not finish_without_imu:
                result["imu_after_last_gate"] += int(at_end)
                send(mavlink.MAVLink_local_position_ned_message(boot, *position, 0, 0, 0))
                send(mavlink.MAVLink_highres_imu_message(
                    count * 5000, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xffff,
                ))
            time.sleep(.005)
        receive()  # Includes the runner's final zero/disarm before SIGTERM.
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
