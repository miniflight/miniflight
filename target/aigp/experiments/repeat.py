"""Reset the owned native race, then transfer its fresh observation cursor."""

from dataclasses import asdict
import math
import time

from target.aigp.client import SimulatorClient
from target.aigp.native import RaceStateReader


def fresh_race(path, timeout=120, course_packets=()):
    reader, client = RaceStateReader(path, .3), SimulatorClient(camera_port=None)
    requested, accepted, fresh, before, course = None, False, None, None, []
    deadline, heartbeat_at = time.monotonic() + timeout, 0
    client.open()
    try:
        while time.monotonic() < deadline:
            packets, states = client.poll(.02), reader.poll()
            course.extend(packet for packet in packets if packet.data.get_type() == "DATA_TRANSMISSION_HANDSHAKE"
                          or packet.data.get_type() == "ENCAPSULATED_DATA" and packet.data.data[0] == 2)
            now = time.monotonic()
            if client.connected and now >= heartbeat_at:
                client.heartbeat()
                heartbeat_at = now + .5
            if requested is None and client.connected and states:
                before = asdict(states[-1])  # Drain old observations before requesting reset.
                client.reset()
                requested = time.monotonic()
                continue
            if requested is None:
                continue
            for packet in packets:
                message = packet.data
                if message.get_type() == "COMMAND_ACK" and message.command == 31000:
                    if message.result != 0:
                        raise RuntimeError("native reset request was rejected")
                    accepted = True
            for state in states:
                if not state.started and state.valid and not state.completed and state.active_gate_index == 0 and state.time_seconds == 0:
                    fresh = asdict(state)
            pose = client.telemetry.get("LOCAL_POSITION_NED")
            if (accepted and fresh is not None and pose is not None and pose._host_received_at > requested
                    and (course_packets or client.gates is not None)):
                position, velocity = (pose.x, pose.y, pose.z), (pose.vx, pose.vy, pose.vz)
                if math.hypot(*position) < .1 and math.hypot(*velocity) < .1:
                    course = tuple(course) if client.gates is not None else course_packets
                    return reader, dict(before=before, after=fresh, position_ned_m=position, velocity_ned_m_s=velocity), course
        raise TimeoutError("native reset did not expose a fresh stationary starting grid")
    except BaseException:
        reader.close()
        raise
    finally:
        client.disconnect()
