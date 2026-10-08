"""Follow the published R1 gates using the simulator's position controller."""

import math

from common.math import Quaternion, Vector3D
from miniflight import Ned, PositionNed
from target.aigp.client import RaceStatus, TrackInfo
from target.aigp.controllers import BaseController, Gate


def gate_center(gate: Gate) -> Ned:
    """Offset the published gate base to its opening center."""
    norm = math.hypot(*gate.orientation)
    rotation = Quaternion(*(value / norm for value in gate.orientation))
    offset = rotation.rotate(Vector3D(0, 0, -gate.height / 2)).v
    return Ned(*(float(p + d) for p, d in zip(gate.position, offset)))


def gate_target(position, index: int, gates: tuple[Gate, ...]) -> Ned:
    """Choose a point one metre beyond the gate center along the approach."""
    center = gate_center(gates[index])
    direction = tuple(c - p for c, p in zip(center, position))
    distance = math.hypot(*direction)
    if distance < 0.1:
        previous = gate_center(gates[index - 1]) if index else (0.0, 0.0, 0.0)
        direction = tuple(c - p for c, p in zip(center, previous))
        distance = math.hypot(*direction)
    if distance == 0:
        raise ValueError(f"gate {index} has no approach direction")
    return Ned(*(c + d / distance for c, d in zip(center, direction)))


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.track = None
        self.telemetry = {}
        self.race = None
        self.gate = None
        self.target = None

    def update(self, telemetry, frames) -> PositionNed | None:
        for packet in telemetry:
            self.telemetry[packet.data.get_type()] = packet
            if isinstance(packet.decoded, RaceStatus):
                self.race = packet.decoded
            elif isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
        pose, imu = self.telemetry.get("LOCAL_POSITION_NED"), self.telemetry.get("HIGHRES_IMU")
        if pose is None or imu is None or self.race is None or not self.race.started or not usable_track(self.track):
            return None
        motion = pose.data
        position = Ned(motion.x, motion.y, motion.z)
        if imu.received_at - pose.received_at > 1 or not all(math.isfinite(v) for v in (*position, motion.vx, motion.vy, motion.vz)):
            return None
        gates, gate_index = self.track, self.race.active_gate_index
        if not 0 <= gate_index <= len(gates):
            raise ValueError(f"gate index {gate_index} does not belong to the published track")
        if gate_index < len(gates) and gates[gate_index] != self.gate:
            self.target = PositionNed(*gate_target(position, gate_index, gates))
            self.gate = gates[gate_index]

        return self.target


def usable_track(gates):
    return bool(gates) and all(gate.id == i and gate.width > 0 and gate.height > 0
                              and all(math.isfinite(v) for v in (*gate.position, *gate.orientation, gate.width, gate.height))
                              and .99 <= math.hypot(*gate.orientation) <= 1.01 for i, gate in enumerate(gates))
