"""Follow the published R1 gates using the simulator's position controller."""

import math

from common.math import Quaternion, Vector3D
from miniflight import PositionNed
from target.aigp.client import TrackInfo
from target.aigp.controllers import BaseController, Gate


def gate_center(gate: Gate) -> Vector3D:
    """Offset the published gate base to its opening center."""
    norm = math.hypot(*gate.orientation)
    rotation = Quaternion(*(value / norm for value in gate.orientation))
    return Vector3D(*gate.position) + rotation.rotate(Vector3D(0, 0, -gate.height / 2))


def gate_target(position: Vector3D, index: int, gates: tuple[Gate, ...], exit_distance=1.0) -> Vector3D:
    """Choose a point beyond the gate center along the approach."""
    center = gate_center(gates[index])
    direction = center - position
    distance = math.hypot(*direction.v)
    if distance < 0.1:
        previous = gate_center(gates[index - 1]) if index else Vector3D()
        direction = center - previous
        distance = math.hypot(*direction.v)
    if distance == 0:
        raise ValueError(f"gate {index} has no approach direction")
    return center + exit_distance * direction / distance


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.track = None
        self.telemetry = {}
        self.gate = None
        self.target = None

    def update(self, telemetry, frames, race_state) -> PositionNed | None:
        for packet in telemetry:
            self.telemetry[packet.data.get_type()] = packet
            if isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
        pose, imu = self.telemetry.get("LOCAL_POSITION_NED"), self.telemetry.get("HIGHRES_IMU")
        if pose is None or imu is None or race_state is None or not race_state.started or not usable_track(self.track):
            return None
        motion = pose.data
        position = Vector3D(motion.x, motion.y, motion.z)
        if imu.received_at - pose.received_at > 1 or not all(math.isfinite(v) for v in (*position.v, motion.vx, motion.vy, motion.vz)):
            return None
        gates, gate_index = self.track, race_state.active_gate_index
        if not 0 <= gate_index <= len(gates):
            raise ValueError(f"gate index {gate_index} does not belong to the published track")
        if gate_index < len(gates) and gates[gate_index] != self.gate:
            self.target = PositionNed(*gate_target(position, gate_index, gates).v)
            self.gate = gates[gate_index]

        return self.target


def usable_track(gates):
    return bool(gates) and all(gate.id == i and gate.width > 0 and gate.height > 0
                              and all(math.isfinite(v) for v in (*gate.position, *gate.orientation, gate.width, gate.height))
                              and .99 <= math.hypot(*gate.orientation) <= 1.01 for i, gate in enumerate(gates))
