"""Follow the published R1 gates using the simulator's position controller."""

import math

from common.math import Vector3D
from miniflight import Ned, PositionNed, vehicle
from target.aigp.client import TrackInfo
from target.aigp.controllers import BaseController


def gate_target(position: Vector3D, index: int, centers: tuple[Ned, ...], exit_distance=1.0) -> Vector3D:
    """Choose a point beyond the native gate center along the approach."""
    center = Vector3D(*centers[index])
    direction = center - position
    distance = math.hypot(*direction.v)
    if distance < 0.1:
        previous = Vector3D(*centers[index - 1]) if index else Vector3D()
        direction = center - previous
        distance = math.hypot(*direction.v)
    if distance < 0.1:
        raise ValueError(f"gate {index} has no approach direction")
    return center + exit_distance * direction / distance


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.track = None
        self.gate = None
        self.target = None

    def update(self, telemetry, frames, race_state) -> PositionNed | None:
        for packet in telemetry:
            if isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
        state = vehicle.state
        if (state is None or state.motion is None or state.received_at is None or race_state is None or not race_state.started or self.track is None
                or len(race_state.gate_centers) != len(self.track)):
            return None
        motion = state.motion
        position = Vector3D(*motion.position)
        if state.received_at - motion.received_at > 1 or not all(math.isfinite(v) for v in (*position.v, *motion.velocity)):
            return None
        gates, gate_index = self.track, race_state.active_gate_index
        if not 0 <= gate_index <= len(gates):
            raise ValueError(f"gate index {gate_index} does not belong to the published track")
        if gate_index < len(gates) and (gate_index, race_state.gate_centers[gate_index]) != self.gate:
            self.target = PositionNed(*gate_target(position, gate_index, race_state.gate_centers).v)
            self.gate = (gate_index, race_state.gate_centers[gate_index])

        return self.target


def usable_track(gates):
    return bool(gates) and all(gate.id == i and gate.width > 0 and gate.height > 0
                              and all(math.isfinite(v) for v in (*gate.position, *gate.orientation, gate.width, gate.height))
                              and .99 <= math.hypot(*gate.orientation) <= 1.01 for i, gate in enumerate(gates))
