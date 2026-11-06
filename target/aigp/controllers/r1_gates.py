"""Follow the published R1 gates using the simulator's position controller."""

import math
from enum import Enum, auto

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


class GatePhase(Enum):
    WAITING_FOR_COURSE = auto()
    WAITING_FOR_GATE = auto()
    FOLLOWING_GATE = auto()
    COURSE_COMPLETE = auto()


class GateFollower:
    def __init__(self, exit_distance=1.0):
        self.phase = GatePhase.WAITING_FOR_COURSE
        self.exit_distance = exit_distance
        self.track = None
        self.gate = None
        self.target = None

    def observe_course(self, telemetry):
        for packet in telemetry:
            if isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
                if self.phase is GatePhase.WAITING_FOR_COURSE:
                    self.phase = GatePhase.WAITING_FOR_GATE

    def has_course_for(self, race_state):
        return self.phase is not GatePhase.WAITING_FOR_COURSE and len(race_state.gate_centers) == len(self.track)

    def select_target(self, position, race_state):
        index, centers = race_state.active_gate_index, race_state.gate_centers
        if index == len(self.track):
            self.phase = GatePhase.COURSE_COMPLETE
            return self.target
        gate = (index, centers[index])
        if gate != self.gate:
            self.target = gate_target(position, index, centers, self.exit_distance)
            self.gate = gate
        self.phase = GatePhase.FOLLOWING_GATE
        return self.target


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.follower = GateFollower()

    def update(self, telemetry, frames, race_state) -> PositionNed | None:
        self.follower.observe_course(telemetry)
        state = vehicle.state
        if (state is None or state.motion is None or state.received_at is None or race_state is None
                or not race_state.started or not self.follower.has_course_for(race_state)):
            return None
        motion = state.motion
        position = Vector3D(*motion.position)
        if state.received_at - motion.received_at > 1 or not all(math.isfinite(v) for v in (*position.v, *motion.velocity)):
            return None
        gates, gate_index = self.follower.track, race_state.active_gate_index
        if not 0 <= gate_index <= len(gates):
            raise ValueError(f"gate index {gate_index} does not belong to the published track")
        target = self.follower.select_target(position, race_state)
        return None if target is None else PositionNed(*target.v)


def usable_track(gates):
    return bool(gates) and all(gate.id == i and gate.width > 0 and gate.height > 0
                              and all(math.isfinite(v) for v in (*gate.position, *gate.orientation, gate.width, gate.height))
                              and .99 <= math.hypot(*gate.orientation) <= 1.01 for i, gate in enumerate(gates))
