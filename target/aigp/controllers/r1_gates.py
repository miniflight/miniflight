"""Six-gate R1 baseline using the simulator's built-in position controller."""

import math
import time

from miniflight import PositionNed, State
from target.aigp.controllers import BaseController


# R1 NED gate centers, retained from examples/aigp/thread_gates.py.
GATES = (
    (-23.297967744257804, -0.3999021772411884, -1.3919580206274782),
    (-46.89374907055175, -2.499990058329445, 3.7080417871475424),
    (-74.5937498334912, 1.200009870144981, 12.308041214942953),
    (-111.49374372997558, -5.099989724543434, 23.208040833473227),
    (-135.4937437299756, -0.7999900702503742, 23.99565374851229),
    (-159.19374067821778, -4.399989915278297, 24.6080404520035),
)


class Controller(BaseController):
    targets = ("vq1.r1",)

    def __init__(self, port=14550, camera_port=5600):
        super().__init__(port=port, camera_port=camera_port)
        self.gate = None
        self.target = None
        self.started_at = None
        self.gate_started_at = None

    def update(self, state: State) -> PositionNed | None:
        if self.started_at is None:
            self.started_at = state.time
        motion, race = state.motion, self.race
        if motion is None or race is None:
            if self.gate is not None or state.time - self.started_at >= 10:
                raise ValueError("r1_gates needs VQ1 position and race telemetry")
            return None  # Wait for startup telemetry without arming.

        now = time.monotonic()
        if now - motion.received_at > 1:
            raise TimeoutError("R1 position telemetry is stale")
        position = motion.position
        if not all(math.isfinite(v) for v in position):
            raise ValueError("R1 position telemetry must be finite")

        index = race.active_gate_index
        if not 0 <= index <= len(GATES):
            raise ValueError(f"gate index {index} does not belong to the six-gate R1 course")
        if self.gate is not None and index < self.gate:
            raise ValueError("race reset during control; start a new run")
        if race.race_finish_time_ns >= 0:
            print("r1_gates: finished", flush=True)
            raise StopIteration
        if race.race_start_boot_time_ms < 0 or race.sim_boot_time_ms < race.race_start_boot_time_ms:
            return None
        if index == len(GATES):
            return self.target  # Hold the final target until the native finish signal.

        if index != self.gate:
            center = GATES[index]
            direction = tuple(c - p for c, p in zip(center, position))
            distance = math.hypot(*direction)
            if distance < 0.1:
                previous = GATES[index - 1] if index else (0.0, 0.0, 0.0)
                direction = tuple(c - p for c, p in zip(center, previous))
                distance = math.hypot(*direction)
            # A fixed target one metre beyond the center crosses the gate instead
            # of stopping in its plane. Only the reported gate index advances us.
            self.target = PositionNed(*(c + d / distance for c, d in zip(center, direction)))
            self.gate, self.gate_started_at = index, state.time
            print(f"r1_gates: gate {index + 1}/{len(GATES)}", flush=True)
        if state.time - self.gate_started_at > 45:
            raise TimeoutError(f"gate {index + 1} was not passed within 45 simulator seconds")
        return self.target
