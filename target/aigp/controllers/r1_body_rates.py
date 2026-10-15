"""Follow the published VQ1 gates with Python position feedback and body-rate commands."""

import math

from miniflight import BodyRates, vehicle
from miniflight.position import PositionConfig, position_control
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import Controller as GatePlanner


# Local hover calibration measured on VQ1 build 3391; see docs/body-rates.md.
CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, config: PositionConfig = CONFIG):
        self.config = config
        self.course = GatePlanner()
        self.target_yaw = None

    def update(self, telemetry, frames, race_state) -> BodyRates | None:
        target = self.course.update(telemetry, frames, race_state)
        state = vehicle.state
        if target is None or state is None or state.attitude is None or state.received_at is None or state.received_at - state.attitude.received_at > 1:
            return None
        motion, attitude = state.motion, state.attitude
        angles = (attitude.roll, attitude.pitch, attitude.yaw)
        if not all(math.isfinite(v) for v in angles):
            return None

        # Keep the initial heading throughout the course, including gate changes.
        if self.target_yaw is None:
            self.target_yaw = angles[2]

        return position_control(
            self.config, motion.position, motion.velocity,
            angles, (target.north, target.east, target.down), self.target_yaw,
        )
