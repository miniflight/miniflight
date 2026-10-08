"""Follow the published VQ1 gates with Python position feedback and body-rate commands."""

import math

from miniflight import BodyRates, Ned
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
        self.target_position = None
        self.target_yaw = None

    def update(self, telemetry, frames) -> BodyRates | None:
        target = self.course.update(telemetry, frames)
        attitude = self.course.telemetry.get("ATTITUDE")
        imu = self.course.telemetry.get("HIGHRES_IMU")
        if target is None or attitude is None or imu.received_at - attitude.received_at > 1:
            return None
        motion, attitude = self.course.telemetry["LOCAL_POSITION_NED"].data, attitude.data
        angles = (attitude.roll, -attitude.pitch, -attitude.yaw)  # Build-3391 wire → FRD/NED, explicit controller math.
        if not all(math.isfinite(v) for v in angles):
            return None
        self.target_position = Ned(target.north, target.east, target.down)

        # Keep the initial heading throughout the course, including gate changes.
        if self.target_yaw is None:
            self.target_yaw = angles[2]

        return position_control(
            self.config, (motion.x, motion.y, motion.z), (motion.vx, motion.vy, motion.vz),
            angles, self.target_position, self.target_yaw,
        )
