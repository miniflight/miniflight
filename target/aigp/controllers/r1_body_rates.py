"""Follow the published VQ1 gates with Python position feedback and body-rate commands."""

from dataclasses import replace

from miniflight import BodyRates, State
from miniflight.position import PositionConfig, position_control
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import gate_target
from target.aigp.yaw_tracking import YawRateFeedback


# Local hover calibration measured on VQ1 build 3391; see docs/body-rates.md.
CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, config: PositionConfig = CONFIG, yaw_feedback=True):
        self.config = config
        self.yaw_feedback = YawRateFeedback() if yaw_feedback else None
        self.gate = None
        self.target = None
        self.yaw = None

    def update(self, state: State, gate_index: int, gates) -> BodyRates | None:
        motion, attitude = state.motion, state.attitude
        if motion is None or attitude is None or not gates:
            if self.yaw_feedback is not None:
                self.yaw_feedback.reset()
            return None

        if self.yaw is None:
            self.yaw = attitude.yaw
        if gate_index < len(gates) and gate_index != self.gate:
            point = gate_target(motion.position, gate_index, gates)
            self.target = (point.north, point.east, point.down)
            self.gate = gate_index
        if self.target is None:
            return None

        command = position_control(self.config, motion.position, motion.velocity,
                                   (attitude.roll, attitude.pitch, attitude.yaw), self.target, self.yaw)
        if self.yaw_feedback is not None:
            command = replace(command, yaw_rate=self.yaw_feedback.update(command.yaw_rate, state.gyro[2], state.dt))
        return command
