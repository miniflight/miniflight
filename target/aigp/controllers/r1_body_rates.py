"""Follow the published VQ1 gates with Python position feedback and body-rate commands."""

from miniflight import BodyRates, State
from miniflight.position import PositionConfig, position_control
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import gate_target


# Local hover calibration measured on VQ1 build 3391; see docs/body-rates.md.
CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, config: PositionConfig = CONFIG):
        self.config = config
        self.gate = None
        self.target_position = None
        self.target_yaw = None

    def update(self, state: State, gate_index: int, gates) -> BodyRates | None:
        motion, attitude = state.motion, state.attitude
        if motion is None or attitude is None or not gates:
            return None

        if gate_index < len(gates) and gates[gate_index] != self.gate:
            self.target_position = gate_target(motion.position, gate_index, gates)
            self.gate = gates[gate_index]
        if self.target_position is None:
            return None

        # Keep the initial heading throughout the course, including gate changes.
        if self.target_yaw is None:
            self.target_yaw = attitude.yaw

        return position_control(
            self.config, motion.position, motion.velocity,
            (attitude.roll, attitude.pitch, attitude.yaw), self.target_position, self.target_yaw,
        )
