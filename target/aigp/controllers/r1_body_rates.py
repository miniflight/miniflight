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
        self.target = None
        self.yaw = None

    def update(self, state: State, gate_index: int, gates) -> BodyRates | None:
        motion, attitude = state.motion, state.attitude
        if motion is None or attitude is None or not gates:
            return None

        if self.yaw is None:
            self.yaw = attitude.yaw
        if gate_index < len(gates) and gate_index != self.gate:
            point = gate_target(motion.position, gate_index, gates)
            self.target = (point.north, point.east, point.down)
            self.gate = gate_index
        if self.target is None:
            return None

        return position_control(self.config, motion.position, motion.velocity,
                                (attitude.roll, attitude.pitch, attitude.yaw), self.target, self.yaw)
