"""R1 position/attitude feedback -> rate PI -> mixer -> individual motors."""

from enum import Enum, auto

from pymavlink.dialects.v20 import common as mavlink

from miniflight import MotorControls, vehicle
from miniflight.mixer import mix
from miniflight.rate import rate_control
from target.aigp import MOTOR_MIX
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_body_rates import Controller as PositionController


class RatePhase(Enum):
    WAITING_FOR_ARM_ACK = auto()
    WAITING_FOR_POST_ARM_SAMPLE = auto()
    INTEGRATING = auto()


class Controller(BaseController[MotorControls]):
    targets = ("vq1.r1",)

    def __init__(self, kp=(.025, .025, .03125), ki=(.1, .1, .125), position=None):
        self.position = PositionController() if position is None else position
        self.arm_received_at = None
        self.rate_phase = RatePhase.WAITING_FOR_ARM_ACK
        self.kp, self.ki = kp, ki
        self.integral, self.time = (0, 0, 0), None
        self.gyro = self.rate_request = self.torque = self.motor_request = None

    def update(self, telemetry, frames, race_state) -> MotorControls | None:
        for packet in telemetry:
            message = packet.data
            kind = message.get_type()
            if kind == "COMMAND_ACK" and message.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if message.result != mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError("native arm command was rejected")
                if self.rate_phase is RatePhase.WAITING_FOR_ARM_ACK:
                    self.arm_received_at = packet.received_at
                    self.rate_phase = RatePhase.WAITING_FOR_POST_ARM_SAMPLE
        request = self.position.update(telemetry, frames, race_state)
        state = vehicle.state
        feedback = None if state is None else state.rate_feedback
        if request is None or feedback is None:
            return None
        stamp, received_at, self.gyro = feedback
        post_arm_sample = self.rate_phase is not RatePhase.WAITING_FOR_ARM_ACK and received_at > self.arm_received_at
        dt = stamp - self.time if self.rate_phase is RatePhase.INTEGRATING and post_arm_sample else 0
        self.integral_dt = dt
        if dt > .3:
            raise TimeoutError("rate feedback lost its cadence")
        self.rate_request = (request.roll_rate, request.pitch_rate, request.yaw_rate)
        self.torque, integral = rate_control(self.kp, self.ki, self.integral, self.rate_request, self.gyro, dt)
        self.motor_request = mix(request.thrust, self.torque, MOTOR_MIX)
        command = MotorControls(self.motor_request)
        self.integral, self.time = integral, stamp if post_arm_sample else None
        if self.rate_phase is not RatePhase.WAITING_FOR_ARM_ACK:
            self.rate_phase = RatePhase.INTEGRATING if post_arm_sample else RatePhase.WAITING_FOR_POST_ARM_SAMPLE
        return command
