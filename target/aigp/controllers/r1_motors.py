"""R1 position/attitude feedback -> rate PI -> mixer -> individual motors."""

from pymavlink.dialects.v20 import common as mavlink

from miniflight import MotorControls, vehicle
from miniflight.mixer import mix
from miniflight.rate import rate_control
from target.aigp import MOTOR_MIX
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_body_rates import Controller as PositionController


class Controller(BaseController[MotorControls]):
    targets = ("vq1.r1",)

    def __init__(self, kp=(.025, .025, .03125), ki=(.1, .1, .125), position=None):
        self.position = PositionController() if position is None else position
        self.arm_received_at = None
        self.kp, self.ki = kp, ki
        self.integral, self.time = (0, 0, 0), None
        self.rate_request = self.torque = self.motor_request = None

    def update(self, telemetry, frames, race_state) -> MotorControls | None:
        for packet in telemetry:
            message = packet.data
            kind = message.get_type()
            if kind == "COLLISION":
                raise RuntimeError("native collision")
            if kind == "COMMAND_ACK" and message.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if message.result != mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError("native arm command was rejected")
                if self.arm_received_at is None:
                    self.arm_received_at = packet.received_at
        request = self.position.update(telemetry, frames, race_state)
        state = vehicle.state
        if request is None or state is None or state.time is None:
            return None
        stamp = state.time
        post_arm_sample = self.arm_received_at is not None and state.received_at > self.arm_received_at
        dt = 0 if self.time is None or not post_arm_sample else stamp - self.time
        self.integral_dt = dt
        if dt > .3:
            raise TimeoutError("rate feedback lost its IMU cadence")
        self.rate_request = (request.roll_rate, request.pitch_rate, request.yaw_rate)
        self.torque, integral = rate_control(self.kp, self.ki, self.integral, self.rate_request, state.gyro, dt)
        self.motor_request = mix(request.thrust, self.torque, MOTOR_MIX)
        command = MotorControls(self.motor_request)  # Infeasible outputs stop the run; no silent clipping.
        self.integral, self.time = integral, stamp if post_arm_sample else None
        return command
