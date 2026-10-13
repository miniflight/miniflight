"""R1 position/attitude feedback -> rate PI -> mixer -> individual motors."""

from pymavlink.dialects.v20 import common as mavlink

from miniflight import MotorControls
from miniflight.mixer import mix
from miniflight.rate import rate_control
from target.aigp import MOTOR_MIX
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_body_rates import Controller as PositionController


class Controller(BaseController[MotorControls]):
    targets = ("vq1.r1",)

    def __init__(self, kp=(.025, .025, .03125), ki=(.1, .1, .125), position=None):
        self.position = PositionController() if position is None else position
        self.imu = None
        self.imu_received_at = self.arm_received_at = None
        self.kp, self.ki = kp, ki
        self.integral, self.time = (0, 0, 0), None
        self.rate_request = self.gyro = self.torque = self.motor_request = None

    def update(self, telemetry, frames, race_state) -> MotorControls | None:
        for packet in telemetry:
            message = packet.data
            kind = message.get_type()
            if kind == "COLLISION":
                raise RuntimeError("native collision")
            if kind == "HIGHRES_IMU" and getattr(message, "_host_received_at", None) == packet.received_at:
                self.imu, self.imu_received_at = message, packet.received_at
            if kind == "COMMAND_ACK" and message.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if message.result != mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError("native arm command was rejected")
                if self.arm_received_at is None:
                    self.arm_received_at = packet.received_at
        request = self.position.update(telemetry, frames, race_state)
        if request is None or self.imu is None:
            return None
        imu = self.imu
        stamp = imu.time_usec * 1e-6
        post_arm_sample = self.arm_received_at is not None and self.imu_received_at > self.arm_received_at
        dt = 0 if self.time is None or not post_arm_sample else stamp - self.time
        self.integral_dt = dt
        if dt > .3:
            raise TimeoutError("rate feedback lost its IMU cadence")
        self.rate_request = (request.roll_rate, request.pitch_rate, request.yaw_rate)
        self.gyro = (-imu.xgyro, -imu.ygyro, -imu.zgyro)  # Build-3391 wire -> FRD rad/s.
        self.torque, integral = rate_control(self.kp, self.ki, self.integral, self.rate_request, self.gyro, dt)
        self.motor_request = mix(request.thrust, self.torque, MOTOR_MIX)
        command = MotorControls(self.motor_request)  # Infeasible outputs stop the run; no silent clipping.
        self.integral, self.time = integral, stamp if post_arm_sample else None
        return command
