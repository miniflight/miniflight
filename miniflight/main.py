#!/usr/bin/env python3
"""
Entry point for firmware: load configuration, set up board, and run control loop.
"""
import os
import numpy as np

from common.logger import get_logger
from common.realtime import RateKeeper
from miniflight.control import StabilityController, GenericMixer
from miniflight.board import Board
from miniflight.estimate import MahonyEstimator
from common.math import wrap_angle
from miniflight.input import KeyboardCommandSource, DualSenseCommandSource
from common.interface import CommandSource
from common.types import PilotCommand, SensorReadings, StateEstimate

logger = get_logger("firmware")

def init_board(target_name: str, dt: float) -> tuple[Board, CommandSource | None]:
    """Instantiate the board and an optional command source for pilot input."""
    target = (target_name or "sitl_python").lower()
    
    if target == "sitl_python":
        from target.simulator import SimBoard

        board = SimBoard(dt=dt)
        # Choose input source
        input_kind = (os.environ.get("INPUT") or "keyboard").lower()
        cmd_src: CommandSource | None = None
        if input_kind == "dualsense":
            cmd_src = DualSenseCommandSource()
        else:
            # keyboard via renderer
            get_keys = lambda: board.get_input_state()
            cmd_src = KeyboardCommandSource(get_keys=get_keys)
        return board, cmd_src
    
    elif target == "sitl_c":
        from target.c_firmware_board import CFirmwareBoard
        
        board = CFirmwareBoard(dt=dt)
        # Choose input source
        input_kind = (os.environ.get("INPUT") or "keyboard").lower()
        cmd_src: CommandSource | None = None
        if input_kind == "dualsense":
            cmd_src = DualSenseCommandSource()
        else:
            # keyboard via renderer
            get_keys = lambda: board.get_input_state()
            cmd_src = KeyboardCommandSource(get_keys=get_keys)
        return board, cmd_src
    
    raise NotImplementedError(f"Unsupported target '{target}'")

class Controls:
    """Controls-style loop: update() -> state_control() -> publish() -> run()."""

    def __init__(self, target: str | None, rate_hz: float = 100.0):
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz

        # Board
        self.board, self.command_source = init_board(target, self.dt)

        # Check if board handles its own control (C firmware)
        # Check by class name to avoid circular import
        self.board_has_firmware = type(self.board).__name__ == 'CFirmwareBoard'
        
        # Only instantiate Python estimator/controller if board doesn't have firmware
        if self.board_has_firmware:
            self.controller = None
            self.estimator = None
            logger.info(f"Board initialized ({type(self.board).__name__}) - Using C firmware")
        else:
            self.controller = StabilityController()
            self.estimator = MahonyEstimator()
            logger.info(f"Board initialized ({type(self.board).__name__}) - Using Python firmware")

        # Mixer inferred from board motor geometry if provided
        geometry = self.board.motor_geometry()
        self.mixer = None
        if geometry is not None:
            positions, spins = geometry
            self.mixer = GenericMixer(positions, spins)

    # -- Pipeline stages -----------------------------------------------------

    def update(self) -> SensorReadings:
        """Sample all asynchronous inputs (pilot + sensors)."""
        pilot_command = self._read_pilot_command()
        self._apply_pilot_command(pilot_command)
        return self.board.read_sensors()

    def state_control(self, readings: SensorReadings) -> tuple[StateEstimate, list[float]]:
        """Run estimator + controller to produce motor commands."""
        if self.board_has_firmware:
            # C firmware already computed control in read_sensors()
            # Just retrieve state and output
            state = self.board.get_c_state_estimate()
            control_cmd = list(self.board._last_control_output)
            # C firmware outputs [thrust, roll, pitch, yaw] - may need mixing
            if self.mixer is not None:
                motor_outputs = list(self.mixer.mix(control_cmd))
            else:
                motor_outputs = control_cmd
            return state, motor_outputs
        else:
            # Python firmware
            state = self.estimator.update(readings, self.dt)
            control_cmd = self.controller.update(state, self.dt)
            if self.mixer is not None:
                motor_outputs = list(self.mixer.mix(control_cmd))
            else:
                motor_outputs = list(control_cmd)
            return state, motor_outputs

    def publish(self, motor_outputs: list[float]) -> None:
        """Write actuator commands and handle outputs (telemetry, logging, etc.)."""
        self.board.write_actuators(motor_outputs)

    # -- Helpers -------------------------------------------------------------

    def _read_pilot_command(self) -> PilotCommand:
        if self.command_source is None:
            return PilotCommand()
        try:
            return self.command_source.read(self.dt)
        except Exception as exc:
            logger.warning(f"Command source read failed: {exc}")
            return PilotCommand()

    def _apply_pilot_command(self, cmd: PilotCommand) -> None:
        if self.board_has_firmware:
            # Route to C firmware
            self.board.update_altitude_setpoint(cmd.z_setpoint_delta)
            self.board.update_yaw_rate(cmd.yaw_rate, self.dt)
            self.board.update_attitude_setpoint(cmd.roll_target, cmd.pitch_target)
        else:
            # Update Python controller
            # Altitude target integrates pilot delta
            self.controller.z_setpoint += cmd.z_setpoint_delta
            self.controller.z_setpoint = float(np.clip(self.controller.z_setpoint, 0.0, 20.0))

            # Yaw integrates rate command
            self.controller.yaw_setpoint = wrap_angle(self.controller.yaw_setpoint + cmd.yaw_rate * self.dt)

            # Roll/pitch are treated as direct angle targets
            self.controller.set_attitude_target(cmd.roll_target, cmd.pitch_target, self.controller.yaw_setpoint)

    def run(self):
        logger.info("Starting controls loop")
        rk = RateKeeper(rate_hz=self.rate_hz, print_delay_threshold=None)
        while True:
            readings = self.update()
            _state, motor_outputs = self.state_control(readings)
            self.publish(motor_outputs)
            rk.keep_time()


def main():
    target_env = os.environ.get("TARGET")
    target = target_env.lower() if target_env else None
    Controls(target, rate_hz=100.0).run()

if __name__ == "__main__":
    main() 