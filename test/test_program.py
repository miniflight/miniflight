"""Tests for the hardware-neutral flight-program boundary."""

import unittest

from miniflight.program import (
    BodyCommand,
    FlightObservation,
    FlightProgram,
    ProgramCapabilities,
    ReadOnlyProgramRuntime,
)


class FixedSource:
    def __init__(self, observations):
        self.observations = iter(observations)

    def read(self):
        return next(self.observations)


class TestProgram(FlightProgram):
    name = "test"

    def __init__(self):
        self.started_with = None
        self.stopped_with = None
        self.received = []

    def start(self, capabilities):
        self.started_with = capabilities

    def step(self, observation):
        self.received.append(observation)
        return BodyCommand(9.81, 0.1, -0.2, 0.3)

    def stop(self, reason):
        self.stopped_with = reason


def observation(timestamp_ns):
    return FlightObservation(
        sequence=timestamp_ns,
        timestamp_ns=timestamp_ns,
        accel_mps2=(0.0, 0.0, -9.81),
        gyro_rad_s=(0.0, 0.0, 0.0),
        source_age_ns=0,
    )


class FlightProgramTests(unittest.TestCase):
    def test_read_only_runtime_runs_program_without_actuator_access(self):
        program = TestProgram()
        runtime = ReadOnlyProgramRuntime(
            FixedSource([observation(100)]),
            program,
            ProgramCapabilities(frozenset({"imu"}), actuators_enabled=False),
        )

        runtime.start()
        command = runtime.step()
        runtime.stop("complete")

        self.assertEqual(command, BodyCommand(9.81, 0.1, -0.2, 0.3))
        self.assertEqual(len(program.received), 1)
        self.assertFalse(program.started_with.actuators_enabled)
        self.assertEqual(program.stopped_with, "complete")

    def test_read_only_runtime_rejects_enabled_actuators(self):
        with self.assertRaisesRegex(ValueError, "cannot enable actuators"):
            ReadOnlyProgramRuntime(
                FixedSource([observation(100)]),
                TestProgram(),
                ProgramCapabilities(frozenset({"imu"}), actuators_enabled=True),
            )

    def test_runtime_rejects_reused_observation(self):
        runtime = ReadOnlyProgramRuntime(
            FixedSource([observation(100), observation(100)]),
            TestProgram(),
            ProgramCapabilities(frozenset({"imu"}), actuators_enabled=False),
        )
        runtime.start()
        runtime.step()
        with self.assertRaisesRegex(RuntimeError, "timestamps must increase"):
            runtime.step()

    def test_body_command_rejects_invalid_collective(self):
        with self.assertRaisesRegex(ValueError, "non-negative"):
            BodyCommand(-0.1, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
