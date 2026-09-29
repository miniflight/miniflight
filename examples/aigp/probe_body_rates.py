"""Measure VQ1 body-rate/thrust pulses between settled position holds.

python -m examples.aigp.probe_body_rates thrust trace.jsonl --thrust .26 .27 .28 --duration .8
python -m examples.aigp.probe_body_rates rates trace.jsonl --thrust .266 --rate .25
python -m examples.aigp.probe_body_rates replay trace.jsonl
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

from miniflight import Attitude, BodyRates, Motion, MotorOutputs, Ned, PositionNed, State
from target.aigp.controllers import BaseController
from target.aigp.simulator import AIGPSimulator, BASE, SHIPPING, SimulatorClient, sha256


def command_data(command):
    return None if command is None else {"kind": type(command).__name__, **asdict(command)}


def angular_rates(command):
    return command.roll_rate, command.pitch_rate, command.yaw_rate


class Probe(BaseController):
    targets = ("vq1.r1",)

    def __init__(self, commands, duration=.6):
        commands = tuple(commands)
        if not commands or not all(isinstance(command, BodyRates) for command in commands):
            raise ValueError("provide at least one body-rate command")
        if not math.isfinite(duration) or not 0 < duration <= 1:
            raise ValueError("pulse duration must be between zero and one second")
        self.commands, self.duration = commands, duration
        self.target = None
        self.trial = 0
        self.phase = "settle"
        self.started_at = self.steady_at = None

    def update(self, state, gate_index, gates):
        stamp, motion, attitude = state.time, state.motion, state.attitude
        if motion is None or attitude is None:
            return None
        if self.target is None:
            north, east, down = motion.position
            self.target = PositionNed(north, east, down - 3)
            self.started_at = stamp
        distance = math.dist(motion.position, (self.target.north, self.target.east, self.target.down))
        speed = math.hypot(*motion.velocity)
        tilt = max(abs(attitude.roll), abs(attitude.pitch))

        if self.phase == "pulse":
            if distance > 2 or speed > 5 or tilt > .5:
                raise RuntimeError("pulse exceeded its motion envelope")
            if stamp - self.started_at < self.duration:
                return self.commands[self.trial]
            previous = self.commands[self.trial]
            self.trial += 1
            if (self.trial < len(self.commands) and any(angular_rates(previous))
                    and not any(angular_rates(self.commands[self.trial]))):
                # Clear the rate request before handing control back to position mode.
                self.started_at = stamp
                return self.commands[self.trial]
            self.phase, self.started_at, self.steady_at = "settle", stamp, None

        if stamp - self.started_at > 20:
            raise TimeoutError("position hold did not settle within 20 simulator seconds")
        if distance < .15 and speed < .15 and tilt < .04 and math.hypot(*state.gyro) < .03:
            if self.steady_at is None:
                self.steady_at = stamp
            if stamp - self.steady_at >= .6:
                if self.trial == len(self.commands):
                    self.phase = "done"
                    raise StopIteration
                self.phase, self.started_at = "pulse", stamp
                return self.commands[self.trial]
        else:
            self.steady_at = None
        return self.target


class RecordedProbe(Probe):
    def __init__(self, commands, duration, record):
        super().__init__(commands, duration)
        self.record = record

    def update(self, state, gate_index, gates):
        row = dict(event="update", state=asdict(state), gate_index=gate_index)
        try:
            command = super().update(state, gate_index, gates)
        except Exception as error:
            self.record(**row, phase=self.phase, trial=self.trial, error=type(error).__name__)
            raise
        self.record(**row, phase=self.phase, trial=self.trial, command=command_data(command))
        return command


class RecordedClient(SimulatorClient):
    def __init__(self, record):
        super().__init__(camera_port=None)
        self.record = record

    def send(self, command):
        super().send(command)
        self.record(event="sent", command=command_data(command))

    def _set_armed(self, armed):
        super()._set_armed(armed)
        self.record(event="arm_request", armed=armed)

    def _receive(self, message, peer, now):
        super()._receive(message, peer, now)
        if message.get_type() in ("COMMAND_ACK", "COLLISION", "HEARTBEAT"):
            self.record(event="telemetry", message=message.to_dict())


def read_state(data):
    data = dict(data)
    data["acceleration"], data["gyro"] = tuple(data["acceleration"]), tuple(data["gyro"])
    if data["motion"] is not None:
        motion = dict(data["motion"])
        motion["position"], motion["velocity"] = Ned(*motion["position"]), Ned(*motion["velocity"])
        data["motion"] = Motion(**motion)
    if data["attitude"] is not None:
        data["attitude"] = Attitude(**data["attitude"])
    if data["motors"] is not None:
        motors = dict(data["motors"])
        motors["outputs"] = tuple(motors["outputs"])
        data["motors"] = MotorOutputs(**motors)
    if data["frame"] is not None:
        raise ValueError("this probe records without camera frames")
    return State(**data)


def replay(path):
    with path.open() as source:
        config = json.loads(next(source))
        probe = Probe([BodyRates(**command) for command in config["commands"]], config["duration"])
        updates = 0
        for line in source:
            row = json.loads(line)
            if row["event"] != "update":
                continue
            actual = {}
            try:
                actual["command"] = command_data(probe.update(read_state(row["state"]), row["gate_index"], None))
            except Exception as error:
                actual["error"] = type(error).__name__
            expected = {key: row[key] for key in ("command", "error") if key in row}
            if actual != expected or (probe.phase, probe.trial) != (row["phase"], row["trial"]):
                raise AssertionError(f"replay differs at update {updates}")
            updates += 1
    return {"updates": updates, "completed": probe.phase == "done"}


def run(path, commands, duration):
    commands = tuple(commands)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", buffering=1) as output:
        def record(**row):
            output.write(json.dumps(dict(host_time=time.monotonic(), **row), allow_nan=False) + "\n")

        root = BASE.parents[1]
        sources = ("target/aigp/simulator.py", "miniflight/vehicle.py", "miniflight/control.py",
                   "examples/aigp/probe_body_rates.py")
        record(event="config", target="vq1.r1", hz=50, timeout=.3, duration=duration,
               angular_convention="FRD/NED after AIGP build-3391 wire conversion",
               commands=[asdict(command) for command in commands],
               sources={name: sha256(root / name) for name in sources})
        probe = RecordedProbe(commands, duration, record)
        client = RecordedClient(record)
        sim = AIGPSimulator(probe, client=client, timeout=.3, gate_timeout=180)
        try:
            sim.rollout()
            if probe.phase != "done":
                raise RuntimeError("simulator stopped before all probe trials completed")
        finally:
            executable = BASE / ".runtime/vq1" / SHIPPING
            record(event="result", completed=probe.phase == "done", connection_closed=not client.connected,
                   executable_sha256=sha256(executable) if executable.is_file() else None)
    return replay(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("thrust", "rates", "replay"))
    parser.add_argument("trace", type=Path)
    parser.add_argument("--thrust", type=float, nargs="+", default=(.26, .27, .28))
    parser.add_argument("--rate", type=float, default=.25, help="positive and negative pulse size on each axis, rad/s")
    parser.add_argument("--duration", type=float, default=.6, help="pulse duration in simulator seconds")
    args = parser.parse_args()
    if args.mode == "replay":
        print(json.dumps(replay(args.trace)))
        return
    if args.mode == "rates":
        if len(args.thrust) != 1 or not math.isfinite(args.rate) or not 0 < args.rate <= 1:
            parser.error("rates requires one --thrust value and 0 < --rate <= 1")
        commands = [BodyRates(**{axis: sign * args.rate}, thrust=args.thrust[0])
                    for axis in ("roll_rate", "pitch_rate", "yaw_rate") for sign in (1, -1)]
        commands = [command for pulse in commands for command in (pulse, BodyRates(thrust=args.thrust[0]))]
    else:
        commands = [BodyRates(thrust=thrust) for thrust in args.thrust]

    def stop(signum, frame):
        raise TimeoutError("probe interrupted or exceeded 180 wall-clock seconds")

    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGALRM):
        signal.signal(signum, stop)
    signal.alarm(180)
    try:
        print(json.dumps(run(args.trace, commands, args.duration)))
    finally:
        signal.alarm(0)


if __name__ == "__main__":
    main()
