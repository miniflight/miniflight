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

from miniflight import BodyRates, PositionNed
from target.aigp.controllers import BaseController
from target.aigp.recording import RecordedClient, RecordedController, read_metadata, recording, replay as replay_controller
from target.aigp.simulator import AIGPSimulator, BASE, SHIPPING, sha256


def angular_rates(command):
    return command.roll_rate, command.pitch_rate, command.yaw_rate


class Probe(BaseController[BodyRates | PositionNed]):
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

    def update(self, state, gate_index, gates) -> BodyRates | PositionNed | None:
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


def replay(path):
    config = read_metadata(path)
    probe = Probe([BodyRates(**command) for command in config["commands"]], config["duration"])
    updates = replay_controller(probe, path)
    return {"updates": updates, "completed": probe.phase == "done"}


def run(path, commands, duration):
    commands = tuple(commands)
    probe = Probe(commands, duration)
    root = BASE.parents[1]
    sources = ("target/aigp/simulator.py", "target/aigp/recording.py", "miniflight/vehicle.py",
               "miniflight/control.py", "examples/aigp/probe_body_rates.py")
    metadata = dict(target="vq1.r1", hz=50, timeout=.3, duration=duration,
                    angular_convention="FRD/NED after AIGP build-3391 wire conversion",
                    commands=[asdict(command) for command in commands],
                    sources={name: sha256(root / name) for name in sources})
    with recording(path, metadata) as record:
        client = RecordedClient(record, camera_port=None)
        sim = AIGPSimulator(RecordedController(probe, record), client=client, timeout=.3, gate_timeout=180)
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
