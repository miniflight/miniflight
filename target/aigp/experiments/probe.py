"""Bounded AIGP body-rate probes with raw-wire recording and offline replay."""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path
from statistics import fmean, median, pstdev

from pymavlink.dialects.v20 import common as mavlink

from miniflight import BodyRates, PositionNed
from target.aigp.controllers import BaseController
from target.aigp.experiments.recording import RecordedClient, RecordedController, command_data, read_metadata, recording, replay as replay_controller
from target.aigp.simulator import AIGPSimulator, BASE, SHIPPING, sha256
from target.aigp.experiments.yaw_tracking import YawRateFeedback


def angular_rates(command):
    return command.roll_rate, command.pitch_rate, command.yaw_rate


class Probe(BaseController[BodyRates | PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self, commands, duration=.6):
        commands = tuple(commands)
        if not commands or not all(isinstance(command, BodyRates) for command in commands):
            raise ValueError("provide at least one body-rate command")
        pulses = []
        for index, command in enumerate(commands):
            pulses.append(command)
            if any(angular_rates(command)) and (index + 1 == len(commands) or any(angular_rates(commands[index + 1]))):
                pulses.append(BodyRates(thrust=command.thrust))
        commands = tuple(pulses)
        yaw_only = any(command.yaw_rate for command in commands) and all(
            command.roll_rate == command.pitch_rate == 0 for command in commands)
        max_duration = 3 if yaw_only else 1
        if not math.isfinite(duration) or not 0 < duration <= max_duration:
            raise ValueError(f"pulse duration must be between zero and {max_duration} seconds")
        self.commands, self.duration = commands, duration
        self.target = None
        self.trial = 0
        self.phase = "settle"
        self.started_at = self.steady_at = None
        self.telemetry = {}
        self.time = None
        self.dt = 0
        self.gyro = (0, 0, 0)

    def update(self, telemetry, frames, race_state) -> BodyRates | PositionNed | None:
        for packet in telemetry:
            self.telemetry[packet.data.get_type()] = packet.data
        imu, motion, attitude = (self.telemetry.get(kind) for kind in ("HIGHRES_IMU", "LOCAL_POSITION_NED", "ATTITUDE"))
        if imu is None or motion is None or attitude is None or race_state is None or not race_state.started:
            return None
        stamp = imu.time_usec * 1e-6
        self.dt = 0 if self.time is None else stamp - self.time
        self.time, self.gyro = stamp, (-imu.xgyro, -imu.ygyro, -imu.zgyro)
        position, velocity = (motion.x, motion.y, motion.z), (motion.vx, motion.vy, motion.vz)
        if self.target is None:
            north, east, down = position
            self.target = PositionNed(north, east, down - 3)
            self.started_at = stamp
        distance = math.dist(position, (self.target.north, self.target.east, self.target.down))
        speed = math.hypot(*velocity)
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
        if distance < .15 and speed < .15 and tilt < .04 and math.hypot(*self.gyro) < .03:
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


class TrackedProbe(BaseController[BodyRates | PositionNed]):
    targets = Probe.targets

    def __init__(self, probe, record=None, feedback_config=None):
        self.probe, self.record = probe, record
        self.feedback = YawRateFeedback(**(feedback_config or {}))

    def update(self, telemetry, frames, race_state):
        requested = self.probe.update(telemetry, frames, race_state)
        if self.record is not None:
            self.record(event="requested", command=command_data(requested))
        if isinstance(requested, BodyRates):
            return replace(requested, yaw_rate=self.feedback.update(requested.yaw_rate, self.probe.gyro[2], self.probe.dt))
        self.feedback.reset()
        return requested


def replay(path):
    config = read_metadata(path)
    probe = Probe([BodyRates(**command) for command in config["commands"]], config["duration"])
    feedback = config.get("yaw_feedback", False)
    controller = TrackedProbe(probe, feedback_config=feedback if isinstance(feedback, dict) else None) if feedback else probe
    updates = replay_controller(lambda: controller, path)
    return {"updates": updates, "completed": probe.phase == "done"}


class DiagnosticClient(RecordedClient):
    """Retain exact successful outgoing packets and accepted sensor packets."""

    def __init__(self, record):
        super().__init__(record, camera_port=None)
        self._outgoing = mavlink.MAVLink(None)

    def write(self, packet):
        result = super().write(packet)
        for message in self._outgoing.parse_buffer(packet) or ():
            if message.get_type() in ("SET_ATTITUDE_TARGET", "SET_POSITION_TARGET_LOCAL_NED"):
                self.record(event="wire_sent", packet_hex=bytes(packet).hex(), message=message.to_dict())
        return result

    def _receive(self, message, peer, now):
        decoded = super()._receive(message, peer, now)
        if (message.get_type() in ("HIGHRES_IMU", "ATTITUDE", "LOCAL_POSITION_NED", "ACTUATOR_OUTPUT_STATUS")
                and self._telemetry.get(message.get_type()) is message):
            # Bytes preserve unsupported/NaN fields without inventing JSON values.
            self.record(event="wire_received", received_at=now, packet_hex=bytes(message.get_msgbuf()).hex())
        return decoded


def run(path, commands, duration, hz=50, yaw_feedback=False):
    commands = tuple(commands)
    probe = Probe(commands, duration)
    controller = TrackedProbe(probe) if yaw_feedback else probe
    feedback_config = ({key: getattr(controller.feedback, key)
                        for key in ("gain", "max_correction", "max_rate", "settle_time", "step_threshold")}
                       if yaw_feedback else None)
    root = BASE.parents[1]
    sources = ("target/aigp/simulator.py", "target/aigp/client.py", "target/aigp/native.py",
               "target/aigp/experiments/recording.py", "miniflight/vehicle.py",
               "miniflight/control.py", "miniflight/state.py", "target/aigp/experiments/probe.py",
               "target/aigp/experiments/yaw_tracking.py", "examples/aigp/probe_body_rates.py")
    metadata = dict(target="vq1.r1", hz=hz, timeout=.3, duration=duration, yaw_feedback=feedback_config,
                    angular_convention="FRD/NED after AIGP build-3391 wire conversion",
                    commands=[asdict(command) for command in probe.commands],
                    sources={name: sha256(root / name) for name in sources})
    with recording(path, metadata) as record:
        client = DiagnosticClient(record)
        if yaw_feedback:
            controller.record = record
        sim = AIGPSimulator(RecordedController(lambda: controller, record), client=client, hz=hz, timeout=.3)
        try:
            sim.rollout()
            if probe.phase != "done":
                raise RuntimeError("simulator stopped before all probe trials completed")
        finally:
            executable = BASE / ".runtime/vq1" / SHIPPING
            record(event="result", completed=probe.phase == "done", connection_closed=not client.connected,
                   executable_sha256=sha256(executable) if executable.is_file() else None)
    return replay(path)


def analyze_yaw(path, tail_seconds=.5):
    """Compare successful yaw packets with the final sensor window of each step.

    Host receipt times associate packets with commands; device timestamps measure
    the gyro window and attitude slope. This never uses private probe phases.
    """
    if not math.isfinite(tail_seconds) or tail_seconds <= 0:
        raise ValueError("tail window must be finite and positive")
    with Path(path).open() as source:
        rows = [json.loads(line) for line in source]
    if not rows or rows[-1].get("event") != "result" or not rows[-1].get("completed"):
        raise ValueError("yaw analysis requires a completed diagnostic trace")
    decoder = mavlink.MAVLink(None)
    outgoing = mavlink.MAVLink(None)
    tracked = bool(rows[0].get("metadata", {}).get("yaw_feedback", False))
    imu, attitudes, segments, send_times = [], [], [], []
    active = requested = None
    for row in rows:
        if row["event"] == "requested":
            requested = row["command"]
        elif row["event"] == "wire_received":
            for message in decoder.parse_buffer(bytes.fromhex(row["packet_hex"])) or ():
                stamp = row["received_at"]
                if message.get_type() == "HIGHRES_IMU":
                    imu.append((stamp, message.time_usec * 1e-6, -message.zgyro))
                elif message.get_type() == "ATTITUDE":
                    attitudes.append((stamp, message.time_boot_ms * .001, message.roll, -message.pitch, -message.yaw))
        elif row["event"] == "wire_sent":
            message, = outgoing.parse_buffer(bytes.fromhex(row["packet_hex"]))
            is_rate = message.get_type() == "SET_ATTITUDE_TARGET"
            wire_yaw = -message.body_yaw_rate if is_rate else 0
            if tracked:
                yaw = requested["yaw_rate"] if requested and requested["kind"] == "BodyRates" else 0
                thrust = requested.get("thrust") if requested else None
            else:
                yaw, thrust = wire_yaw, message.thrust if is_rate else None
            key = (yaw, thrust) if yaw else None
            if key is not None:
                if (not is_rate or message.type_mask != 144 or message.body_roll_rate or message.body_pitch_rate
                        or not all(math.isfinite(value) for value in (yaw, thrust, wire_yaw))):
                    raise ValueError("yaw analysis requires yaw-only radians packets")
            if active is not None and active["key"] != key:
                active["end"] = row["host_time"]
                segments.append(active)
                active = None
            if key is not None and active is None:
                active = dict(key=key, begin=row["host_time"], sent=[])
            if active is not None:
                active["sent"].append((row["host_time"], wire_yaw))
            send_times.append(row["host_time"])
    if not segments or active is not None:
        raise ValueError("trace must contain completed yaw steps and raw sensor packets")
    results = []
    for segment in segments:
        samples = [sample for sample in imu if segment["begin"] <= sample[0] < segment["end"]]
        if not samples or samples[-1][1] - samples[0][1] < tail_seconds:
            raise ValueError("yaw step is shorter than the measurement window")
        tail = [sample for sample in samples if sample[1] >= samples[-1][1] - tail_seconds]
        angles = [sample for sample in attitudes if tail[0][0] <= sample[0] < segment["end"]]
        if len(tail) < 3 or len(angles) < 3:
            raise ValueError("yaw step has insufficient gyro or attitude samples")
        if not all(math.isfinite(value) for sample in tail + angles for value in sample):
            raise ValueError("yaw analysis requires finite sensor samples")
        angles.sort(key=lambda sample: sample[1])
        # The probe starts level. Check that Euler yaw remains a valid independent
        # near-level comparison rather than assuming it equals body r when tilted.
        tilt = max(max(abs(sample[2]), abs(sample[3])) for sample in angles)
        if tilt > .05:
            raise ValueError("yaw attitude comparison requires a near-level step")
        unwrapped = [angles[0][4]]
        for previous, sample in zip(angles, angles[1:]):
            unwrapped.append(unwrapped[-1] + (sample[4] - previous[4] + math.pi) % (2 * math.pi) - math.pi)
        times = [sample[1] - angles[0][1] for sample in angles]
        mean_time, mean_angle = fmean(times), fmean(unwrapped)
        denominator = sum((stamp - mean_time) ** 2 for stamp in times)
        if denominator == 0:
            raise ValueError("attitude timestamps do not advance")
        slope = sum((stamp - mean_time) * (angle - mean_angle) for stamp, angle in zip(times, unwrapped)) / denominator
        measured = fmean(sample[2] for sample in tail)
        request, thrust = segment["key"]
        wire_tail = [rate for stamp, rate in segment["sent"] if stamp >= tail[0][0]]
        if not wire_tail:
            wire_tail = [segment["sent"][-1][1]]  # last request remains active until the next command
        results.append(dict(request=request, thrust=thrust, samples=len(tail),
                            duration=samples[-1][1] - samples[0][1], gyro_mean=measured,
                            gyro_std=pstdev(sample[2] for sample in tail), gain=measured / request,
                            peak_ratio=max(sample[2] / request for sample in samples),
                            wire_yaw_mean=fmean(wire_tail), attitude_yaw_rate=slope, max_tilt=tilt))
    intervals = [end - start for start, end in zip(send_times, send_times[1:])]
    return dict(yaw_feedback=tracked, tail_seconds=tail_seconds, command_interval_median=median(intervals), steps=results)
