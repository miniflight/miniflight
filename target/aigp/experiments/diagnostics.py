"""Convert recorded AIGP inputs and sent commands into ordinary SI quantities."""

import math
from miniflight.diagnostics import chart_data
from target.aigp.experiments.recording import ReplayClient


PANELS = (
    ("Roll rate in FRD", "rad/s", (("Requested", "roll_ref_rad_s"), ("Observed", "roll_rad_s"))),
    ("Pitch rate in FRD", "rad/s", (("Requested", "pitch_ref_rad_s"), ("Observed", "pitch_rad_s"))),
    ("Yaw rate in FRD", "rad/s", (("Requested", "yaw_ref_rad_s"), ("Observed", "yaw_rad_s"))),
    ("Collective", "fraction", (("Requested", "collective_request"), ("Reported", "motor_reported_mean"))),
    ("Horizontal speed", "m/s", (("Reference", "speed_ref_m_s"), ("Observed", "speed_m_s"))),
    ("Position error: reference minus observed", "m", (("North", "north_error_m"), ("East", "east_error_m"), ("Down", "down_error_m"))),
    ("Horizontal acceleration request before limits", "m/s²", (("Requested", "acceleration_request_m_s2"), ("Limit", "acceleration_limit_m_s2"))),
    ("Motor requests and reported mean", "fraction", (("Min", "motor_min"), ("Mean", "motor_mean"), ("Max", "motor_max"), ("Reported", "motor_reported_mean"))),
)
DETAIL_PANELS = (
    ("Vertical velocity: positive down", "m/s", (("Reference", "down_speed_ref_m_s"), ("Observed", "down_speed_m_s"))),
    ("Accepted sample age at command send", "s", (("Pose", "pose_age_s"), ("Rate feedback", "rate_age_s"))),
    ("Sensor boot time minus pose boot time", "s", (("IMU", "pose_clock_gap_s"), ("Attitude", "attitude_clock_gap_s"))),
)


def snapshot(observer, samples, source):
    race, metadata = observer.status, observer.metadata
    clock = race.get("finish_time_seconds") if race.get("completed") else race.get("time_seconds")
    reference = (metadata.get("reference") or {}).get("duration_s")
    seconds = lambda value: "—" if value is None else f"{value:.3f} s"
    return dict(source=f"{source.parent.name}/{source.name}", flight_active=bool(metadata) and not observer.finished,
                title=f"{metadata.get('target', 'Waiting')} · {metadata.get('controller', 'unknown')} · {source.parent.name}/{source.name}",
                summary={"Native finish" if race.get("completed") else "Native clock": seconds(clock),
                         "Reference": seconds(reference), "Gates": race.get("active_gate_index", "—"),
                         "Collisions": observer.collisions, "Commands": len(samples)},
                charts=chart_data(samples, PANELS, x_label="Reference time (s)") +
                       [dict(chart, detail=True) for chart in chart_data(samples, DETAIL_PANELS, x_label="Reference time (s)")])


class Observe:
    def __init__(self):
        self.client = ReplayClient()
        self.metadata, self.pending = {}, None
        self.status, self.collisions = {}, 0
        self.finished = False

    def consume(self, row):
        event, command = row["event"], row.get("command")
        if event == "config":
            self.metadata = row["metadata"]
        if event == "telemetry" and row["message"]["mavpackettype"] == "COLLISION":
            self.collisions += 1
        if event == "result":
            self.finished = True
            self.status = row["race"] or {}
        if event == "update":
            self.status = row["native_race"] or {}
            self.client.feed(row["telemetry"])
            self.client.read(timeout=0)
            self.pending = row if "signals" in row else None
        if event != "sent" or command["kind"] not in ("MotorControls", "BodyRates") or self.pending is None:
            return None
        if command["kind"] == "BodyRates" and command["thrust"] == 0:
            self.pending = None
            return None
        if command != self.pending["command"]:
            raise ValueError("sent command has no matching controller update")
        state = self.client.state
        feedback = state.rate_feedback
        motion = state.motion
        config = self.metadata.get("controller_config", {})
        signals = self.pending["signals"]
        rate_time = signals.get("rate_time_s", signals.get("tilt_time_s"))
        rate_sample = next((sample for sample in (feedback, state.attitude)
                            if sample is not None and sample.time == rate_time), None)
        rate_age = None if rate_sample is None else row["host_time"]-rate_sample.received_at
        if command["kind"] == "BodyRates":
            signals = dict(signals, rate_ref_frd_rad_s=[command[axis + "_rate"] for axis in ("roll", "pitch", "yaw")],
                           gyro_frd_rad_s=feedback.gyro)
        sample = quantities(signals, motion.position, motion.velocity,
                            command.get("outputs", ()), row["host_time"]-motion.received_at, rate_age,
                            config.get("max_horizontal_acceleration") or config.get("max_acceleration"))
        sample["collective_request"] = command.get("thrust", sample["motor_mean"])
        native = state.motors
        values = native.outputs[:4] if native is not None and native.active & 15 == 15 else ()
        sample["motor_reported_mean"] = sum(values)/4 if len(values) == 4 and all(math.isfinite(v) for v in values) else None
        sample["pose_clock_gap_s"] = state.time - motion.time
        attitude = state.attitude
        sample["attitude_clock_gap_s"] = None if attitude is None else attitude.time - motion.time
        self.pending = None
        return sample


def quantities(signals, position, velocity, motors, pose_age, rate_age, acceleration_limit=None):
    target_position = signals["position_ref_ned_m"]
    target_velocity = signals["velocity_ref_ned_m_s"]
    north_error, east_error, down_error = (wanted-actual for wanted, actual in zip(target_position, position, strict=True))
    acceleration = signals["acceleration_request_ned_m_s2"]
    return dict(time_s=signals["reference_time_s"], speed_ref_m_s=math.hypot(*target_velocity[:2]),
                down_speed_ref_m_s=target_velocity[2], down_speed_m_s=velocity[2],
                speed_m_s=math.hypot(*velocity[:2]), north_error_m=north_error, east_error_m=east_error, down_error_m=down_error,
                pitch_ref_rad_s=signals["rate_ref_frd_rad_s"][1], pitch_rad_s=signals["gyro_frd_rad_s"][1],
                yaw_ref_rad_s=signals["rate_ref_frd_rad_s"][2], yaw_rad_s=signals["gyro_frd_rad_s"][2],
                motor_min=min(motors, default=None), motor_mean=sum(motors)/len(motors) if motors else None, motor_max=max(motors, default=None),
                acceleration_request_m_s2=math.hypot(*acceleration[:2]), acceleration_limit_m_s2=acceleration_limit,
                roll_ref_rad_s=signals["rate_ref_frd_rad_s"][0], roll_rad_s=signals["gyro_frd_rad_s"][0],
                pose_age_s=pose_age, rate_age_s=rate_age)
