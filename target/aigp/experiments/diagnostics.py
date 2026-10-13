"""Convert recorded AIGP inputs and sent commands into ordinary SI quantities."""

import math
import base64
from pymavlink.dialects.v20 import common as mavlink
from miniflight.diagnostics import chart_data


PANELS = (
    ("Horizontal speed", "m/s", (("Reference", "speed_ref_m_s"), ("Observed", "speed_m_s"))),
    ("Position error: reference minus observed", "m", (("North", "north_error_m"), ("East", "east_error_m"), ("Down", "down_error_m"))),
    ("Horizontal acceleration request before limits", "m/s²", (("Requested", "acceleration_request_m_s2"), ("Limit", "acceleration_limit_m_s2"))),
    ("Sent motor requests and native reported outputs", "fraction", (("Minimum", "motor_min"), ("Mean", "motor_mean"), ("Maximum", "motor_max"), ("Reported mean", "motor_reported_mean"))),
)
DETAIL_PANELS = (
    ("Vertical velocity: positive down", "m/s", (("Reference", "down_speed_ref_m_s"), ("Observed", "down_speed_m_s"))),
    ("Roll rate in FRD", "rad/s", (("Requested", "roll_ref_rad_s"), ("Observed", "roll_rad_s"))),
    ("Pitch rate in FRD", "rad/s", (("Requested", "pitch_ref_rad_s"), ("Observed", "pitch_rad_s"))),
    ("Accepted sample age at command send", "s", (("Pose", "pose_age_s"), ("IMU", "imu_age_s"))),
    ("IMU boot time minus pose boot time", "s", (("Timestamp difference", "pose_clock_gap_s"),)),
)


def snapshot(observer, samples, source):
    race, metadata = observer.status, observer.metadata
    clock = race.get("finish_time_seconds") if race.get("completed") else race.get("time_seconds")
    reference = (metadata.get("reference") or {}).get("duration_s")
    seconds = lambda value: "—" if value is None else f"{value:.3f} s"
    return dict(source=str(source), title=f"{metadata.get('target', 'Waiting')} · {source.parent.name}/{source.name}",
                summary={"Native finish" if race.get("completed") else "Native clock": seconds(clock),
                         "Reference": seconds(reference), "Gates": race.get("active_gate_index", "—"),
                         "Collisions": observer.collisions, "Sent motor commands": len(samples)},
                charts=chart_data(samples, PANELS, x_label="Reference time (s)") +
                       [dict(chart, detail=True) for chart in chart_data(samples, DETAIL_PANELS, x_label="Reference time (s)")])


class Observe:
    def __init__(self):
        self.decoder = mavlink.MAVLink(None)
        self.latest, self.metadata, self.pending = {}, {}, None
        self.status, self.collisions = {}, 0

    def consume(self, row):
        event = row["event"]
        if event == "config":
            self.metadata = row["metadata"]
        if event == "telemetry" and row["message"]["mavpackettype"] == "COLLISION":
            self.collisions += 1
        if event == "result":
            self.status = row["race"] or {}
        if event == "update":
            self.status = row["native_race"] or {}
            for packet in row["telemetry"]:
                message, = self.decoder.parse_buffer(base64.b64decode(packet["wire"], validate=True))
                kind = message.get_type()
                if kind not in ("LOCAL_POSITION_NED", "HIGHRES_IMU", "ACTUATOR_OUTPUT_STATUS"):
                    continue
                previous = self.latest.get(kind)
                stamp = message.time_boot_ms if kind == "LOCAL_POSITION_NED" else message.time_usec
                if previous is not None:
                    old = previous[0]
                    old_stamp = old.time_boot_ms if kind == "LOCAL_POSITION_NED" else old.time_usec
                    if stamp < old_stamp or message.to_dict() == old.to_dict() or kind == "HIGHRES_IMU" and stamp == old_stamp:
                        continue
                if kind == "HIGHRES_IMU" and not all(math.isfinite(v) for v in (message.xacc,message.yacc,message.zacc,message.xgyro,message.ygyro,message.zgyro)):
                    continue
                self.latest[kind] = (message, packet["received_at"])
            self.pending = row if "signals" in row else None
        if event != "sent" or row["command"]["kind"] != "MotorControls" or self.pending is None:
            return None
        if row["command"] != self.pending["command"]:
            raise ValueError("sent command has no matching controller update")
        motion, pose_at = self.latest["LOCAL_POSITION_NED"]
        imu, imu_at = self.latest["HIGHRES_IMU"]
        config = self.metadata.get("controller_config", {})
        sample = quantities(self.pending["signals"], (motion.x,motion.y,motion.z), (motion.vx,motion.vy,motion.vz),
                            row["command"]["outputs"], row["host_time"]-pose_at, row["host_time"]-imu_at,
                            config.get("max_acceleration"))
        native = self.latest.get("ACTUATOR_OUTPUT_STATUS")
        values = native[0].actuator[:4] if native is not None and native[0].active & 15 == 15 else ()
        sample["motor_reported_mean"] = sum(values)/4 if len(values) == 4 and all(math.isfinite(v) for v in values) else None
        sample["pose_clock_gap_s"] = imu.time_usec/1e6 - motion.time_boot_ms/1e3
        self.pending = None
        return sample


def quantities(signals, position, velocity, motors, pose_age, imu_age, acceleration_limit=None):
    target_position = signals["position_ref_ned_m"]
    target_velocity = signals["velocity_ref_ned_m_s"]
    north_error, east_error, down_error = (wanted-actual for wanted, actual in zip(target_position, position, strict=True))
    acceleration = signals["acceleration_request_ned_m_s2"]
    return dict(time_s=signals["reference_time_s"], speed_ref_m_s=math.hypot(*target_velocity[:2]),
                down_speed_ref_m_s=target_velocity[2], down_speed_m_s=velocity[2],
                speed_m_s=math.hypot(*velocity[:2]), north_error_m=north_error, east_error_m=east_error, down_error_m=down_error,
                pitch_ref_rad_s=signals["rate_ref_frd_rad_s"][1], pitch_rad_s=signals["gyro_frd_rad_s"][1],
                motor_min=min(motors), motor_mean=sum(motors)/len(motors), motor_max=max(motors),
                acceleration_request_m_s2=math.hypot(*acceleration[:2]), acceleration_limit_m_s2=acceleration_limit,
                roll_ref_rad_s=signals["rate_ref_frd_rad_s"][0], roll_rad_s=signals["gyro_frd_rad_s"][0],
                pose_age_s=pose_age, imu_age_s=imu_age)
