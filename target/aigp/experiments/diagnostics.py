"""Convert recorded AIGP inputs and sent commands into ordinary SI quantities."""

import math
import base64
from pymavlink.dialects.v20 import common as mavlink
from miniflight.diagnostics import chart_data


PANELS = (
    ("Horizontal speed", "m/s", (("Reference", "speed_ref_m_s"), ("Observed", "speed_m_s"))),
    ("Position error: reference minus observed", "m", (("North", "north_error_m"), ("East", "east_error_m"), ("Down", "down_error_m"))),
    ("Pitch rate in FRD", "rad/s", (("Requested", "pitch_ref_rad_s"), ("Observed", "pitch_rad_s"))),
    ("Sent motor requests", "fraction", (("Minimum", "motor_min"), ("Mean", "motor_mean"), ("Maximum", "motor_max"))),
    ("Horizontal acceleration request before limits", "m/s²", (("Requested", "acceleration_request_m_s2"), ("Limit", "acceleration_limit_m_s2"))),
    ("Accepted sample age at command send", "s", (("Pose", "pose_age_s"), ("IMU", "imu_age_s"))),
)


def snapshot(observer, samples, source):
    race, metadata = observer.status, observer.metadata
    clock = race.get("finish_time_seconds") if race.get("completed") else race.get("time_seconds")
    reference = (metadata.get("reference") or {}).get("duration_s")
    seconds = lambda value: "—" if value is None else f"{value:.3f} s"
    return dict(source=str(source), title=f"{metadata.get('target', 'Waiting')} · {source.name}",
                summary={"Native finish" if race.get("completed") else "Native clock": seconds(clock),
                         "Reference": seconds(reference), "Gates": race.get("active_gate_index", "—"),
                         "Collisions": observer.collisions, "Sent motor commands": len(samples)},
                charts=chart_data(samples, PANELS, x_label="Reference time (s)"))


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
                if kind not in ("LOCAL_POSITION_NED", "HIGHRES_IMU"):
                    continue
                previous = self.latest.get(kind)
                stamp = message.time_usec if kind == "HIGHRES_IMU" else message.time_boot_ms
                if previous is not None:
                    old = previous[0]
                    old_stamp = old.time_usec if kind == "HIGHRES_IMU" else old.time_boot_ms
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
        _, imu_at = self.latest["HIGHRES_IMU"]
        config = self.metadata.get("controller_config", {})
        sample = quantities(self.pending["signals"], (motion.x,motion.y,motion.z), (motion.vx,motion.vy,motion.vz),
                            row["command"]["outputs"], row["host_time"]-pose_at, row["host_time"]-imu_at,
                            config.get("max_acceleration"))
        self.pending = None
        return sample


def quantities(signals, position, velocity, motors, pose_age, imu_age, acceleration_limit=None):
    target_position = signals["position_ref_ned_m"]
    target_velocity = signals["velocity_ref_ned_m_s"]
    north_error, east_error, down_error = (wanted-actual for wanted, actual in zip(target_position, position, strict=True))
    acceleration = signals["acceleration_request_ned_m_s2"]
    return dict(time_s=signals["reference_time_s"], speed_ref_m_s=math.hypot(*target_velocity[:2]),
                speed_m_s=math.hypot(*velocity[:2]), north_error_m=north_error, east_error_m=east_error, down_error_m=down_error,
                pitch_ref_rad_s=signals["rate_ref_frd_rad_s"][1], pitch_rad_s=signals["gyro_frd_rad_s"][1],
                motor_min=min(motors), motor_mean=sum(motors)/len(motors), motor_max=max(motors),
                acceleration_request_m_s2=math.hypot(*acceleration[:2]), acceleration_limit_m_s2=acceleration_limit,
                pose_age_s=pose_age, imu_age_s=imu_age)
