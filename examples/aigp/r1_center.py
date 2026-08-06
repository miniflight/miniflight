"""Fly the fixed R1 gate centers with Miniflight and VQ1 metric state."""

import csv
from collections import deque
from datetime import datetime
import math
from pathlib import Path
import struct
import time

from pymavlink import mavutil

from common.math import Quaternion
from miniflight.camera import PinholeCamera
from miniflight.core.observation import MetricState, ObservationTick
from miniflight.estimate import NedMahonyEstimator
from miniflight.program import BodyRates
from miniflight.race import ThreadGatesProgram
from target.vq1 import Vq1Link

from r1 import TrackTransfer, request
from vision import GateCamera, solve_square_translation


CONTROL_HZ = 250.0
MAX_POSE_AGE_S = 0.1
MAX_RUN_S = 240.0
MAX_SPEED_MPS = 8.0
MAX_TILT_RAD = math.radians(60.0)
RACE_STATUS_ID = 1
CAMERA_UPTILT_RAD = math.radians(20.0)
GATE_APERTURE_SIZE_M = 1.5
CAMERA_MODEL = PinholeCamera(
    focal_x_px=320.0,
    focal_y_px=320.0,
    center_x_px=320.0,
    center_y_px=180.0,
    camera_to_body=(
        (0.0, math.sin(CAMERA_UPTILT_RAD), math.cos(CAMERA_UPTILT_RAD)),
        (1.0, 0.0, 0.0),
        (0.0, math.cos(CAMERA_UPTILT_RAD), -math.sin(CAMERA_UPTILT_RAD)),
    ),
)
VISION_FIELDS = (
    "vision_frame",
    "vision_detected",
    "vision_aperture_valid",
    "vision_candidates",
    "vision_target_rank",
    "vision_largest_error_px",
    "vision_capture_age_ms",
    "vision_pose_delta_ms",
    "vision_cx_px",
    "vision_cy_px",
    "vision_span_px",
    "vision_gt_cx_px",
    "vision_gt_cy_px",
    "vision_gt_depth_m",
    "vision_error_x_px",
    "vision_error_y_px",
    "vision_est_body_x",
    "vision_est_body_y",
    "vision_est_body_z",
    "vision_gt_body_x",
    "vision_gt_body_y",
    "vision_gt_body_z",
    "vision_error_body_x",
    "vision_error_body_y",
    "vision_error_body_z",
    "vision_error_body_m",
)
PATCH_VERSION = Path(__file__).with_name("VERSION").read_text().strip()

def norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def rotate_body_to_ned(quaternion, vector):
    magnitude = norm(quaternion)
    if magnitude <= 1e-9:
        raise ValueError("quaternion must be nonzero")
    w, x, y, z = (value / magnitude for value in quaternion)
    quaternion_vector = (x, y, z)
    twice_cross = tuple(2.0 * value for value in cross(quaternion_vector, vector))
    second_cross = cross(quaternion_vector, twice_cross)
    return tuple(
        vector[index] + w * twice_cross[index] + second_cross[index]
        for index in range(3)
    )


def rotate_ned_to_body(quaternion, vector):
    w, x, y, z = quaternion
    return rotate_body_to_ned((w, -x, -y, -z), vector)


def audit_gate_frame(frame, pose_history, target):
    values = {name: math.nan for name in VISION_FIELDS}
    values["vision_frame"] = frame.frame_id
    values["vision_detected"] = int(frame.detected)
    values["vision_candidates"] = len(frame.candidates)
    values["vision_capture_age_ms"] = (
        time.monotonic_ns() - frame.capture_monotonic_ns
    ) / 1_000_000.0
    if not pose_history:
        return values

    pose_time_ns, state, raw_quaternion = min(
        pose_history,
        key=lambda item: abs(item[0] - frame.capture_monotonic_ns),
    )
    values["vision_pose_delta_ms"] = (
        pose_time_ns - frame.capture_monotonic_ns
    ) / 1_000_000.0
    truth_ned = tuple(
        target[axis] - state.position_ned_m[axis]
        for axis in range(3)
    )
    rendered_attitude = camera_attitude(raw_quaternion)
    truth_body = rotate_ned_to_body(rendered_attitude, truth_ned)
    values.update((
        ("vision_gt_body_x", truth_body[0]),
        ("vision_gt_body_y", truth_body[1]),
        ("vision_gt_body_z", truth_body[2]),
    ))
    try:
        truth_x_px, truth_y_px, truth_depth_m = CAMERA_MODEL.project_body_vector(truth_body)
    except ValueError:
        return values
    if not frame.detected:
        return values
    target_rank = min(
        range(len(frame.candidates)),
        key=lambda index: math.hypot(
            frame.candidates[index].center_px[0] - truth_x_px,
            frame.candidates[index].center_px[1] - truth_y_px,
        ),
    )
    selected = frame.candidates[target_rank]
    center_px = selected.center_px
    vertical_span_px = selected.vertical_span_px
    largest_center_px = frame.candidates[0].center_px
    values.update((
        ("vision_aperture_valid", int(selected.aperture_corners_px is not None)),
        ("vision_target_rank", target_rank),
        (
            "vision_largest_error_px",
            math.hypot(
                largest_center_px[0] - truth_x_px,
                largest_center_px[1] - truth_y_px,
            ),
        ),
        ("vision_cx_px", center_px[0]),
        ("vision_cy_px", center_px[1]),
        ("vision_span_px", vertical_span_px),
        ("vision_gt_cx_px", truth_x_px),
        ("vision_gt_cy_px", truth_y_px),
        ("vision_gt_depth_m", truth_depth_m),
        ("vision_error_x_px", center_px[0] - truth_x_px),
        ("vision_error_y_px", center_px[1] - truth_y_px),
    ))
    estimate_camera = solve_square_translation(
        selected,
        GATE_APERTURE_SIZE_M,
        CAMERA_MODEL.focal_x_px,
        CAMERA_MODEL.focal_y_px,
        CAMERA_MODEL.center_x_px,
        CAMERA_MODEL.center_y_px,
    )
    if estimate_camera is None:
        return values
    estimate_body = CAMERA_MODEL.camera_vector_to_body(estimate_camera)
    estimate_error = tuple(
        estimate_body[axis] - truth_body[axis]
        for axis in range(3)
    )
    values.update((
        ("vision_est_body_x", estimate_body[0]),
        ("vision_est_body_y", estimate_body[1]),
        ("vision_est_body_z", estimate_body[2]),
        ("vision_error_body_x", estimate_error[0]),
        ("vision_error_body_y", estimate_error[1]),
        ("vision_error_body_z", estimate_error[2]),
        ("vision_error_body_m", norm(estimate_error)),
    ))
    return values


def yaw_of(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def euler_of(quaternion):
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch, yaw_of(quaternion)


def camera_attitude(raw_quaternion):
    """Return the body-to-NED attitude that matches the rendered camera."""
    raw_roll, raw_pitch, raw_yaw = euler_of(raw_quaternion)
    quaternion = Quaternion.from_euler(-raw_roll, raw_pitch, -raw_yaw)
    return tuple(float(value) for value in quaternion.q)


def aperture_center(track_gate):
    _, position, orientation, _, height, valid = track_gate
    if not valid:
        raise RuntimeError("invalid gate pose")
    vertical = rotate_body_to_ned(orientation, (0.0, 0.0, 1.0))
    return tuple(position[axis] - 0.5 * height * vertical[axis] for axis in range(3))


def course_normal(track_gate):
    _, _, orientation, _, _, valid = track_gate
    if not valid:
        raise RuntimeError("invalid gate pose")
    return rotate_body_to_ned(orientation, (0.0, 1.0, 0.0))


def race_status(message):
    if message.get_type() != "ENCAPSULATED_DATA":
        return None
    payload = bytes(message.data)
    if not payload or payload[0] != RACE_STATUS_ID:
        return None
    return struct.unpack_from("<BQqqIq", payload)


def arm(link, value):
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        value,
        0, 0, 0, 0, 0, 0,
    )


def is_armed(heartbeat):
    return bool(
        heartbeat
        and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def run():
    print(f"R1 patch v{PATCH_VERSION}", flush=True)
    link = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise RuntimeError("no VQ1 heartbeat")
    request(link, mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 250)

    boot_ms = int(time.time() * 1000)
    vq1 = Vq1Link(link, boot_ms, mavutil.mavlink)
    track = TrackTransfer()
    camera = None
    try:
        camera = GateCamera()
        print("R1 shadow camera listening on 5600", flush=True)
    except OSError as error:
        print(f"R1 shadow camera unavailable {error}", flush=True)
    print("R1 attached to live simulator", flush=True)
    print("R1 waiting for official track transfer", flush=True)
    log_dir = Path(__file__).with_name("logs")
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"ep_r1_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    fields = (
        "patch_version", "t", "sim_time_s", "race_sim_boot_ms", "race_start_ms",
        "start_margin_ms", "active_gate", "collisions",
        "px", "py", "pz", "vx", "vy", "vz",
        "body_vx", "body_vy", "body_vz",
        "qw", "qx", "qy", "qz", "roll", "pitch", "yaw",
        "raw_qw", "raw_qx", "raw_qy", "raw_qz",
        "raw_roll", "raw_pitch", "raw_yaw",
        "est_qw", "est_qx", "est_qy", "est_qz",
        "est_roll", "est_pitch", "est_yaw", "est_attitude_error_deg",
        "est_accel_ref_x", "est_accel_ref_y", "est_accel_ref_z",
        "est_gyro_bias_x", "est_gyro_bias_y", "est_gyro_bias_z",
        "body_p", "body_q", "body_r",
        "raw_body_p", "raw_body_q", "raw_body_r",
        "imu_time_s", "imu_ax", "imu_ay", "imu_az",
        "imu_gx", "imu_gy", "imu_gz",
        "target_x", "target_y", "target_z", "distance_m",
        "target_body_x", "target_body_y", "target_body_z",
        *VISION_FIELDS,
        "actuator_0", "actuator_1", "actuator_2", "actuator_3",
    )

    latest_state = None
    latest_body_velocity = None
    latest_raw_quaternion = None
    latest_raw_body_rates = None
    latest_imu = (math.nan,) * 6
    latest_imu_time_s = math.nan
    latest_seen_at = None
    latest_heartbeat = heartbeat
    latest_actuators = (math.nan, math.nan, math.nan, math.nan)
    active_gate = None
    collisions = 0
    race_sim_boot_ms = None
    race_start_ms = None
    program = None
    program_gate = None
    target = None
    target_body = None
    flight_command = None
    sequence = 0
    armed_by_us = False
    arm_sent_at = 0.0
    run_started_at = None
    pose_history = deque(maxlen=240)
    vision_frame = None
    vision_audit = {name: math.nan for name in VISION_FIELDS}
    attitude_estimator = NedMahonyEstimator(initial_yaw_rad=math.pi)

    try:
        with log_path.open("w", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(fields)
            next_tick = time.monotonic()
            while True:
                new_state = False
                message = link.recv_match(blocking=False)
                while message is not None:
                    kind = message.get_type()
                    had_track = bool(track.gates)
                    track.add(message)
                    if track.gates and not had_track:
                        print(f"TRACK gates={len(track.gates)}", flush=True)
                        for gate in track.gates:
                            print(
                                f"POSE id={gate[0]} base={tuple(round(x, 3) for x in gate[1])} "
                                f"center={tuple(round(x, 3) for x in aperture_center(gate))} "
                                f"normal={tuple(round(x, 3) for x in course_normal(gate))} "
                                f"size=({gate[3]:.3f}, {gate[4]:.3f})",
                                flush=True,
                            )
                    if kind == "HEARTBEAT":
                        latest_heartbeat = message
                    elif kind == "COLLISION":
                        collisions += 1
                    elif kind == "ACTUATOR_OUTPUT_STATUS":
                        latest_actuators = tuple(float(value) for value in message.actuator[:4])
                    elif kind == "HIGHRES_IMU":
                        latest_imu_time_s = float(message.time_usec) / 1_000_000.0
                        latest_imu = (
                            float(message.xacc),
                            float(message.yacc),
                            float(message.zacc),
                            float(message.xgyro),
                            float(message.ygyro),
                            float(message.zgyro),
                        )
                        attitude_estimator.update(
                            int(message.time_usec) * 1000,
                            latest_imu[:3],
                            latest_imu[3:],
                        )
                    elif kind == "ODOMETRY":
                        if (
                            int(message.frame_id) != mavutil.mavlink.MAV_FRAME_LOCAL_NED
                            or int(message.child_frame_id) != mavutil.mavlink.MAV_FRAME_BODY_NED
                        ):
                            raise RuntimeError(
                                f"unexpected odometry frames {message.frame_id} {message.child_frame_id}"
                            )
                        raw_quaternion = tuple(float(value) for value in message.q)
                        magnitude = norm(raw_quaternion)
                        raw_quaternion = tuple(value / magnitude for value in raw_quaternion)
                        latest_raw_quaternion = raw_quaternion
                        quaternion = raw_quaternion
                        latest_body_velocity = (message.vx, message.vy, message.vz)
                        latest_raw_body_rates = (
                            message.rollspeed,
                            message.pitchspeed,
                            message.yawspeed,
                        )
                        velocity_ned = rotate_body_to_ned(
                            raw_quaternion,
                            latest_body_velocity,
                        )
                        latest_state = MetricState(
                            timestamp_ns=int(message.time_usec) * 1000,
                            position_ned_m=(message.x, message.y, message.z),
                            velocity_ned_mps=velocity_ned,
                            attitude_body_to_ned=quaternion,
                            body_rates_rad_s=latest_raw_body_rates,
                        )
                        latest_seen_at = time.monotonic()
                        pose_history.append((time.monotonic_ns(), latest_state, raw_quaternion))
                        new_state = True
                    status = race_status(message)
                    if status is not None:
                        status_boot_ms = int(status[1])
                        race_sim_boot_ms = status_boot_ms
                        race_start_ms = int(status[2])
                        active_gate = int(status[4])
                    message = link.recv_match(blocking=False)

                now = time.monotonic()
                pose_fresh = (
                    latest_seen_at is not None
                    and now - latest_seen_at <= MAX_POSE_AGE_S
                )
                race_live = (
                    race_sim_boot_ms is not None
                    and race_start_ms is not None
                    and race_start_ms >= 0
                    and race_sim_boot_ms >= race_start_ms
                )
                if not race_live or latest_state is None or active_gate is None or not track.gates:
                    time.sleep(0.002)
                    continue
                if not pose_fresh:
                    if armed_by_us and now - latest_seen_at > 0.5:
                        raise RuntimeError("metric pose became stale")
                    time.sleep(0.002)
                    continue
                if active_gate >= len(track.gates):
                    print(f"PASS gates={active_gate} log={log_path}", flush=True)
                    return

                if not is_armed(latest_heartbeat):
                    if now - arm_sent_at >= 0.25:
                        arm(link, 1)
                        arm_sent_at = now
                        armed_by_us = True
                    time.sleep(0.002)
                    continue

                if run_started_at is None:
                    run_started_at = now
                    program = ThreadGatesProgram(tuple(
                        aperture_center(gate) for gate in track.gates
                    ))
                    program.start()
                    print(f"R1 Miniflight armed log={log_path}", flush=True)
                if now - run_started_at > MAX_RUN_S:
                    print(f"TIMEOUT gates={active_gate} log={log_path}", flush=True)
                    return

                if program_gate != active_gate:
                    program_gate = active_gate
                    target = aperture_center(track.gates[active_gate])
                    program.select_gate(active_gate)
                    target_body = rotate_ned_to_body(
                        latest_raw_quaternion,
                        tuple(
                            target[axis] - latest_state.position_ned_m[axis]
                            for axis in range(3)
                        ),
                    )
                    print(
                        f"GATE {active_gate} id={track.gates[active_gate][0]} "
                        f"target={tuple(round(x, 3) for x in target)} "
                        "controller=local_position_target",
                        flush=True,
                    )

                if new_state:
                    command_sent = False
                    roll, pitch, yaw = euler_of(latest_state.attitude_body_to_ned)
                    raw_roll, raw_pitch, raw_yaw = euler_of(latest_raw_quaternion)
                    if attitude_estimator.initialized:
                        estimated_attitude = attitude_estimator.attitude_body_to_ned
                        estimated_euler = euler_of(estimated_attitude)
                        estimated_accel_reference = (
                            attitude_estimator.accel_reference_mps2
                        )
                        estimated_gyro_bias = attitude_estimator.gyro_bias_rad_s
                        attitude_dot = abs(sum(
                            estimated_attitude[index] * latest_raw_quaternion[index]
                            for index in range(4)
                        ))
                        attitude_error_deg = math.degrees(
                            2.0 * math.acos(max(-1.0, min(1.0, attitude_dot)))
                        )
                    else:
                        estimated_attitude = (math.nan,) * 4
                        estimated_euler = (math.nan,) * 3
                        estimated_accel_reference = (math.nan,) * 3
                        estimated_gyro_bias = (math.nan,) * 3
                        attitude_error_deg = math.nan
                    speed = norm(latest_state.velocity_ned_mps)
                    if speed > MAX_SPEED_MPS or abs(roll) > MAX_TILT_RAD or abs(pitch) > MAX_TILT_RAD:
                        print(
                            f"ABORT envelope speed={speed:.2f} "
                            f"roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f}",
                            flush=True,
                        )
                        return
                    observation = ObservationTick(
                        sequence=sequence,
                        timestamp_ns=latest_state.timestamp_ns,
                        accel_mps2=(0.0, 0.0, 0.0),
                        gyro_rad_s=latest_state.body_rates_rad_s,
                        source_age_ns=0,
                        metric_state=latest_state,
                    )
                    flight_command = program.step(observation)
                    target_body = rotate_ned_to_body(
                        latest_raw_quaternion,
                        tuple(
                            target[axis] - latest_state.position_ned_m[axis]
                            for axis in range(3)
                        ),
                    )
                    vq1.send_position_ned(flight_command)
                    command_sent = True
                    sequence += 1
                    distance = norm(tuple(
                        target[axis] - latest_state.position_ned_m[axis]
                        for axis in range(3)
                    ))
                    if camera is not None:
                        frame = camera.latest()
                        if frame is not None and frame.frame_id != vision_frame:
                            vision_frame = frame.frame_id
                            vision_audit = audit_gate_frame(
                                frame,
                                pose_history,
                                target,
                            )
                    writer.writerow((
                        PATCH_VERSION,
                        now - run_started_at,
                        latest_state.timestamp_ns / 1_000_000_000.0,
                        race_sim_boot_ms,
                        race_start_ms,
                        race_sim_boot_ms - race_start_ms,
                        active_gate,
                        collisions,
                        *latest_state.position_ned_m,
                        *latest_state.velocity_ned_mps,
                        *latest_body_velocity,
                        *latest_state.attitude_body_to_ned,
                        roll, pitch, yaw,
                        *latest_raw_quaternion,
                        raw_roll, raw_pitch, raw_yaw,
                        *estimated_attitude,
                        *estimated_euler,
                        attitude_error_deg,
                        *estimated_accel_reference,
                        *estimated_gyro_bias,
                        *latest_state.body_rates_rad_s,
                        *latest_raw_body_rates,
                        latest_imu_time_s,
                        *latest_imu,
                        *target,
                        distance,
                        *target_body,
                        *(vision_audit[name] for name in VISION_FIELDS),
                        *latest_actuators,
                    ))
                if (not new_state or not command_sent) and flight_command is not None:
                    vq1.send_position_ned(flight_command)

                next_tick += 1.0 / CONTROL_HZ
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
    finally:
        if camera is not None:
            camera.close()
        if program is not None:
            program.stop("runtime exit")
        neutral = BodyRates(0.0, 0.0, 0.0, 0.0)
        for _ in range(10):
            vq1.send_body_rates(neutral)
            time.sleep(1.0 / CONTROL_HZ)
        if armed_by_us:
            arm(link, 0)
        link.close()
        print("neutral sent and disarm requested", flush=True)


if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("stopped", flush=True)
