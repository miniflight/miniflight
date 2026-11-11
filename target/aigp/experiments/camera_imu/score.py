"""Score two native gate crossings without feeding pose to the controller."""

import base64
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from pymavlink.dialects.v20 import common
from scipy.spatial.transform import Rotation

from .controller import ALLOWED
from .geometry import gate_openings
from .accuracy import score_estimates


def score(root):
    path = root/"flight.jsonl"
    decoder = common.MAVLink(None)
    positions, attitudes, ticks, race, collisions, arms, outcomes = [], [], [], [], [], [], []
    accepted = set()
    resets = 0
    settings = json.loads(path.open().readline())["metadata"]["settings"]
    observations, previous_image, host_offset = [], None, None
    for row in map(json.loads, path.open()):
        if row["event"] == "call":
            if row["name"] == "reset":
                resets += 1
            elif row["name"] == "arm":
                arms.append(row["arguments"]["armed"])
            elif row["name"] == "race":
                race.extend(row.get("result", []))
            elif row["name"] == "producer_imu" and row.get("result", {}).get("rows"):
                batch = row["result"]
                host_offset = batch["published_ns"]*1e-9-batch["rows"][-1][0]*1e-6
            elif row["name"] == "poll":
                for packet in row.get("result", []):
                    if packet["camera"]:
                        continue
                    for message in decoder.parse_buffer(base64.b64decode(packet["wire"])) or ():
                        kind = message.get_type()
                        if kind == "LOCAL_POSITION_NED":
                            positions.append([message.time_boot_ms*.001, message.x, message.y, message.z,
                                              message.vx, message.vy, message.vz])
                        elif kind == "ATTITUDE":
                            attitudes.append([message.time_boot_ms*.001, message.roll, -message.pitch, -message.yaw])
                        elif kind == "COLLISION":
                            collisions.append(dict(host=packet["received_at"], data=message.to_dict()))
        elif row["event"] == "update":
            for packet in row["telemetry"]:
                message, = common.MAVLink(None).parse_buffer(base64.b64decode(packet["wire"]))
                accepted.add(message.get_type())
            if row.get("signals"):
                signal = row["signals"]
                assert not signal["motion_available"] and not signal["attitude_available"]
                ticks.append(signal)
                observed = signal.get("camera_observation")
                if observed is not None and observed["image_id"] != previous_image:
                    previous_image = observed["image_id"]
                    observations.append(dict(observed, capture_to_output_s=row["host_time"]-host_offset-observed["capture_time"]))
        elif row["event"] == "result":
            outcomes.append(row)
    assert accepted <= ALLOWED
    p = np.asarray(positions)
    clock_regressions = int(np.sum(np.diff(p[:, 0]) < 0))
    p = p[np.r_[True, np.diff(p[:, 0]) > 0]]
    position_at = lambda t: np.array([np.interp(t, p[:, 0], col) for col in p[:, 1:4].T])
    requested = settings.get("gates_to_pass", 2)
    openings = gate_openings()[:requested]
    local_to_native = Rotation.from_euler("z", attitudes[0][3]).as_matrix()
    approach = [tick for tick in ticks if tick["phase"] in ("TRACKING", "FINAL_IMU", "HANDOFF")]
    histories = {tick["imu_history_start"] for tick in approach}
    samples = [tick["imu_history_samples"] for tick in approach]
    if approach:
        assert len(histories) == 1 and np.all(np.diff(samples) >= 0)
    handoffs = [tick for tick in ticks if tick["phase"] == "HANDOFF"]
    result = dict(controller_error=outcomes[-1]["error"], controller_phase=outcomes[-1]["phase"],
        native_gates_passed=max((state["active_gate_index"] for state in race), default=0),
        valid_race=outcomes[-1]["race"]["valid"], collision_packets=collisions,
        reset_calls_during_flight=resets, arm_calls=arms, native_pose_clock_regressions=clock_regressions,
        controller_packet_types=sorted(accepted), filtered_packets=outcomes[-1]["filtered_packets"],
        continuous_imu_history=len(histories) == 1, handoff_time_s=handoffs[0]["imu_time"] if handoffs else None,
        trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), gates=[])
    for number, opening in enumerate(openings, 1):
        center = opening.mean(axis=0)
        width, vertical = opening[1]-opening[0], opening[3]-opening[0]
        width /= np.linalg.norm(width)
        vertical /= np.linalg.norm(vertical)
        normal = np.cross(width, vertical)
        basis = np.array([width, vertical, normal])
        signed = (p[:, 1:4]-center) @ normal
        indices = [i for i in np.flatnonzero(signed[:-1]*signed[1:] < 0) if np.linalg.norm(p[i, 1:4]-center) < 2.]
        native = [state for state in race if state["active_gate_index"] >= number]
        gate = dict(gate=number, native_pass=bool(native and native[0]["valid"]), crossing_imu_time_s=None)
        gate_ticks = [tick for tick in ticks if tick.get("gate_number") == number and tick["phase"] in ("TRACKING", "FINAL_IMU")]
        locked = [tick for tick in gate_ticks if tick["phase"] == "TRACKING"]
        if locked:
            gate["first_lock_time_s"] = locked[0]["imu_time"]
            gate["first_lock_image_id"] = locked[0]["image_id"]
        if indices:
            i = indices[0]
            crossing = float(p[i, 0]+(p[i+1, 0]-p[i, 0])*signed[i]/(signed[i]-signed[i+1]))
            point = position_at(crossing)
            gate.update(crossing_imu_time_s=crossing, crossing_position_ned_m=point.tolist(),
                crossing_offset_width_down_m=(basis @ (point-center))[:2].tolist())
            final = [tick for tick in gate_ticks if tick["phase"] == "FINAL_IMU"]
            previous = [tick for tick in final if tick["imu_time"] <= crossing]
            if previous:
                assert len({tick["image_id"] for tick in final}) == 1
                assert len({tick["accepted_images"] for tick in final}) == 1
                tick = previous[-1]
                estimate = local_to_native @ (np.array(tick["gate_center"])-tick["position"])
                truth = center-position_at(tick["imu_time"])
                gate.update(last_image_id=tick["image_id"], last_image_time_s=tick["image_time"],
                    blind_time_to_crossing_s=crossing-tick["image_time"],
                    final_phase_to_crossing_s=crossing-final[0]["imu_time"],
                    camera_updates_during_final=0,
                    estimated_gate_error_width_down_normal_m=(basis @ (estimate-truth)).tolist())
        result["gates"].append(gate)
    passed = all(gate["native_pass"] and gate["crossing_imu_time_s"] is not None for gate in result["gates"])
    result["passed_without_reset"] = (passed and result["controller_error"] is None
        and result["controller_phase"] == "COMPLETE" and result["valid_race"] and not collisions
        and resets == 0 and arms == [True, False] and clock_regressions == 0 and len(histories) == 1)
    if requested == 2:
        result["passed_two_without_reset"] = result["passed_without_reset"]
    if requested == 2 and result["passed_without_reset"]:
        result["time_between_crossings_s"] = result["gates"][1]["crossing_imu_time_s"]-result["gates"][0]["crossing_imu_time_s"]
    if settings.get("camera_correction"):
        result["estimation"] = score_estimates(root, p, approach, openings[0].mean(axis=0), local_to_native, observations)
    (root/"score.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    score(Path(sys.argv[1]))
