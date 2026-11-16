"""Score two native gate crossings without feeding pose to the controller."""

import base64
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from pymavlink.dialects.v20 import common
from scipy.spatial.transform import Rotation, Slerp

from .controller import ALLOWED
from .geometry import gate_openings
from .accuracy import score_estimates, statistics
from .vision import CAMERA_FROM_BODY


def scored_gate_passes(gates,completed_finish=False):
    """The simulator can freeze at its finish volume before publishing a centre crossing."""
    return bool(gates) and all(gate['native_pass'] and
        (gate['crossing_imu_time_s'] is not None or (completed_finish and index==len(gates)-1))
        for index,gate in enumerate(gates))


def armed_contacts(collisions,armed_at,disarmed_at):
    if armed_at is None:return list(collisions)
    return [c for c in collisions if c['host']>=armed_at and (disarmed_at is None or c['host']<=disarmed_at)]


def score(root):
    path = root/"flight.jsonl"
    decoder = common.MAVLink(None)
    positions, attitudes, ticks, race, collisions, arms, outcomes = [], [], [], [], [], [], []
    rates,motors=[],[]
    accepted = set()
    resets = 0
    armed_at,disarmed_at=None,None
    settings = json.loads(path.open().readline())["metadata"]["settings"]
    observations, previous_image, host_offset = [], None, None
    for row in map(json.loads, path.open()):
        if row["event"] == "call":
            if row["name"] == "reset":
                resets += 1
            elif row["name"] == "arm":
                arms.append(row["arguments"]["armed"])
                if row['arguments']['armed'] and armed_at is None:armed_at=row['host_time']
                elif not row['arguments']['armed']:disarmed_at=row['host_time']
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
                            rates.append([message.time_boot_ms*.001,-message.rollspeed,-message.pitchspeed,-message.yawspeed])
                        elif kind=='ACTUATOR_OUTPUT_STATUS':
                            motors.append([message.time_usec*1e-6,*message.actuator[:4]])
                        elif kind == "COLLISION":
                            collisions.append(dict(host=packet["received_at"], data=message.to_dict()))
        elif row["event"] == "update":
            for packet in row["telemetry"]:
                message, = common.MAVLink(None).parse_buffer(base64.b64decode(packet["wire"]))
                accepted.add(message.get_type())
            if row.get("signals"):
                signal = dict(row["signals"],motor_commands=(row.get('command') or {}).get('outputs'))
                assert not signal["motion_available"] and not signal["attitude_available"]
                ticks.append(signal)
                observed = signal.get("camera_observation")
                if observed is not None and observed["image_id"] != previous_image:
                    previous_image = observed["image_id"]
                    age = row['host_time']-host_offset-observed['capture_time'] if host_offset is not None else signal['imu_time']-observed['capture_time']
                    observations.append(dict(observed,gate_number=signal.get('gate_number',1),capture_to_output_s=age))
        elif row["event"] == "result":
            outcomes.append(row)
    assert accepted <= ALLOWED
    # Private references stay in the scorer; estimator exports never open this file.
    np.savez_compressed(root/'native-reference.npz',position_velocity=np.asarray(positions).reshape(-1,7),
        attitude=np.asarray(attitudes).reshape(-1,4),body_rates=np.asarray(rates).reshape(-1,4),motor_outputs=np.asarray(motors).reshape(-1,5))
    if not positions or not attitudes:
        outcome=outcomes[-1] if outcomes else {}
        result=dict(scoring_status='missing_native_reference',passed_without_reset=False,gates=[],
            native_gates_passed=None,native_completed=None,native_finish_time_s=None,valid_race=None,
            estimated_gates_passed=outcome.get('estimated_gates_passed'),controller_error=outcome.get('error'),
            controller_phase=outcome.get('phase'),arm_calls=arms,reset_calls_during_flight=resets,
            continuous_imu_history=False,in_flight_collision_packets=armed_contacts(collisions,armed_at,disarmed_at))
        (root/'score.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    if settings.get('continuous_course'):
        import gzip
        fields=('phase','imu_time','image_id','image_time','position','velocity','camera_orientation','gate_center','remaining_m','target',
                'gate_number','estimated_gates_passed','visibility_alignment','alignment_relative_gate','alignment_hint','search_hint','camera_observation',
                'motor_commands','alignment_height_step','gyro','rate_request','rate_integral','visual_gravity','partial_gate',
                'full_image_time','partial_retargets','aperture_corners_measured','measurement_time','association_counts','crossing','crossing_alignment',
                'joint_posterior','brake_test_active','brake_test_state','launch_cleared','excitation','visual_outage','reference')
        with gzip.open(root/'control.jsonl.gz','wt') as stream:
            for tick in ticks:
                stream.write(json.dumps({key:tick[key] for key in fields if key in tick},separators=(',',':'))+'\n')
    p = np.asarray(positions)
    clock_regressions = int(np.sum(np.diff(p[:, 0]) < 0))
    p = p[np.r_[True, np.diff(p[:, 0]) > 0]]
    position_at = lambda t: np.array([np.interp(t, p[:, 0], col) for col in p[:, 1:4].T])
    requested = settings.get("gates_to_pass", 2)
    openings = gate_openings()[:requested]
    local_to_native = Rotation.from_euler("z", attitudes[0][3]).as_matrix()
    scored_phases=("SHIFTING","TRACKING","FINAL_IMU","HANDOFF") if settings.get('continuous_origin') else ("TRACKING","FINAL_IMU","HANDOFF")
    approach = [tick for tick in ticks if tick["phase"] in scored_phases]
    histories = {tick["imu_history_start"] for tick in approach}
    samples = [tick["imu_history_samples"] for tick in approach]
    if approach:
        assert len(histories) == 1 and np.all(np.diff(samples) >= 0)
    handoffs = [tick for tick in ticks if tick["phase"] == "HANDOFF"]
    result = dict(controller_error=outcomes[-1]["error"], controller_phase=outcomes[-1]["phase"],
        native_gates_passed=max((state["active_gate_index"] for state in race), default=0),
        valid_race=bool(race and all(state['valid'] for state in race if state['started'])), collision_packets=collisions,
        reset_calls_during_flight=resets, arm_calls=arms, native_pose_clock_regressions=clock_regressions,
        controller_packet_types=sorted(accepted), filtered_packets=outcomes[-1]["filtered_packets"],
        continuous_imu_history=len(histories) == 1, handoff_time_s=handoffs[0]["imu_time"] if handoffs else None,
        trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), gates=[])
    result.update(native_completed=any(state['completed'] for state in race),
        native_finish_time_s=next((state['finish_time_seconds'] for state in race if state['completed']),None),
        estimated_gates_passed=outcomes[-1].get('estimated_gates_passed',max((tick.get('estimated_gates_passed',0) for tick in ticks),default=0)),
        termination=outcomes[-1].get('termination'))
    flight_contacts=armed_contacts(collisions,armed_at,disarmed_at)
    result.update(armed_host_time=armed_at,disarmed_host_time=disarmed_at,
        in_flight_collision_packets=flight_contacts,prearm_collision_packets=[c for c in collisions if armed_at is not None and c['host']<armed_at])
    result['final_native_state']=dict(time=float(p[-1,0]),position_ned=p[-1,1:4].tolist(),velocity_ned=p[-1,4:7].tolist())
    result['prediction_bias']=outcomes[-1].get('prediction_bias')
    trim=lambda s:{k:s[k] for k in ['started','valid','completed','time_seconds','active_gate_index','received_at']}
    result['race_audit']=dict(first=trim(race[0]) if race else None,
        first_invalid=next((trim(s) for s in race if not s['valid']),None),
        first_running=next((trim(s) for s in race if s['started'] and s['time_seconds']>=0),None),
        last=trim(race[-1]) if race else None,armed_boot_time=outcomes[-1].get('armed_boot_time'))
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
    native_finish=bool(settings.get('continuous_course') and requested==6 and result['native_completed'] and result['valid_race']
                       and result['native_gates_passed']>=6 and result['estimated_gates_passed']==6)
    result['finish_verified_by_native_event']=native_finish
    passed = scored_gate_passes(result['gates'],native_finish)
    if settings.get('continuous_course') and requested==6:
        passed=passed and native_finish
    result["passed_without_reset"] = (passed and result["controller_error"] is None
        and result["controller_phase"] == "COMPLETE" and result["valid_race"] and not (flight_contacts if settings.get('continuous_course') else collisions)
        and resets == 0 and arms == [True, False] and clock_regressions == 0 and len(histories) == 1)
    if requested == 2:
        result["passed_two_without_reset"] = result["passed_without_reset"]
    if requested == 2 and result["passed_without_reset"]:
        result["time_between_crossings_s"] = result["gates"][1]["crossing_imu_time_s"]-result["gates"][0]["crossing_imu_time_s"]
    if settings.get("camera_correction"):
        result["estimation"] = score_estimates(root, p, approach, openings.mean(axis=1), local_to_native, observations)
        result['estimation']['capture_age_uses_estimated_sensor_clock']=settings.get('continuous_course',False)
    if settings.get('continuous_course') and approach:
        a=np.array(attitudes);a=a[np.r_[True,np.diff(a[:,0])>0]]
        orientations=Slerp(a[:,0],Rotation.from_euler('xyz',a[:,1:4]))
        origin=position_at(approach[0]['imu_history_start'])
        rows=[]
        for tick in approach:
            stamp=tick['imu_time']
            estimated=local_to_native @ tick['position']+origin
            actual=position_at(stamp)
            velocity=np.array([np.interp(stamp,p[:,0],col) for col in p[:,4:7].T])
            body=local_to_native @ np.array(tick['camera_orientation']) @ CAMERA_FROM_BODY
            actual_body=orientations(np.clip(stamp,a[0,0],a[-1,0])).as_matrix()
            rows.append(dict(time=stamp,gate=tick.get('gate_number'),
                position_error_m=float(np.linalg.norm(estimated-actual)),
                velocity_error_m_s=float(np.linalg.norm(local_to_native @ tick['velocity']-velocity)),
                attitude_error_deg=float(np.degrees(Rotation.from_matrix(body.T @ actual_body).magnitude()))))
            for name,values in [('estimated_position',estimated),('native_position',actual),
                                ('estimated_velocity',local_to_native @ tick['velocity']),('native_velocity',velocity)]:
                rows[-1].update({f'{name}_{axis}':float(value) for axis,value in zip('ned',values)})
            for name,rotation in [('estimated_attitude',body),('native_attitude',actual_body)]:
                rows[-1].update({f'{name}_{axis}_deg':float(value) for axis,value in
                    zip(('roll','pitch','yaw'),Rotation.from_matrix(rotation).as_euler('xyz',degrees=True))})
        with (root/'state-errors.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        result['local_state_errors']={key:statistics([row[key] for row in rows]) for key in ['position_error_m','velocity_error_m_s','attitude_error_deg']}
        result['state_alignment']='One fixed yaw/origin alignment at stationary start; no scale fit.'
    (root/"score.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    score(Path(sys.argv[1]))
