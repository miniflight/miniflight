"""Camera/IMU gate flights, replay and post-flight scoring."""

import argparse
import base64
from contextlib import closing
import hashlib
import json
import math
from pathlib import Path
import subprocess
import shutil
import tempfile
import time

from pymavlink.dialects.v20 import common as mavlink

from miniflight.replay import Calls
from target.aigp import native
from target.aigp.client import HeartbeatCadence
from target.aigp.experiments.recording import ProcessClient, recording

from .controller import ALLOWED, SETTINGS
from .course import WireClient, execute


HERE = Path(__file__).resolve().parent


class ResetClient(ProcessClient):
    def _receive(self, message, now):
        if message.get_type() not in ALLOWED:
            return None
        return super()._receive(message, now)

    def poll(self, timeout=0.):
        return tuple(packet for packet in super().poll(timeout) if packet.data.get_type() in ALLOWED)


def reset_ack(path):
    """Reset using transport acknowledgements, without reading native race state."""
    with recording(path,dict(process='sensor_reset_ack')) as record:
        calls=Calls(record);client=ResetClient(calls,camera_port=None);heartbeat=HeartbeatCadence()
        client.open();requested=-float('inf');deadline=time.monotonic()+120
        try:
            while time.monotonic()<deadline:
                packets=client.poll(.02);now=time.monotonic();heartbeat.send_if_due(client,now)
                if client.connected and now-requested>=1.:
                    client.reset();requested=now
                for packet in packets:
                    message=packet.data
                    if message.get_type()=='COMMAND_ACK' and message.command==31000:
                        if message.result!=0:raise RuntimeError('native reset rejected')
                        return
            raise TimeoutError('reset was not acknowledged')
        finally:client.disconnect()


def replay(path, poison=False):
    previous=dict(SETTINGS)
    try:return _replay(path,poison)
    finally:
        SETTINGS.clear();SETTINGS.update(previous)


def _replay(path, poison=False):
    with path.open() as source:metadata=json.loads(source.readline())["metadata"]
    settings=metadata.get('settings',{})
    if (metadata.get('process')!='public_camera_imu_course' or not settings.get('continuous_course')
            or settings.get('route_prior') is not None):
        revision=metadata.get('revision','unknown')
        raise ValueError(f'Archived instrumented or route-snapshot replay is unsupported; use recorded revision {revision}')
    SETTINGS.clear();SETTINGS.update(settings)
    SETTINGS["acquisition_close_gaps"] = metadata["settings"].get("acquisition_close_gaps", False)
    SETTINGS["acquisition_largest_rim"] = metadata["settings"].get("acquisition_largest_rim", False)
    SETTINGS["lookahead_enabled"] = metadata["settings"].get("lookahead_enabled", False)
    SETTINGS["lookahead_during_final"] = metadata["settings"].get("lookahead_during_final", False)
    SETTINGS["height_alignment"] = metadata["settings"].get("height_alignment", False)
    SETTINGS.update(metadata["settings"])
    SETTINGS['continuous_course'] = metadata['settings'].get('continuous_course',False)
    SETTINGS['command_epoch'] = metadata['settings'].get('command_epoch','imu')
    SETTINGS['ground_support'] = metadata['settings'].get('ground_support',False)
    SETTINGS['gyro_attitude'] = metadata['settings'].get('gyro_attitude',False)
    SETTINGS['joint_rim'] = metadata['settings'].get('joint_rim',False)
    SETTINGS['direct_launch'] = metadata['settings'].get('direct_launch',False)
    SETTINGS['continuous_path'] = metadata['settings'].get('continuous_path',False)
    SETTINGS['position_config']=metadata['settings'].get('position_config')
    SETTINGS['visual_initialization']=metadata['settings'].get('visual_initialization',False)
    SETTINGS['partial_gate']=metadata['settings'].get('partial_gate',False)
    updates = [row for row in map(json.loads, path.open()) if row["event"] == "update"]
    outcomes = [row for row in map(json.loads, path.open()) if row["event"] == "result"]
    decoder = mavlink.MAVLink(None)
    encoder = mavlink.MAVLink(None)
    changed = 0
    def rows():
        nonlocal changed
        outgoing = []
        for row in map(json.loads, path.open()):
            if row["event"] == "outgoing":
                outgoing.append(row["wire"])
            elif row["event"] == "call":
                if poison and row["name"] == "poll":
                    for packet in row.get("result", []):
                        if packet["camera"]:
                            continue
                        message, = decoder.parse_buffer(base64.b64decode(packet["wire"]))
                        kind = message.get_type()
                        if kind == "LOCAL_POSITION_NED":
                            message.x, message.y, message.z = 12345., -9876., 4567.
                            message.vx, message.vy, message.vz = 200., -300., 400.
                        elif kind == "ATTITUDE":
                            message.roll, message.pitch, message.yaw = 2., -1.5, .75
                        elif SETTINGS.get('continuous_course') and kind=='ODOMETRY':
                            message.x,message.y,message.z=11111.,-22222.,33333.
                            message.vx,message.vy,message.vz=999.,-888.,777.
                            message.q=[.5,.5,.5,.5]
                        elif SETTINGS.get('continuous_course') and kind=='DATA_TRANSMISSION_HANDSHAKE':
                            message.size,message.packets,message.width,message.height=2_000_000_000,50000,50000,50000
                        elif SETTINGS.get('continuous_course') and kind=='ENCAPSULATED_DATA':
                            message.data=bytearray([message.data[0]]+[255]*(len(message.data)-1))
                        elif SETTINGS.get('continuous_course') and kind=='ACTUATOR_OUTPUT_STATUS':
                            message.actuator=[1234.]*len(message.actuator)
                        elif SETTINGS.get('continuous_course') and kind=='COLLISION':
                            message.id,message.action,message.threat_level=65535,0,0
                        else:
                            continue
                        packet["wire"] = base64.b64encode(message.pack(encoder)).decode("ascii")
                        changed += 1
                if poison and row["name"] == "race":
                    for race in row.get("result", []):
                        race["gate_centers"] = [[9999., -9999., 9999.]]*len(race.get("gate_centers", []))
                        race["active_gate_index"] = 0
                        if SETTINGS.get('continuous_course'):
                            race.update(started=False,valid=False,completed=True,time_seconds=-123.,finish_time_seconds=-456.)
                if row["name"] == "send" and "error" not in row:
                    if len(outgoing) != 1:
                        raise AssertionError("send does not own one outgoing packet")
                    client.expect_wire_command(outgoing[0])
                outgoing.clear()
                yield row
    recorded = iter(rows())
    calls = Calls(rows=recorded)
    client=WireClient(calls,record_camera=True)
    def compare_updates(**row):
        if row.get("event") == "update":
            expected = updates.pop(0)
            actual = json.loads(json.dumps(row))
            if "signals" in expected:
                actual["signals"] = {key: actual["signals"][key] for key in expected["signals"]}
            omitted = {"host_time", "native_race"} if poison else {"host_time"}
            assert {k:v for k,v in expected.items() if k not in omitted} == {k:v for k,v in actual.items() if k not in omitted}, "controller update differs"
    factory=None
    if metadata.get('fixture') is not None:
        from .unknown_start import Controller
        factory=lambda calls:Controller(calls,metadata['fixture'])
    outcome, client = execute(calls, compare_updates, path.with_name("native.jsonl"), client=client,controller_factory=factory)
    calls.finish()
    assert not updates
    expected = {key:outcomes[-1][key] for key in outcome}
    compared = json.loads(json.dumps(outcome))
    if expected["final_signal"] is not None:
        compared["final_signal"] = {key: compared["final_signal"][key] for key in expected["final_signal"]}
    assert compared == expected, (compared, expected)
    return dict(calls=calls.count, wire_commands=client.wire_commands, exact_updates=True,
                corrupted_pose_packets=changed,corrupted_navigation_packets=changed, native_gate_counter_zeroed=poison, outcome=outcome)


def public_course(root,shift,fixture=None,scoring_frame=None):
    """Run the ordinary camera/MAVLink interface; no capture/IMU DLLs are loaded."""
    if SETTINGS.get('position_config') is None:raise ValueError('Public course requires a publicly identified dynamics model')
    root.mkdir(parents=True)
    SETTINGS['shift_right_m']=shift
    sources=tuple(sorted(HERE.glob('*.py')))+tuple(HERE.parents[3]/'miniflight'/name for name in ['gate_filter.py','relocalization.py'])
    metadata=dict(process='public_camera_imu_course',revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        settings=SETTINGS,shift_right_m=shift,position_used_for='offline scoring',native_taps_loaded=False,
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    if fixture is not None:metadata['fixture']=fixture
    (root/'protocol.json').write_text(json.dumps(metadata,indent=2)+'\n')
    for source in sources:
        dest=root/'source'/source.name;dest.parent.mkdir(exist_ok=True);dest.write_bytes(source.read_bytes())
    with native.launch('vq1.r1',('-RenderOffScreen',),race_status_path=root/'native.jsonl'), closing(native.RaceStateReader(root/'native.jsonl',.3)) as reader:
        reset_ack(root/'reset.jsonl')
        with recording(root/'flight.jsonl',metadata) as record:
            count=0
            def bounded(**row):
                nonlocal count
                record(**row);count+=1
                if count%64==0 and (root/'flight.jsonl').stat().st_size>SETTINGS.get('trace_limit_mib',384)*1024**2:
                    raise RuntimeError('temporary public-sensor trace exceeded its size limit')
            factory=None
            if fixture is not None:
                from .unknown_start import Controller
                factory=lambda calls:Controller(calls,fixture)
            outcome,_=execute(Calls(bounded),bounded,root/'native.jsonl',reader,controller_factory=factory)
            if outcome.get('estimated_map') is not None:
                (root/'estimated-map.json').write_text(json.dumps(outcome['estimated_map'],indent=2)+'\n')
            print(json.dumps(outcome),flush=True)
    from .pipeline import export_public
    export_public(root)
    if scoring_frame is not None:shutil.copyfile(scoring_frame,root/'score-frame.json')
    from .score import score
    result=score(root)
    if SETTINGS.get('render_video',True) and any(g.get('first_lock_time_s') is not None for g in result['gates']):
        from .video import render
        render(root)
    checked=replay(root/'flight.jsonl')
    (root/'replay.json').write_text(json.dumps(checked,indent=2)+'\n')
    from .pipeline import analyze
    analyze(root)
    return result


def run_course(root, shift, gates=6, speed=2., dynamics_model=None,partial_gate=False,crossing_constraint=False,joint_estimator=False,brake_test=False,moving_start=False,excitation=None,gyro_attitude=False,joint_rim=False,direct_launch=False,continuous_path=False,image_time_uncertainty=False,upright_attitude=False,map_prior=None,unknown_start=False,scoring_frame=None,camera_gap_s=0.,view_recovery=False):
    """Public course; remove pixels and raw traces after scoring and replay."""
    if dynamics_model is None:raise ValueError('Public course requires a publicly identified dynamics model')
    if not math.isfinite(shift):raise ValueError('The sideways setup shift must be finite')
    if not math.isfinite(camera_gap_s) or not 0<=camera_gap_s<3.:raise ValueError('Camera gaps must be in[0,3)s')
    if unknown_start and (map_prior is None or scoring_frame is None or not joint_estimator):raise ValueError('Unknown start needs joint estimation, a learned map and a separate scoring frame')
    fixture=dict(target=[5.,shift,3.],yaw=.18 if shift>=0 else -.18,delay_s=8.) if unknown_start else None
    calibration=dynamics_model.read_bytes()
    position_config=json.loads(calibration)['position_config']
    prior=None
    if map_prior is not None:
        from . import vision
        data=json.loads(map_prior.read_text())
        if (not joint_estimator or data['frame']!='Initial stationary camera/IMU local frame' or data['source']!='Public pixels, IMU and own commands only'
                or data['dynamics_model_sha256']!=hashlib.sha256(calibration).hexdigest()
                or data['camera_from_body']!=vision.CAMERA_FROM_BODY.tolist() or data['intrinsics']!=vision.K.tolist()):raise ValueError('Map calibration or launch frame does not match')
        prior=data['posterior']
    root.mkdir(parents=True)
    original = dict(SETTINGS)
    SETTINGS.update(view_recovery=view_recovery,camera_gap_s=camera_gap_s,thrust_excitation=excitation,render_video=excitation is None,gyro_attitude=gyro_attitude,joint_rim=joint_rim,direct_launch=direct_launch,continuous_path=continuous_path,image_time_uncertainty=image_time_uncertainty,upright_attitude=upright_attitude,map_prior=prior)
    SETTINGS.update(continuous_course=True,sensor_interface='public_camera_and_HIGHRES_IMU',camera_correction=True,maximum_blind_s=4.5,
        gates_to_pass=gates,speed_m_s=speed,
        beyond_m=4.,finish_beyond_m=1.,flight_timeout_s=30.,course_timeout_s=180.,trace_limit_mib=384,
        camera_position_min_side_px=20.,camera_velocity_min_side_px=20.,
        vertical_speed_limit_m_s=3.,
        camera_window_s=1.,
        startup_allowance_s=8.,
        visibility_height_change_m=1.5,
        motion_prediction='command_model',
        visual_gravity=True,
        lookahead_enabled=True,lookahead_during_final=True,height_alignment=True,
        control_inputs='camera pixels, public gyro/accelerometer, public timestamps and arrivals; no native taps or race state')
    SETTINGS.update(position_config=position_config,visual_initialization=True,partial_gate=partial_gate,
        crossing_constraint=crossing_constraint,
        dynamics_model_sha256=hashlib.sha256(calibration).hexdigest())
    SETTINGS.update(joint_estimator=joint_estimator,brake_test=brake_test)
    if joint_estimator:SETTINGS.update(camera_correction=False,visual_gravity=False,continuous_origin=True,moving_start=moving_start,command_epoch='public_send',ground_support=True)
    if brake_test:SETTINGS['finish_beyond_m']=1.
    if excitation:SETTINGS.update(flight_timeout_s=90.,course_timeout_s=110.)
    try:
        with tempfile.TemporaryDirectory(prefix="raw-", dir=root) as temporary:
            raw = Path(temporary)/"flight"
            try:
                result = public_course(raw,0. if unknown_start else shift,fixture,scoring_frame)
                checked = replay(raw/"flight.jsonl", poison=True)
                checked.pop("outcome")
                (root/"input-independence.json").write_text(json.dumps(checked, indent=2)+"\n")
                return result
            except Exception as error:
                (root/"failure.json").write_text(json.dumps(dict(type=type(error).__name__, message=str(error)), indent=2)+"\n")
                raise
            finally:
                for name in ("protocol.json", "score.json", "replay.json", "camera-errors.csv",
                             'course-camera-imu.mp4','video.json','state-errors.csv','last-camera.jpg','control.jsonl.gz','score-frame.json'):
                    if (raw/name).exists():
                        shutil.copyfile(raw/name, root/name)
                for name in ('pipeline.json','public-inputs.npz','public-observations.jsonl.gz','pipeline-timing.csv.gz','client-latency.csv.gz','architecture-score.json','estimated-map.json','estimator-inputs.jsonl.gz','estimator-states.jsonl.gz','native-reference.npz'):
                    if (raw/name).exists():shutil.copyfile(raw/name,root/name)
                if fixture is not None and (raw/'source').exists():shutil.copytree(raw/'source',root/'source',dirs_exist_ok=True)
                size = sum(path.stat().st_size for path in raw.rglob("*") if path.is_file())
                (root/"retention.json").write_text(json.dumps(dict(temporary_bytes_removed=size, raw_recordings_retained=False))+"\n")
    finally:
        SETTINGS.clear()
        SETTINGS.update(original)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("course", "replay", "score", "video"))
    parser.add_argument("path", type=Path)
    parser.add_argument("--shift", type=float, default=-.5)
    parser.add_argument('--crossing-constraint',action='store_true')
    parser.add_argument('--joint-estimator',action='store_true');parser.add_argument('--brake-test',action='store_true')
    parser.add_argument('--moving-start',action='store_true')
    parser.add_argument('--gyro-attitude',action='store_true')
    parser.add_argument('--joint-rim',action='store_true')
    parser.add_argument('--direct-launch',action='store_true')
    parser.add_argument('--continuous-path',action='store_true')
    parser.add_argument('--image-time-uncertainty',action='store_true',help='Shared10ms exposure-time uncertainty prior; not a measured delay')
    parser.add_argument('--upright-attitude',action='store_true',help='Use upright edges for tilt; partition their pixel information once')
    parser.add_argument('--thrust-excitation',choices=('train','validate'))
    parser.add_argument("--poison", action="store_true")
    parser.add_argument('--gates',type=int,choices=range(1,7),default=6)
    parser.add_argument('--speed',type=float,default=2.)
    parser.add_argument('--dynamics-model',type=Path)
    parser.add_argument('--map-prior',type=Path,help='Joint public map from the same stationary launch frame')
    parser.add_argument('--unknown-start',action='store_true',help='Displace with an independent pilot, then cold-localize without its state')
    parser.add_argument('--scoring-frame',type=Path,help='Fixed learned-map alignment, opened only by the postflight scorer')
    parser.add_argument('--view-recovery',action='store_true',help='Experimental brake/align/reacquire controller; not validated for complete laps')
    parser.add_argument('--camera-gap',type=float,default=0.,help='Withhold this many camera seconds every3s, starting1s after visual tracking')
    parser.add_argument('--partial-gate',action='store_true')
    args = parser.parse_args()
    if args.mode=='course':
        if args.dynamics_model is None:parser.error('course requires --dynamics-model; see dynamics.model.json for the public-data calibration')
        if not math.isfinite(args.shift):parser.error('The sideways setup shift must be finite')
        if not 0<args.speed<=(12. if args.continuous_path else 8.):parser.error('course speed exceeds the selected controller limit')
        if args.brake_test and (not args.joint_estimator or args.gates!=1):parser.error('brake test requires --joint-estimator --gates 1')
        if args.thrust_excitation and (not args.joint_estimator or args.gates!=1):parser.error('thrust excitation requires --joint-estimator --gates 1')
        if args.direct_launch and not args.joint_estimator:parser.error('direct launch requires --joint-estimator')
        if args.image_time_uncertainty and not args.joint_estimator:parser.error('image-time uncertainty requires --joint-estimator')
        if args.upright_attitude and not (args.joint_estimator and args.gyro_attitude):parser.error('upright attitude requires joint estimation with other attitude corrections held')
        if args.continuous_path and (not args.joint_estimator or args.thrust_excitation):parser.error('continuous path requires joint estimation without excitation')
        result=run_course(args.path.resolve(),args.shift,args.gates,args.speed,args.dynamics_model,args.partial_gate,args.crossing_constraint,args.joint_estimator,args.brake_test,args.moving_start,args.thrust_excitation,args.gyro_attitude,args.joint_rim,args.direct_launch,args.continuous_path,args.image_time_uncertainty,args.upright_attitude,args.map_prior,args.unknown_start,args.scoring_frame,args.camera_gap,args.view_recovery)
        from .plots import plot
        if (args.path/'state-errors.csv').exists():plot(args.path.resolve())
        if (args.path/'pipeline.json').exists():
            from .pipeline import render
            render(args.path.resolve())
        if args.joint_estimator:
            from .gate_filter_score import score_live
            score_live(args.path.resolve())
        if not result['passed_without_reset'] and not (args.thrust_excitation and result['termination']=='public thrust excitation complete' and result['controller_error'] is None):
            raise SystemExit(1)
    elif args.mode == "replay":
        print(json.dumps(replay(args.path.resolve(), args.poison)), flush=True)
    elif args.mode == "score":
        from .score import score
        if not score(args.path.resolve())["passed_without_reset"]:
            raise SystemExit(1)
    else:
        from .video import render
        render(args.path.resolve())
