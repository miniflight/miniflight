"""Privileged native pose scoring for the live joint-turn experiment only."""
import base64
import json
from pathlib import Path
import struct
import subprocess

import cv2
import numpy as np
from pymavlink.dialects.v20 import common

from miniflight.joint import solve_window
from miniflight.initialization import fit_gate_motion
from miniflight.state import Frame
from target.aigp.client import _Camera
from .sensors import ALLOWED


def rendered_views(path):
    """Follow view creation -> render -> surface read -> UDP image identity."""
    blob = path.read_bytes()
    count = struct.unpack_from('<Q', blob)[0]
    assert count < (len(blob)-4096)//512
    records = []
    for i in range(count):
        offset = 4096+i*512
        d = struct.unpack_from('<16Q', blob, offset)
        if d[15]:
            records.append((d, blob[offset+128:offset+512]))
    records.sort(key=lambda row: row[0][2])
    views, created, running, rendered, readbacks, output, captures = {}, {}, {}, {}, {}, {}, {}
    for d, raw in records:
        kind, obj, other, identity = d[0], d[7], d[8], d[11]
        if kind == 1:
            origin = struct.unpack('<d', struct.pack('<Q', d[5]))[0]
            scale = struct.unpack('<d', struct.pack('<Q', d[6]))[0]
            captures[identity] = d[4], origin, scale
        elif kind == 8:
            low, origin, scale = captures[identity]
            capture_time = .5*(low+d[2])*scale+16777216-origin
            views[identity] = (np.array(struct.unpack_from('<3f', raw)),
                np.array(struct.unpack_from('<16f', raw, 16)).reshape(4, 4)[:3, :3], capture_time)
        elif kind == 9:
            created[obj] = d
        elif kind == 10:
            renderer, target = struct.unpack_from('<2Q', raw)
            if renderer in created:
                assert created[renderer][8] == target+0x50
                running[renderer] = created[renderer][11]
        elif kind == 11 and obj in running:
            rendered[other+0x50] = running.pop(obj)
        elif kind == 3:
            readbacks[other] = (obj, None)
        elif kind == 7:
            resource, array = struct.unpack_from('<2Q', raw)
            if array in readbacks:
                assert readbacks[array][0] == resource
                readbacks[array] = (resource, rendered.get(resource))
        elif kind == 4 and other in readbacks:
            resource, identity = readbacks.pop(other)
            if identity in views:
                output[d[10], d[9]] = views[identity]
    return output


def score(root):
    path = root/'flight.jsonl'
    decoder = common.MAVLink(None)
    positions, updates, calls, arms, collisions, results, initializations = [], [], [], [], [], [], []
    accepted, resets, offset = set(), 0, None
    for row in map(json.loads, path.open()):
        if row['event'] == 'call':
            name = row['name']
            if name == 'producer_imu' and row.get('result', {}).get('rows'):
                batch = row['result']
                offset = batch['published_ns']*1e-9-batch['rows'][-1][0]*1e-6
            elif name == 'poll':
                for packet in row.get('result', []):
                    if packet['camera']:
                        continue
                    for message in decoder.parse_buffer(base64.b64decode(packet['wire'])) or ():
                        if message.get_type() == 'LOCAL_POSITION_NED':
                            positions.append([message.time_boot_ms*.001,message.x,message.y,message.z,message.vx,message.vy,message.vz])
                        elif message.get_type() == 'COLLISION':
                            collisions.append(message.to_dict())
            elif name == 'arm':
                arms.append(row['arguments']['armed'])
            elif name == 'reset':
                resets += 1
            elif name == 'joint_solve' and 'result' in row:
                calls.append(row)
            elif name == 'joint_gate_fit' and 'result' in row:
                initializations.append(row['result'])
        elif row['event'] == 'update' and row.get('signals'):
            signal = row['signals']
            assert not signal['motion_available'] and not signal['attitude_available']
            for packet in row['telemetry']:
                message, = common.MAVLink(None).parse_buffer(base64.b64decode(packet['wire']))
                accepted.add(message.get_type())
            if signal.get('control_estimate_source') == 'joint_camera_imu':
                updates.append(dict(host=row['host_time'], output_time=row['host_time']-offset, signal=signal))
        elif row['event'] == 'result':
            results.append(row)
    assert accepted <= ALLOWED
    numerical_errors = []
    for row in calls:
        replayed = solve_window(row['result']['request'])
        recorded = row['result']['fit']
        error = max(float(np.max(abs(np.array(replayed[key])-recorded[key]))) for key in ['states','depths'])
        numerical_errors.append(error)
        assert error < 1e-5, 'joint numerical replay differs'
    result = dict(controller_error=results[-1]['error'], controller_phase=results[-1]['phase'],
        joint_control_updates=len(updates), joint_solves=len(calls), controller_packets=sorted(accepted),
        arm_calls=arms, resets=resets, collisions=collisions,
        numerical_replay_max_difference=max(numerical_errors,default=0.), passed_joint_turn=False, passed_gate_return=False)
    replay_data = [dict(arguments=row['arguments'], request=row['result']['request'], fit=row['result']['fit']) for row in calls]
    (root/'joint-replay.json').write_text(json.dumps(replay_data,indent=2)+'\n')
    if initializations:
        result['initialization'] = score_initializations(root,initializations,positions)
    if not updates:
        (root/'score.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        return result
    views = rendered_views(root/'events.bin')
    by_id = {key[0]:value for key,value in views.items()}
    first = updates[0]['signal']['joint']
    _, native_rotation, reference_time = by_id[first['reference_id']]
    camera_to_native = native_rotation @ np.diag([1.,-1.,1.])
    data = np.array(positions)
    data = data[np.r_[True,np.diff(data[:,0])>0]]
    at = lambda t,start:np.array([np.interp(t,data[:,0],column) for column in data[:,start:start+3].T])
    origin = at(reference_time,1)
    native = lambda p:(p-origin)*[1.,1.,-1.] @ camera_to_native
    trajectory = []
    for row in updates:
        signal = row['signal'];joint = signal['joint'];stamp = signal['imu_time']
        q = np.array(joint['reference_rotation']);pref = np.array(joint['reference_position'])
        actual = native(at(stamp,1));velocity = at(stamp,4)*[1.,1.,-1.] @ camera_to_native
        target = q.T @ (np.array(signal['target'])-pref)
        output_actual = native(at(row['output_time'],1))
        feedback = 1.2*(np.array(signal['target'])-signal['position'])+2.*(np.array(signal['target_velocity'])-signal['velocity'])
        np.testing.assert_allclose(feedback,signal['feedback_acceleration'],atol=1e-10)
        trajectory.append(dict(time=stamp,elapsed=joint['elapsed_s'],estimated=joint['position_camera'],
            estimated_velocity=joint['velocity_camera'],actual=actual.tolist(),actual_velocity=velocity.tolist(),target=target.tolist(),
            position_error_m=float(np.linalg.norm(np.array(joint['position_camera'])-actual)),
            velocity_error_m_s=float(np.linalg.norm(np.array(joint['velocity_camera'])-velocity)),
            tracking_error_m=float(np.linalg.norm(output_actual-target)),
            input_age_s=float(row['output_time']-stamp),visual_age_s=float(row['output_time']-joint['solution_time']),
            solutions=joint['solutions'],depth_revisions=joint['depth_revisions'],depth_change=joint['depth_change'],
            solve_ms=joint['solve_ms'],points=joint['points']))
    turn = [row for row in trajectory if 1. <= row['elapsed'] <= 3.]
    if len(turn) < 2:
        result['joint_turn_samples'] = len(turn)
        (root/'joint-trajectory.json').write_text(json.dumps(trajectory,indent=2)+'\n')
        (root/'score.json').write_text(json.dumps(result,indent=2)+'\n')
        render(root,updates,views,trajectory)
        print(json.dumps(result,indent=2),flush=True)
        return result
    rms = lambda key:float(np.sqrt(np.mean([row[key]**2 for row in turn])))
    first_velocity = at(updates[0]['signal']['imu_time'],4)[:2]
    heading = []
    for row in updates:
        if not 1. <= row['signal']['joint']['elapsed_s'] <= 3.:
            continue
        velocity = at(row['signal']['imu_time'],4)[:2]
        heading.append(float(np.degrees(np.arctan2(first_velocity[0]*velocity[1]-first_velocity[1]*velocity[0],np.dot(first_velocity,velocity)))))
    clock_errors = [abs(row['arguments']['capture_ns']*1e-9-by_id[row['arguments']['image_id']][2]) for row in calls]
    gaps = np.diff([row['host'] for row in updates])
    end = trajectory[-1]
    result.update(position_rms_m=rms('position_error_m'),velocity_rms_m_s=rms('velocity_error_m_s'),
        tracking_rms_m=rms('tracking_error_m'),maximum_tracking_error_m=max(row['tracking_error_m'] for row in turn),
        heading_change_deg=max(map(abs,heading)),final_actual_speed_m_s=float(np.linalg.norm(end['actual_velocity'])),
        final_tracking_error_m=end['tracking_error_m'],depth_revisions=end['depth_revisions'],depth_change=end['depth_change'],
        solve_ms_min_median_p95_max=np.percentile([row['solve_ms'] for row in trajectory],[0,50,95,100]).tolist(),
        maximum_command_update_gap_s=float(max(gaps,default=0.)),
        maximum_capture_clock_error_s=max(clock_errors,default=0.),
        input_age_ms_median_p95=np.percentile([row['input_age_s']*1000 for row in trajectory],[50,95]).tolist(),
        gate_pose_last_used_time=first['gate_pose_frozen_at'],joint_control_start_time=updates[0]['signal']['imu_time'])
    result['maximum_turn_speed_m_s'] = max(np.linalg.norm(row['actual_velocity']) for row in turn)
    result['joint_solves_during_turn'] = turn[-1]['solutions']-turn[0]['solutions']
    result['depth_revisions_during_turn'] = turn[-1]['depth_revisions']-turn[0]['depth_revisions']
    gate_times = {row['signal']['joint']['gate_pose_frozen_at'] for row in updates}
    result['gate_pose_frozen'] = len(gate_times)==1 and first['gate_pose_frozen_at']<updates[0]['signal']['imu_time']-.1
    slow_turn = (result['controller_error'] is None and result['controller_phase']=='COMPLETE'
        and len(turn)>=40 and len(calls)>=8 and end['depth_revisions']>=4
        and result['position_rms_m']<.35 and result['velocity_rms_m_s']<.25
        and result['maximum_tracking_error_m']<.6 and 12.<result['heading_change_deg']<35.
        and result['final_actual_speed_m_s']<.3 and not collisions and arms==[True,False] and resets==0
        and result['final_tracking_error_m']<.6
        and result['maximum_turn_speed_m_s']<1.5 and result['joint_solves_during_turn']>=8
        and result['depth_revisions_during_turn']>=4 and result['gate_pose_frozen']
        and result['maximum_command_update_gap_s']<.15 and result['maximum_capture_clock_error_s']<.001)
    returning = 'gate_updates' in first
    result['passed_joint_turn'] = slow_turn and not returning
    if returning:
        from .gate_return_score import score as score_return
        result['gate_return'] = score_return(root,calls,updates,trajectory,native)
        result['gate_measurements_frozen'] = False
        result['passed_gate_return'] = slow_turn and result['gate_return']['passed_recovery']
    (root/'joint-trajectory.json').write_text(json.dumps(trajectory,indent=2)+'\n')
    (root/'score.json').write_text(json.dumps(result,indent=2)+'\n')
    render(root,updates,views,trajectory,calls)
    print(json.dumps(result,indent=2),flush=True)
    return result


def score_initializations(root, initializations, positions):
    views = {key[0]:value for key,value in rendered_views(root/'events.bin').items()}
    data = np.array(positions)
    data = data[np.r_[True,np.diff(data[:,0])>0]]
    at = lambda stamp,start:np.array([np.interp(stamp,data[:,0],column) for column in data[:,start:start+3].T])
    scored = []
    for item in initializations:
        request, recorded = item['request'], item['result']
        replayed = fit_gate_motion(request)
        assert (replayed is None)==(recorded is None)
        difference = None
        if replayed is not None:
            difference = max(float(np.max(abs(np.array(replayed[key])-recorded[key]))) for key in ('velocity','center'))
            assert difference < 1e-5, 'initialization numerical replay differs'
        meta = request['meta']
        q = views[meta['reference_id']][1] @ np.diag([1.,-1.,1.])
        reference_time, cut_time = views[meta['reference_id']][2], views[meta['cut_id']][2]
        true_position = (at(cut_time,1)-at(reference_time,1))*[1.,1.,-1.] @ q
        true_velocity = at(cut_time,4)*[1.,1.,-1.] @ q
        row = dict(reference_time=reference_time,cut_time=cut_time,corner_views=len(request['times']),
            linear_position_error_m=float(np.linalg.norm(np.array(meta['linear_position'])-true_position)),
            linear_velocity_error_m_s=float(np.linalg.norm(np.array(meta['linear_velocity'])-true_velocity)),
            numerical_replay_difference=difference,candidate_valid=recorded is not None)
        if recorded is not None:
            velocity=np.array(recorded['velocity']);dt=meta['duration'];gravity=np.array(meta['gravity'])
            position=velocity*dt+meta['delta_position']+.5*gravity*dt**2
            velocity=velocity+meta['delta_velocity']+gravity*dt
            row.update(candidate_position_error_m=float(np.linalg.norm(position-true_position)),
                candidate_velocity_error_m_s=float(np.linalg.norm(velocity-true_velocity)),
                reprojection_rms_px=recorded['reprojection_rms_px'],fit_ms=recorded['fit_ms'])
        scored.append(row)
    (root/'joint-initialization.json').write_text(json.dumps(dict(records=initializations,scores=scored),indent=2)+'\n')
    return scored


def render(root, updates, views, trajectory, calls=()):
    start = updates[0]['signal']['imu_time']-.3
    end = min(updates[-1]['signal']['imu_time'], start+5.)
    signals = {}
    gate_pixels = {row['arguments']['image_id']:row['result']['request']['frames'][-1].get('gate_pixels',[]) for row in calls}
    returning = 'gate_updates' in updates[0]['signal']['joint']
    gate_updates = 0
    for row in updates:
        signals.setdefault(row['signal']['joint']['image_id'],row['signal'])
    camera = _Camera()
    command = ['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','640x432',
        '-r','30','-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(root/'joint-flight.mp4')]
    with subprocess.Popen(command,stdin=subprocess.PIPE) as writer:
        saved = 0
        for row in map(json.loads,(root/'flight.jsonl').open()):
            if row['event']!='call' or row['name']!='poll':continue
            for packet in row.get('result',[]):
                if not packet['camera']:continue
                frame = camera.receive(base64.b64decode(packet['wire']),packet['received_at'])
                if not isinstance(frame,Frame) or (frame.id,frame.time_ns) not in views:continue
                stamp = views[frame.id,frame.time_ns][2]
                if not start<=stamp<=end:continue
                image = frame.bgr.copy();signal=signals.get(frame.id)
                if signal is not None:
                    info=signal['joint'];q=np.array(info['reference_rotation']);current=np.array(info['camera_rotation'])
                    gate_updates = info.get('gate_updates',0)
                    points=np.column_stack((info['measured_points'],np.ones(len(info['measured_points']))))
                    pixels=points @ np.linalg.inv(np.array([[320.,0,320],[0,320,180],[0,0,1.]])).T @ q.T @ current
                    pixels=pixels @ np.array([[320.,0,320],[0,320,180],[0,0,1.]]).T
                    for point in pixels[:,:2]/pixels[:,2:]:cv2.circle(image,tuple(np.round(point).astype(int)),2,(100,235,140),1,cv2.LINE_AA)
                    if gate_pixels.get(frame.id):
                        points=np.column_stack((gate_pixels[frame.id],np.ones(4))) @ np.linalg.inv(np.array([[320.,0,320],[0,320,180],[0,0,1.]])).T @ q.T @ current
                        points=points @ np.array([[320.,0,320],[0,320,180],[0,0,1.]]).T
                        corners=np.round(points[:,:2]/points[:,2:]).astype(np.int32)
                        cv2.polylines(image,[corners],True,(230,210,80),2,cv2.LINE_AA)
                canvas=cv2.copyMakeBorder(image,42,30,0,0,cv2.BORDER_CONSTANT,value=(22,25,30))
                cv2.putText(canvas,'joint camera + imu feedback',(12,26),0,.65,(100,235,140),1,cv2.LINE_AA)
                label='background pose, velocity and depths update during flight'
                if returning:
                    label='gate observations withheld; background + imu continue' if not gate_updates else f'mapped gate observations used: {gate_updates}'
                cv2.putText(canvas,label,(12,419),0,.42,(215,222,230),1,cv2.LINE_AA)
                writer.stdin.write(canvas.tobytes());saved+=1
                if saved==30:cv2.imwrite(str(root/'joint-preview.png'),canvas)
        writer.stdin.close();assert writer.wait()==0


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Numerically replay retained joint optimizer inputs.')
    parser.add_argument('path',type=Path)
    args=parser.parse_args()
    path=args.path/'joint-replay.json' if args.path.is_dir() else args.path
    rows=json.loads(path.read_text());errors=[]
    for row in rows:
        result=solve_window(row['request'])
        errors.append(max(float(np.max(abs(np.array(result[key])-row['fit'][key]))) for key in ('states','depths')))
    maximum=max(errors,default=0.)
    assert maximum<1e-5
    initializations=0
    if args.path.is_dir() and (args.path/'joint-initialization.json').exists():
        for item in json.loads((args.path/'joint-initialization.json').read_text())['records']:
            result=fit_gate_motion(item['request']);expected=item['result']
            assert (result is None)==(expected is None)
            if result is not None:
                np.testing.assert_allclose(result['velocity'],expected['velocity'],atol=1e-5)
            initializations+=1
    if args.path.is_dir() and (args.path/'gate-return.json').exists():
        from .gate_return_score import replay_without_gate
        expected=json.loads((args.path/'gate-return.json').read_text())['baseline']
        for fit,recorded in zip(replay_without_gate(rows),expected,strict=True):
            np.testing.assert_allclose(fit['states'][-1],recorded['state'],atol=1e-5,rtol=0)
            np.testing.assert_allclose(fit['depths'],recorded['depths'],atol=1e-5,rtol=0)
    print(json.dumps(dict(solves=len(rows),initializations=initializations,maximum_numeric_difference=maximum)))
