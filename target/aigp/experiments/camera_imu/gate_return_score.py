"""Causal gate-off ablation and postflight scoring for mapped-gate recovery."""
import copy
import json

import numpy as np

from miniflight.joint import solve_window
from .geometry import gate_openings


def replay_without_gate(rows):
    """Reuse camera tracks/IMU edges; propagate only this ablation's own states.

    Tracking and image rectification depend on pixels and gyro, not optimized
    position or velocity. The comparison is conditional on the recorded flight.
    """
    fitted, previous, depths, prior = [], {}, None, None
    for row in rows:
        request = copy.deepcopy(row['request'])
        for frame in request['frames']:
            frame['gate_pixels'] = []
        ids = request['frame_ids']
        if previous:
            states = []
            for index, identity in enumerate(ids):
                if identity in previous:
                    states.append(previous[identity])
                else:
                    edge = next(edge for edge in request['edges'] if edge['last'] == index)
                    state = np.array(previous[ids[edge['first']]])
                    dt = edge['dt'];gravity = np.array(request['gravity'])
                    state[:3] += state[3:6]*dt+edge['dp']+.5*gravity*dt**2
                    state[3:6] += edge['dv']+gravity*dt
                    states.append(state.tolist())
            request['states'], request['depths'] = states, depths
            active = {point for frame in request['frames'] for point in frame['ids']}
            for point in range(len(prior)):
                if point not in active:
                    prior[point] = depths[point]
            request['depth_prior'] = list(prior)
        else:
            prior = list(request['depth_prior'])
        result = solve_window(request)
        previous = dict(zip(ids, result['states'], strict=True))
        depths = result['depths']
        fitted.append(result)
    return fitted


def score(root, calls, updates, trajectory, native):
    rows = [dict(request=row['result']['request'],fit=row['result']['fit']) for row in calls]
    baseline = replay_without_gate(rows)
    estimated_gate = np.array(rows[0]['request']['gate_world']).mean(axis=0)
    true_gate = native(gate_openings()[0].mean(axis=0))
    values = []
    for update, sample in zip(updates,trajectory,strict=True):
        joint = update['signal']['joint'];index = joint['solutions']-1
        on, off = np.array(rows[index]['fit']['states'][-1]), np.array(baseline[index]['states'][-1])
        delta = off-on
        off_position = np.array(sample['estimated'])+delta[:3]+delta[3:6]*joint['prediction_dt_s']
        off_velocity = np.array(sample['estimated_velocity'])+delta[3:6]
        truth = np.array(sample['actual'])-true_gate
        error = lambda p: float(np.linalg.norm(p-estimated_gate-truth))
        values.append(dict(time=sample['time'],elapsed=sample['elapsed'],gate_updates=joint['gate_updates'],
            position_error_m=error(np.array(sample['estimated'])),off_position_error_m=error(off_position),
            velocity_error_m_s=sample['velocity_error_m_s'],
            off_velocity_error_m_s=float(np.linalg.norm(off_velocity-sample['actual_velocity']))))
    first = next((u['signal']['joint'] for u in updates if u['signal']['joint']['first_gate_return'] is not None),None)
    metrics = dict(gate_updates=updates[-1]['signal']['joint']['gate_updates'],
        gate_detection_counts=updates[-1]['signal']['joint'].get('gate_detection_counts',{}),
        mapped_gate_center=estimated_gate.tolist(),scored_gate_center=true_gate.tolist(),
        gate_return_time=None if first is None else first['first_gate_return'],
        gate_return_elapsed_s=None if first is None else first['gate_return_elapsed_s'],passed_recovery=False)
    if first is not None:
        returned = first['gate_return_elapsed_s']
        before = [r for r in values if r['elapsed'] < returned]
        eligible = [r for r in values if 0 <= r['elapsed']-returned <= .5]
        recovered = [r for r in eligible if r['position_error_m'] < .15 and r['velocity_error_m_s'] < .1]
        after = [r for r in values if returned+.5 <= r['elapsed'] <= 3.]
        if eligible and after:
            end = eligible[-1]
            rms = lambda key: float(np.sqrt(np.mean([r[key]**2 for r in after])))
            metrics.update(recovery_delay_s=None if not recovered else recovered[0]['elapsed']-returned,
                already_within_bounds_before_return=bool(before and before[-1]['position_error_m']<.15 and before[-1]['velocity_error_m_s']<.1),
                half_second_position_error_m=end['position_error_m'],half_second_velocity_error_m_s=end['velocity_error_m_s'],
                after_return_position_rms_m=rms('position_error_m'),off_position_rms_m=rms('off_position_error_m'),
                after_return_velocity_rms_m_s=rms('velocity_error_m_s'),off_velocity_rms_m_s=rms('off_velocity_error_m_s'),
                final_position_error_m=values[-1]['position_error_m'],off_final_position_error_m=values[-1]['off_position_error_m'],
                passed_recovery=bool(returned>=1. and len(after)>=10 and metrics['gate_updates']>=4 and recovered
                    and end['position_error_m']<.15 and end['velocity_error_m_s']<.1
                    and rms('position_error_m')<.15 and rms('velocity_error_m_s')<.1))
    compact = [dict(state=fit['states'][-1],depths=fit['depths']) for fit in baseline]
    (root/'gate-return.json').write_text(json.dumps(dict(metrics=metrics,baseline=compact,trajectory=values),indent=2)+'\n')
    return metrics
