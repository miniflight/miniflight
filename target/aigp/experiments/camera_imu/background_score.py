"""Read frozen estimates, then score against native camera poses."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(sys.argv[1]).resolve()
ESTIMATES = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else ROOT
estimate_path = ESTIMATES/'multiview-estimates.json'
estimate = json.loads(estimate_path.read_text())
reference = json.loads((ROOT/'reference-score-only.json').read_text())['frames']
first = estimate['reference_frame']
origin = np.array(reference[first]['view_origin_cm'])
orientation = np.array(reference[first]['camera_to_world'])
times = np.array([f['capture_time_s']-reference[first]['capture_time_s'] for f in reference])
world = np.array([f['view_origin_cm'] for f in reference])*.01
truth = (world-world[first]) @ orientation
cut = estimate['protocol']['calibration_end_frame']
initial = np.array(estimate['shared_initial_position'])
velocity = np.array(estimate['shared_initial_velocity'])
baseline = initial+(times-times[cut])[:, None]*velocity
baseline_error = np.linalg.norm(baseline-truth, axis=1)
score = dict(estimated_before_scoring=True, estimate_sha256=hashlib.sha256(estimate_path.read_bytes()).hexdigest(),
    native_pose_used_only_for_scoring=True, expected_frames=len(reference)-cut-1,
    constant_velocity_rms_m=float(np.sqrt(np.mean(baseline_error[cut+1:]**2))),
    constant_velocity_final_error_m=float(baseline_error[-1]), methods={})
for method, rows in estimate['methods'].items():
    errors = []
    for row in rows:
        if row['status'] != 'estimated':
            continue
        index = row['frame']
        np.testing.assert_allclose(row['constant_velocity_position'], baseline[index], atol=1e-10)
        errors.append(dict(frame=index, elapsed_s=float(times[index]-times[cut]),
            position_error_m=float(np.linalg.norm(np.array(row['position'])-truth[index])),
            constant_velocity_error_m=float(baseline_error[index])))
    good = len(errors) == score['expected_frames']
    rms = float(np.sqrt(np.mean([r['position_error_m']**2 for r in errors]))) if errors else None
    score['methods'][method] = dict(valid_frames=len(errors), complete=good, position_rms_m=rms,
        final_error_m=errors[-1]['position_error_m'] if good else None,
        better_than_constant_velocity=good and rms < score['constant_velocity_rms_m'], rows=errors)
pre = slice(cut-7, cut+1)
before = np.linalg.lstsq(np.column_stack((np.ones(8), times[pre]-times[cut])), world[pre], rcond=None)[0][1]
velocities = []
for index in range(cut+8, len(reference)-3):
    indices = np.arange(index-3, index+4)
    v = np.linalg.lstsq(np.column_stack((np.ones(7), times[indices]-times[index])), world[indices], rcond=None)[0][1]
    a, b = before[:2], v[:2]
    angle = np.degrees(np.arctan2(a[0]*b[1]-a[1]*b[0], np.dot(a,b)))
    velocities.append(float(angle))
cv_truth = world[cut]+(times-times[cut])[:,None]*before
forward = before[:2]/np.linalg.norm(before[:2])
normal = np.array([-forward[1], forward[0]])
lateral = (world[:, :2]-cv_truth[:, :2]) @ normal
score['native_turn'] = dict(maximum_heading_change_deg=float(max(map(abs, velocities))),
    maximum_lateral_departure_m=float(np.max(abs(lateral[cut+1:]))),
    positive_turn_angle_max_deg=max(velocities), negative_turn_angle_min_deg=min(velocities))
score['turn_valid'] = score['native_turn']['maximum_heading_change_deg'] >= 10. and score['native_turn']['maximum_lateral_departure_m'] >= .25
score['passed_turn_comparison'] = score['turn_valid'] and score['methods']['multi']['better_than_constant_velocity']
(ESTIMATES/'multiview-score.json').write_text(json.dumps(score, indent=2)+'\n')
print(json.dumps({**score,'methods':{key:{k:v for k,v in value.items() if k!='rows'} for key,value in score['methods'].items()}},indent=2))
