"""Fit two-view and twelve-view background maps, then estimate the turn."""
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from common.math import Quaternion
from miniflight.attitude import gyro_step
from miniflight.vision import landmark_covariance, map_pose, triangulate_views
from target.aigp.experiments.camera_imu import vision
from target.aigp.experiments.patch_track import seeds, track

ROOT = Path(sys.argv[1]).resolve()
CUT = 59
RECTIFY = '--raw' not in sys.argv
UNCERTAINTY = '--map-uncertainty' in sys.argv
K, K_INV = vision.K, vision.K_INV


def rotations_at(frames, imu):
    q, previous, matrices = Quaternion(), np.array(imu[0]['gyro_frd_rad_s']), []
    for sample in imu:
        current = np.array(sample['gyro_frd_rad_s'])
        q = gyro_step(q, (current+previous)*.5, sample['dt_s'])
        matrices.append(q.as_rotation_matrix())
        previous = current
    interpolate = Slerp([s['time_s'] for s in imu], Rotation.from_matrix(matrices))
    body = interpolate([f['capture_time_s'] for f in frames]).as_matrix()
    c = vision.CAMERA_FROM_BODY
    return np.array([c @ r.T @ body[0] @ c.T for r in body])


def project(points):
    h = points @ K.T
    return h[:, :2]/h[:, 2:]


def main():
    started = time.perf_counter()
    frames = json.loads((ROOT/'camera-input.json').read_text())['frames']
    imu = json.loads((ROOT/'gyro-input.json').read_text())['samples']
    rotations = rotations_at(frames, imu)
    capture = cv2.VideoCapture(str(ROOT/'source-lossless.mp4'))
    rgb = []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        rgb.append(image)
    capture.release()
    assert len(rgb) == len(frames)
    gate = []
    for index in range(CUT+1):
        corners = vision.nearest_aperture(rgb[index], close_gaps=True, largest_rim=True)
        pose = vision.best_pose(corners)
        if pose is not None and pose[0] <= 1.:
            gate.append(dict(frame=index, translation=pose[2].tolist(), reprojection_rms_px=float(pose[0])))
    assert len(gate) >= 12
    first = gate[0]['frame']
    assert gate[-1]['frame'] >= CUT-2
    rotations = np.array([r @ rotations[first].T for r in rotations])
    times = np.array([f['capture_time_s']-frames[first]['capture_time_s'] for f in frames])
    measured = np.array([rotations[g['frame']].T @ g['translation'] for g in gate])
    design = np.column_stack((np.ones(len(gate)), [times[g['frame']] for g in gate]))
    initial = np.linalg.lstsq(design, measured, rcond=None)[0]
    fit = least_squares(lambda value:(design @ value.reshape(2, 3)-measured).ravel(),
                        initial.ravel(), loss='huber', f_scale=.05)
    coefficients = fit.x.reshape(2, 3)
    velocity = -coefficients[1]
    positions = times[:, None]*velocity
    images, gate_masks, valid_masks = [], [], []
    for index, image in enumerate(rgb):
        blue, green, red = cv2.split(image.astype(float))
        mask = np.uint8(red-(blue+green)/2 > 20)
        valid = np.ones((360, 640), np.uint8)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if RECTIFY:
            homography = K @ rotations[index].T @ K_INV
            gray = cv2.warpPerspective(gray, homography, (640, 360))
            mask = cv2.warpPerspective(mask, homography, (640, 360), flags=cv2.INTER_NEAREST)
            valid = cv2.warpPerspective(valid, homography, (640, 360), flags=cv2.INTER_NEAREST)
        images.append(gray)
        gate_masks.append(cv2.dilate(mask, np.ones((19, 19), np.uint8)))
        valid_masks.append(cv2.erode(valid, np.ones((19, 19), np.uint8)))
    if RECTIFY:
        rotations = np.repeat(np.eye(3)[None], len(frames), axis=0)
    selected = [p for p in seeds(images[first]) if not gate_masks[first][round(p[1]), round(p[0])]]
    tracks = [dict(seed=p.tolist(), rows=[dict(status='before_reference')]*first+
                   track(images[first:], p, anchor_first=True)) for p in selected]
    overlap_rejections = 0
    for item in tracks:
        for index in range(first, len(item['rows'])):
            row = item['rows'][index]
            if row['status'] == 'tracked':
                x, y = np.round(row['point']).astype(int)
                if gate_masks[index][y, x] or not valid_masks[index][y, x]:
                    row['status'] = 'lost_gate_overlap' if gate_masks[index][y, x] else 'lost_warp_border'
                    item['rows'] = item['rows'][:index+1]
                    overlap_rejections += 1
                    break
    keyframes = np.unique(np.round(np.linspace(first, CUT, 12)).astype(int))
    mapped = []
    for item in tracks:
        if len(item['rows']) <= CUT or item['rows'][CUT]['status'] != 'tracked':
            continue
        pixels = np.array([item['rows'][i]['point'] for i in keyframes])
        ray = np.r_[pixels[0], 1.] @ K_INV.T @ rotations[CUT].T
        warped = K @ ray
        parallax = np.linalg.norm(pixels[-1]-warped[:2]/warped[2])
        if parallax < 1.5:
            continue
        endpoints = [0, len(keyframes)-1]
        two = triangulate_views(pixels[endpoints], rotations[keyframes[endpoints]], positions[keyframes[endpoints]], K)
        many = triangulate_views(pixels, rotations[keyframes], positions[keyframes], K)
        if two is None or many is None:
            continue
        projected = project(np.einsum('nij,nj->ni', rotations[keyframes], many-positions[keyframes]))
        rms = np.sqrt(np.mean((projected-pixels)**2))
        if rms <= 1. and min(two[2], many[2]) > .5:
            covariance = None
            if UNCERTAINTY:
                covariance = landmark_covariance(many, rotations[keyframes], positions[keyframes], K, max(.3, float(rms)))
                assert covariance is not None, 'unobservable landmark covariance'
            mapped.append(dict(track=item, two=two, multi=many, covariance=covariance,
                               map_rms_px=float(rms), parallax_px=float(parallax)))
    print('MAP', first, CUT, 'gate observations', len(gate), 'tracks', len(tracks), 'shared points', len(mapped), flush=True)
    assert len(mapped) >= 6
    result = dict(protocol=dict(calibration_end_frame=CUT, map_views=keyframes.tolist(), tracker='fixed first reference, 15x15',
        gyro_rectified=RECTIFY,
        landmark_uncertainty=UNCERTAINTY,
        visual_rotation_prior_sigma_deg=1., pixel_sigma=.3,
        gate_size_m=1.5, gate_exclusion='Red excess>20, expanded 9px; permanently reject intersecting tracks',
        map_parallax_min_px=1.5, map_rms_max_px=1.,
        position_huber_px=.8, pose_minimum_points=6),
        reference_frame=first, shared_initial_position=positions[CUT].tolist(),
        shared_initial_velocity=velocity.tolist(), gate_observations=gate,
        gate_fit_rms_m=float(np.sqrt(np.mean((design @ coefficients-measured)**2))),
        gate_overlap_rejections=overlap_rejections,
        map_points=[dict(seed=m['track']['seed'], two=m['two'].tolist(), multi=m['multi'].tolist(),
                        covariance=None if m['covariance'] is None else m['covariance'].tolist(),
                        reprojection_rms_px=m['map_rms_px'], parallax_px=m['parallax_px']) for m in mapped],
        methods={}, simulator_state_used=False)
    for method in (('two', 'uniform', 'multi') if UNCERTAINTY else ('two', 'multi')):
        current = positions[CUT].copy()
        output = []
        for index in range(CUT+1, len(frames)):
            visible = [m for m in mapped if len(m['track']['rows']) > index and m['track']['rows'][index]['status'] == 'tracked']
            if len(visible) < 6:
                output.append(dict(frame=index, status='lost_points', points=len(visible)))
                break
            world = np.array([m['multi' if method == 'uniform' else method] for m in visible])
            pixels = np.array([m['track']['rows'][index]['point'] for m in visible])
            covariance = np.array([m['covariance'] for m in visible]) if UNCERTAINTY and method == 'multi' else None
            pose = map_pose(world, pixels, K, rotations[index], current, landmark_covariances=covariance)
            if pose is None:
                output.append(dict(frame=index, status='lost_fit', points=len(visible)))
                break
            current, corrected, rms, correction = pose
            output.append(dict(frame=index, status='estimated',
                position=current.tolist(), constant_velocity_position=positions[index].tolist(), points=len(visible),
                reprojection_rms_px=rms, corrected_rotation_feature=corrected.tolist(),
                visual_rotation_correction_rad=correction.tolist()))
        result['methods'][method] = output
    result['processing_s'] = time.perf_counter()-started
    result['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (ROOT/'multiview-estimates.json').write_text(json.dumps(result, indent=2)+'\n')
    print('ESTIMATED', {method:len(rows) for method, rows in result['methods'].items()}, flush=True)


if __name__ == '__main__':
    main()
