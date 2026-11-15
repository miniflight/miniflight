"""Metric motion initialization from one known square and camera/IMU data."""
import time

import cv2
import numpy as np
from scipy.optimize import least_squares

from .vision import projection_jacobians


def fit_gate_motion(request):
    """Fit one static gate pose and initial velocity to multiple image views.

    Frame rotations map the reference camera world into each camera. Offsets
    are IMU-predicted translations with zero initial velocity; no native pose
    or velocity is an input. Known square dimensions set the metric scale.
    """
    started = time.perf_counter()
    square, k = np.array(request['square']), np.array(request['intrinsics'])
    times = np.array(request['times'])
    rotations = np.array(request['rotations'])
    offsets = np.array(request['offsets'])
    pixels = np.array(request['pixels'])
    best = None
    for seed in request['seeds']:
        initial = np.r_[seed['center'], seed['rotation'], seed['velocity']]

        def geometry(value):
            gate = square @ cv2.Rodrigues(value[3:6])[0].T+value[:3]
            relative = gate[None]-times[:, None, None]*value[None, None, 6:9]-offsets[:, None]
            return np.einsum('nij,nkj->nki', rotations, relative)

        def residual(value):
            camera = geometry(value)
            h = camera @ k.T
            return (h[:, :, :2]/h[:, :, 2:]-pixels).ravel()

        def jacobian(value):
            camera = geometry(value)
            projection = projection_jacobians(camera.reshape(-1, 3), k).reshape(len(times), 4, 2, 3)
            world_jac = projection @ rotations[:, None]
            derivative = cv2.Rodrigues(value[3:6])[1].reshape(3, 3, 3)
            gate_derivative = np.array([square @ matrix.T for matrix in derivative])
            angle_jac = np.einsum('nfij,kfj->nfik', world_jac, gate_derivative)
            return np.concatenate((world_jac, angle_jac, -times[:, None, None, None]*world_jac), axis=3).reshape(-1, 9)

        try:
            fit = least_squares(residual, initial, jac=jacobian, loss='huber', f_scale=.8,
                max_nfev=60, ftol=1e-7, xtol=1e-7, gtol=1e-7,
                x_scale=[1., 1., 10., .1, .1, .1, 1., 1., 1.])
        except ValueError:
            continue
        camera = geometry(fit.x)
        if not fit.success or not np.isfinite(fit.x).all() or np.min(camera[:, :, 2]) <= .5:
            continue
        if np.linalg.norm(fit.x[6:9]) > 5.:
            continue
        result = dict(center=fit.x[:3].tolist(), rotation=fit.x[3:6].tolist(), velocity=fit.x[6:9].tolist(),
            reprojection_rms_px=float(np.sqrt(np.mean(residual(fit.x)**2))), cost=float(fit.cost), evaluations=fit.nfev)
        if best is None or result['cost'] < best['cost']:
            best = result
    if best is not None:
        best['fit_ms'] = (time.perf_counter()-started)*1000
    return best
