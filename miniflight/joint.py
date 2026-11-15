"""A small joint visual/inertial window; inputs contain no simulator state."""
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from miniflight.vision import projection_jacobians


def solve_window(request, check_jacobian=False):
    """Jointly fit positions, velocities, small gyro corrections and log depths.

    State rows are [p(3), v(3), rotation correction(3)]. Each frame contains
    landmark ids and rectified pixels; rays are fixed first-view bearings.
    IMU edges contain measured duration and specific-force dv/dp integrals.
    The anchor and depth priors come from camera/IMU initialization.
    Version 2 keeps supplied world-force integrals in world axes and robustifies
    image residuals only. Version 1 remains for replaying earlier experiments.
    Optional gate_pixels observe a static gate_world learned during camera
    initialization. fixed_anchor_position leaves the initial velocity adjustable.
    """
    started = time.perf_counter()
    k = np.array(request['intrinsics'])
    rays = np.array(request['rays'])
    depth_prior = np.array(request['depth_prior'])
    depth_sigma = np.array(request['depth_sigma'])
    states = np.array(request['states'])
    frames = request['frames']
    edges = request['edges']
    gravity = np.array(request['gravity'])
    anchor = np.array(request['anchor'])
    count, landmarks = len(states), len(rays)
    world_force = request.get('model_version', 1) >= 2
    anchor_velocity_sigma = request.get('anchor_velocity_sigma', .1)
    columns = count*9+landmarks
    gate_world = np.array(request.get('gate_world', [])).reshape(-1, 3)
    frame_data = []
    for frame in frames:
        ids = np.array(frame['ids'], dtype=int)
        pixels = np.array(frame['pixels']).reshape(-1, 2)
        gate_pixels = np.array(frame.get('gate_pixels', [])).reshape(-1, 2)
        if len(gate_pixels) and (len(gate_pixels) != 4 or len(gate_world) != 4):
            raise ValueError('gate observations require four mapped corners')
        sigma = np.r_[np.full(len(ids), .3), np.full(len(gate_pixels), request.get('gate_pixel_sigma', .8))]
        frame_data.append((ids, np.vstack((pixels, gate_pixels)), sigma))
    pixel_rows = sum(len(pixels)*2 for _, pixels, _ in frame_data)
    total_rows = pixel_rows+count*3+6+landmarks+len(edges)*6
    if count < 2 or len(frames) != count or len(frame_data[-1][0]) < 6:
        raise ValueError('joint window needs at least two states and six current landmarks')
    edge_data = []
    for edge in edges:
        i, j, dt = edge['first'], edge['last'], edge['dt']
        density = (.01 if world_force else .3)**2
        velocity_floor = .003 if world_force else .03
        position_floor = .003 if world_force else .005
        covariance = np.block([[np.eye(3)*(density*dt**3/3+position_floor**2), np.eye(3)*density*dt**2/2],
                               [np.eye(3)*density*dt**2/2, np.eye(3)*(density*dt+velocity_floor**2)]])
        edge_data.append((i, j, dt, np.array(edge['dv']), np.array(edge['dp']),
                          np.linalg.inv(np.linalg.cholesky(covariance))))

    def residual(value):
        state = value[:count*9].reshape(count, 9)
        depths = np.exp(value[count*9:])
        world = rays*depths[:, None]
        rotations = [cv2.Rodrigues(s[6:9])[0] for s in state]
        errors = []
        for i, (ids, pixels, sigma) in enumerate(frame_data):
            if not len(pixels):
                continue
            observed = np.vstack((world[ids], gate_world)) if len(pixels) > len(ids) else world[ids]
            camera = (observed-state[i, :3]) @ rotations[i].T
            h = camera @ k.T
            errors.extend(((h[:, :2]/h[:, 2:]-pixels)/sigma[:, None]).ravel())
        errors.extend((state[:, 6:9]/np.radians(1.)).ravel())
        errors.extend((state[0, :3]-anchor[:3])/.05)
        errors.extend((state[0, 3:6]-anchor[3:6])/anchor_velocity_sigma)
        errors.extend((value[count*9:]-depth_prior)/depth_sigma)
        for i, j, dt, dv, dp, whitening in edge_data:
            correction = np.eye(3) if world_force else cv2.Rodrigues((state[i, 6:9]+state[j, 6:9])*.5)[0].T
            position = state[j, :3]-state[i, :3]-state[i, 3:6]*dt-correction @ dp-.5*gravity*dt**2
            velocity = state[j, 3:6]-state[i, 3:6]-correction @ dv-gravity*dt
            errors.extend(whitening @ np.r_[position, velocity])
        return np.asarray(errors)

    def jacobian(value):
        state = value[:count*9].reshape(count, 9)
        world = rays*np.exp(value[count*9:])[:, None]
        result = np.zeros((total_rows, columns))
        row = 0
        for i, (ids, pixels, sigma) in enumerate(frame_data):
            if not len(pixels):
                continue
            rotation, derivative = cv2.Rodrigues(state[i, 6:9])
            observed = np.vstack((world[ids], gate_world)) if len(pixels) > len(ids) else world[ids]
            relative = observed-state[i, :3]
            camera = relative @ rotation.T
            projection = projection_jacobians(camera, k)/sigma[:, None, None]
            position_jacobian = -projection @ rotation
            dr = derivative.reshape(3, 3, 3)
            camera_rotation = np.array([relative @ d.T for d in dr])
            rotation_jacobian = np.einsum('nij,knj->nik', projection, camera_rotation)
            depth_jacobian = np.einsum('nij,nj->ni', projection[:len(ids)] @ rotation, world[ids])
            n = len(pixels)*2
            result[row:row+n, i*9:i*9+3] = position_jacobian.reshape(-1, 3)
            result[row:row+n, i*9+6:i*9+9] = rotation_jacobian.reshape(-1, 3)
            for point_index, point in enumerate(ids):
                result[row+point_index*2:row+point_index*2+2, count*9+point] = depth_jacobian[point_index]
            row += n
        for i in range(count):
            result[row:row+3, i*9+6:i*9+9] = np.eye(3)/np.radians(1.)
            row += 3
        result[row:row+3, :3] = np.eye(3)/.05
        result[row+3:row+6, 3:6] = np.eye(3)/anchor_velocity_sigma
        row += 6
        result[row:row+landmarks, count*9:] = np.diag(1/depth_sigma)
        row += landmarks
        for i, j, dt, dv, dp, whitening in edge_data:
            left = np.zeros((6, 9));right = np.zeros((6, 9))
            left[:3, :3] = -np.eye(3)
            left[:3, 3:6] = -np.eye(3)*dt
            left[3:, 3:6] = -np.eye(3)
            right[:3, :3] = np.eye(3)
            right[3:, 3:6] = np.eye(3)
            if not world_force:
                derivative = cv2.Rodrigues((state[i, 6:9]+state[j, 6:9])*.5)[1].reshape(3, 3, 3)
                rotation = np.column_stack([np.r_[-.5*d.T @ dp, -.5*d.T @ dv] for d in derivative])
                left[:, 6:9] = right[:, 6:9] = rotation
            result[row:row+6, i*9:i*9+9] = whitening @ left
            result[row:row+6, j*9:j*9+9] = whitening @ right
            row += 6
        return result

    initial = np.r_[states.ravel(), request['depths']]
    def visual_huber(squared):
        rho = np.array([squared, np.ones_like(squared), np.zeros_like(squared)])
        ids = np.flatnonzero(squared[:pixel_rows] > 1.)
        root = np.sqrt(squared[ids])
        rho[0, ids] = 2*root-1
        rho[1, ids] = 1/root
        rho[2, ids] = -.5/root**3
        return rho
    lower = np.r_[np.full(count*9, -np.inf), np.full(landmarks, np.log(.5))]
    upper = np.r_[np.full(count*9, np.inf), np.full(landmarks, np.log(10000.))]
    fixed = 6 if request.get('fixed_anchor') else (3 if request.get('fixed_anchor_position') else 0)
    if fixed:
        initial[:fixed] = anchor[:fixed]
        lower[:fixed], upper[:fixed] = anchor[:fixed]-1e-10, anchor[:fixed]+1e-10
    error = None
    if check_jacobian:
        step = 1e-6
        numeric = np.column_stack([(residual(initial+np.eye(columns)[i]*step)-residual(initial-np.eye(columns)[i]*step))/(2*step) for i in range(columns)])
        error = float(np.max(abs(numeric-jacobian(initial))))
    fit = least_squares(residual, initial, jac=jacobian, bounds=(lower, upper),
                        loss=visual_huber if world_force else 'huber', f_scale=.8/.3, max_nfev=32,
                        ftol=1e-4, xtol=1e-5, gtol=1e-4,
                        x_scale=np.r_[np.tile([1., 1., 1., 1., 1., 1., .01, .01, .01], count), np.full(landmarks, .1)])
    state = fit.x[:count*9].reshape(count, 9)
    depths = fit.x[count*9:]
    last_ids, last_pixels, _ = frame_data[-1]
    last_pixels = last_pixels[:len(last_ids)]
    camera = (rays[last_ids]*np.exp(depths[last_ids, None])-state[-1, :3]) @ cv2.Rodrigues(state[-1, 6:9])[0].T
    h = camera @ k.T
    rms = float(np.sqrt(np.mean((h[:, :2]/h[:, 2:]-last_pixels)**2)))
    c = .8/.3
    initial_squared = residual(initial)**2
    if world_force:
        initial_cost = .5*c*c*np.sum(visual_huber(initial_squared/(c*c))[0])
    else:
        initial_cost = .5*np.sum(np.where(initial_squared <= c*c, initial_squared, 2*c*np.sqrt(initial_squared)-c*c))
    return dict(states=state.tolist(), depths=depths.tolist(), reprojection_rms_px=rms,
        finite=bool(np.isfinite(fit.x).all() and np.all(camera[:, 2] > 0)), converged=bool(fit.success),
        evaluations=fit.nfev, cost=float(fit.cost), initial_cost=float(initial_cost),
        solve_ms=(time.perf_counter()-started)*1000, jacobian_check_error=error)
