"""Geometric estimates from camera observations."""

import numpy as np


def relative_pose(previous, current, intrinsics, threshold_px=1.0):
    """Estimate X_current = R @ X_previous + t, up to translation scale.

    Inputs are matching N-by-2 undistorted pixel coordinates and a 3-by-3
    pinhole intrinsic matrix. Return (R, unit t, positive-depth inlier mask),
    or None when no single essential matrix is found. Camera displacement
    points along -R.T @ t in the previous frame; its magnitude is unknown.
    Translation is poorly constrained when parallax is small.
    Distant points remain useful for rotation; no depth cutoff is imposed.
    """
    import cv2

    if len(previous) < 5:
        return None
    essential, mask = cv2.findEssentialMat(previous, current, intrinsics, method=cv2.RANSAC,
                                          prob=.999, threshold=threshold_px)
    if essential is None or essential.shape != (3, 3):
        return None
    _, rotation, translation, mask, _ = cv2.recoverPose(
        essential, previous, current, intrinsics, distanceThresh=np.inf, mask=mask)
    return rotation, translation.ravel(), mask.ravel() != 0


def relative_rotation(previous, current, intrinsics, threshold_px=1.0):
    """Return rotation and positive-depth inliers, allowing translation."""
    pose = relative_pose(previous, current, intrinsics, threshold_px)
    return None if pose is None else (pose[0], pose[2])


def triangulate_views(pixels, rotations, positions, intrinsics):
    """One world point from multiple views, or None for unobservable depth.

    Rotations map world vectors into camera axes; positions are camera centers
    in the same world frame. Camera translations set the reconstruction's scale.
    """
    if len(pixels) < 2:
        return None
    projection = np.array([intrinsics @ np.column_stack((r, -r @ p))
                           for r, p in zip(rotations, positions, strict=True)])
    pixels = np.asarray(pixels)
    design = (pixels[:, :, None]*projection[:, None, 2, :]-projection[:, :2, :]).reshape(-1, 4)
    _, singular, vt = np.linalg.svd(design, full_matrices=False)
    if singular[-2] < 1e-10 or abs(vt[-1, 3]) < 1e-10:
        return None
    point = vt[-1, :3]/vt[-1, 3]
    camera = np.array([r @ (point-p) for r, p in zip(rotations, positions, strict=True)])
    return point if np.all(camera[:, 2] > 0) else None


def projection_jacobians(camera, intrinsics):
    """Derivative of image coordinates with respect to camera XYZ, per point."""
    h = np.asarray(camera) @ intrinsics.T
    return (intrinsics[None, :2, :]*h[:, 2, None, None]
            -h[:, :2, None]*intrinsics[None, 2:3, :])/h[:, 2, None, None]**2


def landmark_covariance(point, rotations, positions, intrinsics, pixel_sigma=.3):
    """Conditional XYZ covariance for fixed camera poses and independent pixel noise."""
    rotations, positions = np.asarray(rotations), np.asarray(positions)
    camera = np.einsum('nij,nj->ni', rotations, np.asarray(point)-positions)
    if len(camera) < 2 or np.any(camera[:, 2] <= 0):
        return None
    jacobian = (projection_jacobians(camera, intrinsics) @ rotations).reshape(-1, 3)/pixel_sigma
    _, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
    if singular[-1] < 1e-10:
        return None
    return (vt.T/singular**2) @ vt


def map_pose(world, pixels, intrinsics, rotation, position, pixel_sigma=.3, rotation_sigma=np.pi/180,
             landmark_covariances=None):
    """Position and rotation from mapped points with a soft gyro rotation prior.

    Rotation maps world into camera axes. Return position, corrected rotation,
    pixel RMS and the fitted rotation correction; return None on failure.
    The prior sigmas are in pixels and radians, respectively.
    Optional landmark covariances are projected at the supplied initial pose;
    these observation weights remain fixed during this fit.
    """
    import cv2
    from scipy.optimize import least_squares

    if len(world) < 6:
        return None
    world, pixels = np.asarray(world), np.asarray(pixels)
    whitening = None
    if landmark_covariances is not None:
        camera = (world-position) @ rotation.T
        jacobian = projection_jacobians(camera, intrinsics) @ rotation
        variance = pixel_sigma**2*np.eye(2)+jacobian @ landmark_covariances @ jacobian.transpose(0, 2, 1)
        whitening = np.linalg.inv(np.linalg.cholesky(variance))
    def residual(value):
        correction, _ = cv2.Rodrigues(value[3:])
        camera = (world-value[:3]) @ (correction @ rotation).T
        projected = camera @ intrinsics.T
        error = projected[:, :2]/projected[:, 2:]-pixels
        weighted = error/pixel_sigma if whitening is None else np.einsum('nij,nj->ni', whitening, error)
        return np.r_[weighted.ravel(), value[3:]/rotation_sigma]
    fit = least_squares(residual, np.r_[position, np.zeros(3)], loss='huber',
                        f_scale=.8/pixel_sigma, x_scale=[1., 1., 1., .01, .01, .01])
    corrected = cv2.Rodrigues(fit.x[3:])[0] @ rotation
    camera = (world-fit.x[:3]) @ corrected.T
    if not fit.success or not np.isfinite(fit.x).all() or np.any(camera[:, 2] <= 0):
        return None
    projected = camera @ intrinsics.T
    rms = float(np.sqrt(np.mean((projected[:, :2]/projected[:, 2:]-pixels)**2)))
    return fit.x[:3], corrected, rms, fit.x[3:]


def translation_direction(previous, current, intrinsics, rotation):
    """Unit camera displacement in previous-camera axes, with supplied rotation.

    Return (direction, positive-depth mask, singular values), or None for
    insufficient constraints. Rotation maps previous-camera to current-camera
    axes. Correspondences must describe a static scene; reject outliers before
    this least-squares fit. Magnitude is unknown, and weak parallax is unreliable.
    """
    if len(previous) < 3:
        return None
    inverse = np.linalg.inv(intrinsics)
    a, b = (np.column_stack((points, np.ones(len(points)))) @ inverse.T
            for points in (previous, current))
    rotated = a @ rotation.T
    _, singular, vt = np.linalg.svd(np.cross(rotated, b), full_matrices=False)
    if singular[1] < 1e-10:
        return None
    direction = vt[-1]
    depths = np.array([np.linalg.lstsq(np.column_stack((u, -v)), direction, rcond=None)[0]
                       for u, v in zip(rotated, b, strict=True)])
    if np.sum(np.all(depths < 0, axis=1)) > np.sum(np.all(depths > 0, axis=1)):
        direction, depths = -direction, -depths
    return rotation.T @ direction, np.all(depths > 0, axis=1), singular


def parallax_angles(previous, current, rotation):
    """Return per-point parallax in radians for matching nonzero N-by-3 rays.

    Rotation maps previous-camera directions into the current camera frame.
    """
    previous, current = (np.asarray(rays, dtype=float) for rays in (previous, current))
    rotated = previous @ rotation.T
    cross = np.linalg.norm(np.cross(rotated, current), axis=1)
    dot = np.sum(rotated * current, axis=1)
    return np.arctan2(cross, dot)


def rotation_between_bearings(previous, current):
    """Fit the rotation from previous-camera rays into current-camera rays.

    Inputs are matching nonzero N-by-3 bearings with at least two nonparallel
    directions. Assumes a static scene and pure camera rotation; translation
    creates parallax and biases this fit. No distance or absolute heading.
    """
    previous, current = (np.asarray(rays, dtype=float) for rays in (previous, current))
    previous = previous / np.linalg.norm(previous, axis=1, keepdims=True)
    current = current / np.linalg.norm(current, axis=1, keepdims=True)
    u, _, vt = np.linalg.svd(current.T @ previous)
    return u @ np.diag((1, 1, np.linalg.det(u @ vt))) @ vt
