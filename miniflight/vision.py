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
