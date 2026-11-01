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
