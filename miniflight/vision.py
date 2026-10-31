"""Geometric estimates from camera observations."""

import numpy as np


def relative_rotation(previous, current, intrinsics, threshold_px=1.0):
    """Estimate rotation from previous to current camera, allowing translation.

    Inputs are matching N-by-2 undistorted pixel coordinates and a 3-by-3
    pinhole intrinsic matrix. Return (rotation, positive-depth inlier mask),
    or None when no single essential matrix is found. No absolute heading.
    """
    import cv2

    if len(previous) < 5:
        return None
    essential, mask = cv2.findEssentialMat(previous, current, intrinsics, method=cv2.RANSAC,
                                          prob=.999, threshold=threshold_px)
    if essential is None or essential.shape != (3, 3):
        return None
    _, rotation, _, mask = cv2.recoverPose(essential, previous, current, intrinsics, mask=mask)
    return rotation, mask.ravel() != 0


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
