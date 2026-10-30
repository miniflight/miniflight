"""Geometric estimates from camera observations."""

import numpy as np


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
