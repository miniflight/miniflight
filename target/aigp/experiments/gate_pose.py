"""Camera-relative square pose from image corners; no simulator state is an input."""

import cv2
import numpy as np


def estimate(corners, side_m, camera_matrix):
    """Corners are image TL/TR/BR/BL. Camera axes are right, down, forward.

    The square's axes are right, up, out of its face. Keep both IPPE solutions;
    returned errors are pixel RMS per coordinate, not a confidence estimate.
    """
    square = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], dtype=float) * side_m / 2
    _, rotations, translations, errors = cv2.solvePnPGeneric(
        square, np.array(corners, dtype=float), np.array(camera_matrix, dtype=float), None,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return tuple(dict(rotation_vector_rad=rotation.ravel().tolist(), translation_camera_m=translation.ravel().tolist(),
                      reprojection_coordinate_rmse_px=float(error[0]))
                 for rotation, translation, error in zip(rotations, translations, errors, strict=True))
