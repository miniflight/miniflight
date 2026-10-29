"""Camera-relative square pose from image corners; no simulator state is an input."""

import cv2
import numpy as np


def frame_and_opening(contours, hierarchy):
    """Largest outer component and its largest child hole; no other-frame fallback."""
    if hierarchy is None:
        return None, None
    parents = np.flatnonzero(hierarchy[0, :, 3] < 0)
    if not len(parents):
        return None, None
    parent = max(parents, key=lambda i: cv2.contourArea(contours[i]))
    holes = np.flatnonzero(hierarchy[0, :, 3] == parent)
    opening = None if not len(holes) else contours[max(holes, key=lambda i: cv2.contourArea(contours[i]))]
    return contours[parent], opening


def fit_edges(contour, seed):
    """Fit a supplied contour; the ordered seed only associates its edges.

    Return corner intersections and four pixel RMS residuals, or None.
    This does not identify gates or validate which physical edge was measured.
    """
    points = np.asarray(contour, dtype=float).reshape(-1, 2)
    seed = np.asarray(seed, dtype=float)
    edges = np.roll(seed, -1, axis=0) - seed
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths == 0):
        return None
    relative = points[:, None, :] - seed[None, :, :]
    fraction = np.sum(relative * edges, axis=2) / lengths**2
    distance = np.abs(relative[:, :, 0]*edges[:, 1] - relative[:, :, 1]*edges[:, 0]) / lengths
    nearest = np.argmin(distance, axis=1)
    lines, residuals = [], []
    for i in range(4):
        samples = points[(nearest == i) & (fraction[:, i] > .15) & (fraction[:, i] < .85)]
        if len(samples) < 3:
            return None
        vx, vy, x, y = cv2.fitLine(samples.astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
        line = np.array([-vy, vx, vy*x-vx*y], dtype=float)
        lines.append(line)
        residuals.append(float(np.sqrt(np.mean((samples @ line[:2] + line[2])**2))))
    corners = np.array([np.cross(lines[i-1], lines[i]) for i in range(4)])
    if np.any(np.abs(corners[:, 2]) < 1e-9):
        return None
    corners = corners[:, :2] / corners[:, 2:]
    if not np.isfinite(corners).all() or not cv2.isContourConvex(corners.astype(np.float32)):
        return None
    return corners, residuals


def fit_outline(contour, image_shape):
    """Fit a fully visible convex envelope using its original contour pixels."""
    if contour is None:
        return None
    x, y, width, height = cv2.boundingRect(contour)
    if x == 0 or y == 0 or x+width == image_shape[1] or y+height == image_shape[0]:
        return None
    hull = cv2.convexHull(contour)
    points = contour.reshape(-1, 2)
    boundary = points[[abs(cv2.pointPolygonTest(hull, tuple(map(float, p)), True)) <= 1 for p in points]]
    seed = cv2.boxPoints(cv2.minAreaRect(hull))
    if cv2.contourArea(seed, oriented=True) < 0:
        seed = seed[::-1]
    seed = np.roll(seed, -np.argmin(seed.sum(axis=1)), axis=0)
    return fit_edges(boundary, seed)


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
