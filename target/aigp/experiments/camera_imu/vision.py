"""Aperture measurements and IMU-warped corner tracking from pixels."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from target.aigp.experiments.gate_pose import estimate, fit_outline

K = np.array([[320., 0, 320.], [0, 320., 180.], [0, 0, 1.]])
K_INV = np.linalg.inv(K)
SQUARE = .75 * np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]])
tilt = np.radians(20)
CAMERA_FROM_BODY = np.array([[0, 1, 0], [np.sin(tilt), 0, np.cos(tilt)],
                             [np.cos(tilt), 0, -np.sin(tilt)]])

def refine(image, corners):
    corners = np.asarray(corners, dtype=float)
    if np.any(corners < 1) or np.any(corners[:, 0] >= image.shape[1]-1) or np.any(corners[:, 1] >= image.shape[0]-1):
        return None
    blue, green, red = cv2.split(image.astype(np.float32)/255)
    signal = red-(green+blue)/2
    gx = cv2.Sobel(signal, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(signal, cv2.CV_32F, 0, 1, ksize=3)
    edges = np.roll(corners, -1, axis=0)-corners
    lengths = np.linalg.norm(edges, axis=1)
    margin = max(2, round(lengths.min()*.16))
    offsets = np.arange(-margin, margin+1)
    lines = []
    supported = 0
    for corner, edge, length in zip(corners, edges, lengths):
        normal = np.array([-edge[1], edge[0]])/length
        centers = corner+np.linspace(.15, .85, 40)[:, None]*edge
        samples = centers[:, None]+offsets[None, :, None]*normal
        x, y = (samples[:, :, i].astype(np.float32) for i in (0, 1))
        response = -cv2.remap(gx, x, y, cv2.INTER_LINEAR)*normal[0]-cv2.remap(gy, x, y, cv2.INTER_LINEAR)*normal[1]
        points = samples[np.arange(len(centers)), response.argmax(axis=1)]
        outside = (points-2*normal).astype(np.float32)
        rim = cv2.remap(signal, outside[:, 0, None], outside[:, 1, None], cv2.INTER_LINEAR)
        supported += np.median(rim) > .08
        vx, vy, x0, y0 = cv2.fitLine(points.astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
        lines.append([-vy, vx, vy*x0-vx*y0])
    if supported < 2:
        return None
    intersections = np.array([np.cross(lines[i-1], lines[i]) for i in range(4)])
    if np.any(abs(intersections[:, 2]) < 1e-9):
        return None
    result = intersections[:, :2]/intersections[:, 2:]
    return result if np.isfinite(result).all() and cv2.isContourConvex(result.astype(np.float32)) else None


def best_pose(corners):
    if corners is None:
        return None
    usable = []
    for pose in estimate(corners, 1.50, K):
        translation = np.asarray(pose["translation_camera_m"])
        rotation = np.asarray(pose["rotation_vector_rad"])
        if not np.isfinite(np.r_[translation, rotation, pose["reprojection_coordinate_rmse_px"]]).all():
            continue
        xyz = Rotation.from_rotvec(rotation).apply(SQUARE) + translation
        if np.min(xyz[:, 2]) > .05:
            usable.append((pose["reprojection_coordinate_rmse_px"], xyz, translation))
    return min(usable, key=lambda p: p[0], default=None)


def warped(previous, position, orientation, method):
    old_position, old_orientation, corners, pose = previous
    if method != "rotation" and pose is not None:
        world = pose[1] @ old_orientation.T + old_position
        camera = (world - position) @ orientation
    else:
        rays = np.column_stack((corners, np.ones(4))) @ K_INV.T
        camera = rays @ old_orientation.T @ orientation
    if np.min(camera[:, 2]) <= .05:
        return None
    pixels = camera @ K.T
    return pixels[:, :2] / pixels[:, 2:]


def aperture_candidates(image, close_gaps=False, largest_rim=False):
    # An enclosed hole uses all four sides of the rim. Prefer the largest hole
    # that survives the same edge fitter rather than the strongest red peak.
    blue, green, red = cv2.split(image.astype(np.float32) / 255)
    mask = np.uint8(red-(blue+green)/2 > .08) * 255
    if close_gaps:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return None
    parents = np.flatnonzero(hierarchy[0, :, 3] < 0)
    nearest = max(parents, key=lambda i: cv2.contourArea(cv2.convexHull(contours[i]))) if len(parents) else None
    apertures = []
    for index, node in enumerate(hierarchy[0]):
        parent = node[3]
        if parent < 0:
            continue
        if largest_rim and parent != nearest:
            continue
        area = cv2.contourArea(contours[index])
        outer_area = cv2.contourArea(contours[parent])
        if area >= 25 and .16*outer_area <= area <= .8*outer_area:
            apertures.append((area, index))
    for _, index in sorted(apertures, reverse=True):
        fit = fit_outline(contours[index], image.shape)
        if fit is None:
            continue
        corners = refine(image, fit[0])
        if best_pose(corners) is not None:
            yield corners


def nearest_aperture(image, close_gaps=False, largest_rim=False):
    return next(aperture_candidates(image, close_gaps, largest_rim), None)


def iou(a, b):
    a, b = a.astype(np.float32), b.astype(np.float32)
    if not cv2.isContourConvex(a) or not cv2.isContourConvex(b):
        return 0.
    overlap, _ = cv2.intersectConvexConvex(a, b)
    union = cv2.contourArea(a)+cv2.contourArea(b)-overlap
    return overlap/union if union > 0 else 0.
