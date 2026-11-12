"""One image patch over time, with texture and forward/backward checks."""

import cv2
import numpy as np


def seeds(image, columns=4, rows=3, per_cell=8):
    """Choose image corners in separate cells, rather than one textured object."""
    height, width = image.shape
    points = []
    for y in range(rows):
        for x in range(columns):
            mask = np.zeros_like(image, dtype=np.uint8)
            mask[y*height//rows+10:(y+1)*height//rows-10,
                 x*width//columns+10:(x+1)*width//columns-10] = 255
            found = cv2.goodFeaturesToTrack(image, per_cell, .01, 18, mask=mask, blockSize=7)
            if found is not None:
                points.extend(found.reshape(-1, 2))
    return np.asarray(points, dtype=float).reshape(-1, 2)


def sample(image, centers, side=8):
    """Bilinear samples; centers use pixel-center coordinates."""
    centers = np.asarray(centers, dtype=float).reshape(-1, 2)
    offset = np.arange(side)-(side-1)/2
    x = centers[:, 0, None, None]+offset[None, None, :]+np.zeros((1, side, 1))
    y = centers[:, 1, None, None]+offset[None, :, None]+np.zeros((1, 1, side))
    ix, iy = np.floor(x).astype(int), np.floor(y).astype(int)
    if ix.min() < 0 or iy.min() < 0 or ix.max()+1 >= image.shape[1] or iy.max()+1 >= image.shape[0]:
        raise ValueError("patch search is outside the image")
    u, v = x-ix, y-iy
    return ((1-u)*(1-v)*image[iy, ix]+u*(1-v)*image[iy, ix+1]
            +(1-u)*v*image[iy+1, ix]+u*v*image[iy+1, ix+1])


def texture(patch):
    gx = (patch[1:-1, 2:]-patch[1:-1, :-2])/2
    gy = (patch[2:, 1:-1]-patch[:-2, 1:-1])/2
    return np.linalg.eigvalsh([[np.mean(gx*gx), np.mean(gx*gy)],
                              [np.mean(gx*gy), np.mean(gy*gy)]])


def track(images, seed, side=15):
    """Track consecutive grayscale frames; stop at the first rejected estimate.

    A cycle is a consistency check, not independent evidence of correspondence.
    No camera pose, gate geometry, timestamps or simulator state enters here.
    """
    point = np.asarray(seed, dtype=np.float32).reshape(1, 1, 2)
    parameters = dict(winSize=(side, side), maxLevel=0,
                      criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .001))
    rows = []
    previous = None
    for index, image in enumerate(images):
        image = np.asarray(image, dtype=np.uint8)
        row = dict(frame=index, status="tracked")
        try:
            if previous is not None:
                next_point, good, error = cv2.calcOpticalFlowPyrLK(previous, image, point, None, **parameters)
                if not good[0, 0] or not np.isfinite(next_point).all():
                    rows.append(dict(row, status="lost_flow"))
                    break
                back, reverse_good, _ = cv2.calcOpticalFlowPyrLK(image, previous, next_point, None, **parameters)
                if not reverse_good[0, 0] or not np.isfinite(back).all():
                    rows.append(dict(row, status="lost_cycle"))
                    break
                row.update(point=next_point.ravel().tolist(), photometric_mae=float(error[0, 0]),
                           cycle_px=float(np.linalg.norm(back-point)))
                if row["cycle_px"] > 1.:
                    row["status"] = "lost_cycle"
                elif row["photometric_mae"] > 20.:
                    row["status"] = "lost_appearance"
                point = next_point
            else:
                row["point"] = point.ravel().tolist()
            if row["status"] == "tracked":
                low, high = texture(sample(image.astype(float), point, side)[0])
                row.update(minimum_eigenvalue=float(low), eigenvalue_ratio=float(low/high) if high else 0.)
                if high < 2:
                    row["status"] = "lost_low_contrast"
                elif low < 2 or low/high < .04:
                    row["status"] = "lost_one_dimensional"
        except ValueError:
            row["status"] = "lost_outside_image"
        rows.append(row)
        if row["status"] != "tracked":
            break
        previous = image
    return rows
