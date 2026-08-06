"""Receive AIGP camera frames and measure the nearest orange gate."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np


HEADER = struct.Struct("<IHHIIQ")
MIN_GATE_AREA_PX = 150.0
MIN_GATE_SPAN_PX = 8.0


@dataclass(frozen=True)
class GateCandidate:
    center_px: Tuple[float, float]
    vertical_span_px: float
    corners_px: Tuple[Tuple[float, float], ...]
    aperture_corners_px: Optional[Tuple[Tuple[float, float], ...]]


@dataclass(frozen=True)
class GateFrame:
    frame_id: int
    source_timestamp_ns: int
    capture_monotonic_ns: int
    received_monotonic_ns: int
    candidates: Tuple[GateCandidate, ...]

    @property
    def detected(self) -> bool:
        return bool(self.candidates)


def inner_aperture_corners(
    parent_index: int,
    contours,
    hierarchy,
    outer_area: float,
) -> Optional[Tuple[Tuple[float, float], ...]]:
    """Return an unambiguous inner aperture in TL, TR, BR, BL order."""
    children = []
    child_index = hierarchy[parent_index][2]
    while child_index != -1:
        if hierarchy[child_index][3] == parent_index:
            area = cv2.contourArea(contours[child_index])
            if area > 0.0:
                children.append((area, contours[child_index]))
        child_index = hierarchy[child_index][0]
    if not children:
        return None
    children.sort(key=lambda item: item[0], reverse=True)
    area, contour = children[0]
    if area / sum(item[0] for item in children) < 0.85:
        return None
    if not 0.08 <= area / outer_area <= 0.88:
        return None

    points = contour.reshape(-1, 2).astype(np.float64)
    sums = points[:, 0] + points[:, 1]
    differences = points[:, 0] - points[:, 1]
    corners = np.array((
        points[np.argmin(sums)],
        points[np.argmax(differences)],
        points[np.argmax(sums)],
        points[np.argmin(differences)],
    ))
    if len({tuple(point) for point in corners}) != 4:
        return None
    polygon = corners.astype(np.float32).reshape((-1, 1, 2))
    if not cv2.isContourConvex(polygon) or cv2.contourArea(polygon) < 16.0:
        return None
    return tuple(
        tuple(float(value) for value in point)
        for point in corners
    )


def detect_gates(jpeg: bytes) -> Tuple[GateCandidate, ...]:
    """Return each visible gate center and outer vertical span."""
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return ()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 100, 100), (12, 255, 255)),
        cv2.inRange(hsv, (168, 100, 100), (180, 255, 255)),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, hierarchy = cv2.findContours(
        mask,
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if hierarchy is None:
        return ()

    candidates = []
    hierarchy = hierarchy[0]
    for index, relation in enumerate(hierarchy):
        if relation[3] != -1:
            continue
        outer_area = cv2.contourArea(contours[index])
        if outer_area < MIN_GATE_AREA_PX:
            continue
        x, y, width, height = cv2.boundingRect(contours[index])
        center = (x + width / 2.0, y + height / 2.0)

        perimeter = cv2.arcLength(contours[index], True)
        quadrilateral = None
        for epsilon in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08):
            points = cv2.approxPolyDP(contours[index], epsilon * perimeter, True)
            if len(points) == 4:
                quadrilateral = points.reshape(4, 2).astype(np.float64)
                break
        if quadrilateral is None:
            quadrilateral = cv2.boxPoints(cv2.minAreaRect(contours[index]))
        ordered = np.array(sorted(quadrilateral, key=lambda point: point[1]))
        top = sorted(ordered[:2], key=lambda point: point[0])
        bottom = sorted(ordered[2:], key=lambda point: point[0])
        corners = np.array((top[0], top[1], bottom[1], bottom[0]))

        aperture = inner_aperture_corners(
            index,
            contours,
            hierarchy,
            outer_area,
        )
        left_span = float(np.linalg.norm(corners[3] - corners[0]))
        right_span = float(np.linalg.norm(corners[2] - corners[1]))
        vertical_span = 0.5 * (left_span + right_span)
        if vertical_span >= MIN_GATE_SPAN_PX:
            candidates.append(GateCandidate(
                center_px=center,
                vertical_span_px=vertical_span,
                corners_px=tuple(
                    tuple(float(value) for value in point)
                    for point in corners
                ),
                aperture_corners_px=aperture,
            ))

    if not candidates:
        return ()
    candidates.sort(key=lambda item: item.vertical_span_px, reverse=True)
    return tuple(candidates)


def solve_square_translation(
    candidate: GateCandidate,
    square_size_m: float,
    focal_x_px: float,
    focal_y_px: float,
    center_x_px: float,
    center_y_px: float,
) -> Optional[Tuple[float, float, float]]:
    """Return square-center translation in the OpenCV camera frame."""
    if square_size_m <= 0.0:
        raise ValueError("gate size must be positive")
    if not candidate.aperture_corners_px:
        return None
    half = square_size_m / 2.0
    object_points = np.array((
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
    ), dtype=np.float64)
    camera_matrix = np.array((
        (focal_x_px, 0.0, center_x_px),
        (0.0, focal_y_px, center_y_px),
        (0.0, 0.0, 1.0),
    ), dtype=np.float64)
    try:
        solved, _, translation = cv2.solvePnP(
            object_points,
            np.asarray(candidate.aperture_corners_px, dtype=np.float64),
            camera_matrix,
            np.zeros((4, 1)),
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        return None
    if not solved:
        return None
    result = tuple(float(value) for value in translation.reshape(3))
    if result[2] <= 0.0 or not all(np.isfinite(result)):
        return None
    return result


class GateCamera:
    """Own the AIGP camera socket and expose the newest complete frame."""

    def __init__(self, port: int = 5600):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)
        self._socket.bind(("127.0.0.1", port))
        self._socket.settimeout(0.25)
        self._lock = threading.Lock()
        self._latest: Optional[GateFrame] = None
        self._stop = threading.Event()
        self._source_offset_ns: Optional[int] = None
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

    def latest(self) -> Optional[GateFrame]:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._socket.close()
        self._thread.join(timeout=1.0)

    def _receive(self) -> None:
        frames = {}
        completed = set()
        while not self._stop.is_set():
            try:
                packet, _ = self._socket.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            received_ns = time.monotonic_ns()
            if len(packet) < HEADER.size:
                continue
            frame_id, chunk_id, chunk_count, jpeg_size, payload_size, source_ns = \
                HEADER.unpack_from(packet)
            payload = packet[HEADER.size:]
            if payload_size != len(payload) or frame_id in completed:
                continue
            candidate_offset_ns = received_ns - source_ns
            if (
                self._source_offset_ns is None
                or candidate_offset_ns < self._source_offset_ns
            ):
                self._source_offset_ns = candidate_offset_ns
            frame = frames.setdefault(
                frame_id,
                {"chunks": {}, "count": chunk_count, "size": jpeg_size, "source": source_ns},
            )
            frame["chunks"][chunk_id] = payload
            if len(frame["chunks"]) == frame["count"]:
                try:
                    jpeg = b"".join(
                        frame["chunks"][index] for index in range(frame["count"])
                    )
                except KeyError:
                    del frames[frame_id]
                    continue
                del frames[frame_id]
                completed.add(frame_id)
                completed = {item for item in completed if item > frame_id - 60}
                if len(jpeg) != frame["size"]:
                    continue
                candidates = detect_gates(jpeg)
                sample = GateFrame(
                    frame_id=frame_id,
                    source_timestamp_ns=frame["source"],
                    capture_monotonic_ns=frame["source"] + self._source_offset_ns,
                    received_monotonic_ns=received_ns,
                    candidates=candidates,
                )
                with self._lock:
                    self._latest = sample
            for old_id in [item for item in frames if item < frame_id - 30]:
                del frames[old_id]
