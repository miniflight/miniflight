"""Measure camera-only rotation over the recorded opposite-yaw probe."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import cv2
import numpy as np

from common.math import Quaternion
from miniflight import Frame, vehicle
from miniflight.vision import rotation_between_bearings
from target.aigp.experiments.recording import ReplayClient


CAMERA = np.array(((320., 0, 320), (0, 320, 180), (0, 0, 1)))
BODY_TO_CAMERA = Quaternion.from_euler(0, np.radians(20), 0).as_rotation_matrix().T[[1, 2, 0]]


def image_rotation(previous, current):
    previous, current = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in (previous, current))
    points = cv2.goodFeaturesToTrack(previous, 500, .01, 8)
    if points is None:
        return None
    tracked, found, _ = cv2.calcOpticalFlowPyrLK(previous, current, points, None, winSize=(21, 21), maxLevel=3)
    backward, returned, _ = cv2.calcOpticalFlowPyrLK(current, previous, tracked, None, winSize=(21, 21), maxLevel=3)
    keep = (found.ravel() != 0) & (returned.ravel() != 0) & (np.linalg.norm(backward-points, axis=2).ravel() <= 1)
    points, tracked = points[keep].reshape(-1, 2), tracked[keep].reshape(-1, 2)
    if len(points) < 8:
        return None
    rays = [np.column_stack((pixels, np.ones(len(pixels)))) @ np.linalg.inv(CAMERA).T for pixels in (points, tracked)]
    rotation = rotation_between_bearings(*rays)
    projected = rays[0] @ rotation.T @ CAMERA.T
    residual = np.linalg.norm(projected[:, :2]/projected[:, 2:] - tracked, axis=1)
    return dict(rotation=rotation.tolist(), matches=len(points), median_residual_px=float(np.median(residual)))


def measure(trace):
    segments = []
    with trace.open() as source:
        for row in map(json.loads, source):
            if row['event'] != 'sent':
                continue
            command = row['command']
            key = (command['kind'], command.get('yaw_rate', 0))
            if not segments or segments[-1]['key'] != key:
                if segments:
                    segments[-1]['end'] = row['host_time']
                segments.append(dict(key=key, start=row['host_time']))
    windows = [(s['start']-.25, segments[i+1]['end']-.25) for i, s in enumerate(segments[:-1])
               if s['key'][0] == 'BodyRates' and s['key'][1] and i+1 < len(segments) and 'end' in segments[i+1]]
    selected = {time: None for window in windows for time in window}
    attitudes, motions = {}, {}
    client = ReplayClient()
    vehicle.bind(client)
    try:
        with trace.open() as source:
            for row in map(json.loads, source):
                if row['event'] != 'call' or row['name'] != 'poll':
                    continue
                packets = client.feed(row.get('result', ()))
                state = vehicle.read(timeout=0)
                if state.attitude is not None:
                    attitudes[state.attitude.received_at] = state.attitude
                if state.motion is not None:
                    motions[state.motion.received_at] = state.motion
                for packet in packets:
                    if isinstance(frame := packet.decoded, Frame):
                        for deadline in selected:
                            if frame.received_at <= deadline:
                                selected[deadline] = frame
    finally:
        vehicle.disconnect()
    results = []
    for start, end in windows:
        a, b = selected[start], selected[end]
        result = dict(window_host_s=(start, end), frames=None if a is None or b is None else (a.id, b.id), estimate=None)
        if a is not None and b is not None:
            result.update(receipts=(a.received_at, b.received_at), estimate=image_rotation(a.bgr, b.bgr))
            # Native pose is read only after the image-only estimate, for scoring.
            result['truth'] = [asdict(attitudes[min(attitudes, key=lambda t: abs(t-frame.received_at))]) for frame in (a, b)]
            result['motion'] = [asdict(motions[min(motions, key=lambda t: abs(t-frame.received_at))]) for frame in (a, b)]
            if result['estimate'] is not None:
                native = [Quaternion.from_euler(p['roll'], p['pitch'], p['yaw']).as_rotation_matrix() for p in result['truth']]
                truth = native[0].T @ native[1]
                estimate = BODY_TO_CAMERA.T @ np.array(result['estimate']['rotation']).T @ BODY_TO_CAMERA
                result['error_deg'] = float(np.degrees(np.arccos(np.clip((np.trace(truth.T @ estimate)-1)/2, -1, 1))))
                result['estimated_yaw_deg'] = float(np.degrees(np.arctan2(estimate[1, 0], estimate[0, 0])))
                result['native_yaw_deg'] = float(np.degrees(np.arctan2(truth[1, 0], truth[0, 0])))
                result['translation_m'] = float(np.linalg.norm(np.array(result['motion'][1]['position'])-result['motion'][0]['position']))
        results.append(result)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    print(json.dumps(measure(args.trace), indent=2))
