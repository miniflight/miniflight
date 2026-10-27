"""Project privileged VQ1 gate geometry onto recorded camera frames."""

import argparse
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np

from common.math import Quaternion
from miniflight import Frame
from target.aigp.experiments.recording import ReplayClient, read_metadata
from target.aigp.native import sha256


def observations(path):
    client = ReplayClient()
    with path.open() as source:
        for row in map(json.loads, source):
            if row["event"] == "call" and row["name"] == "poll":
                packets = client.feed(row.get("result", ()))
                frames = tuple(packet.decoded for packet in packets if isinstance(packet.decoded, Frame))
                yield client.read(timeout=0), frames, client.gates


def project(gate, motion, attitude, width, height, pivot_to_center):
    """Gate local X is width, Z is down; camera X is right, Y down, Z forward."""
    corners = np.array([(-width/2, 0, -height/2), (width/2, 0, -height/2),
                        (width/2, 0, height/2), (-width/2, 0, height/2)]) + pivot_to_center
    world = corners @ Quaternion(*gate.orientation).as_rotation_matrix().T + gate.position
    rotation = Quaternion.from_euler(attitude.roll, attitude.pitch, attitude.yaw).as_rotation_matrix()
    mount = Quaternion.from_euler(0, math.radians(20), 0).as_rotation_matrix()
    camera = ((world - motion.position) @ rotation @ mount)[:, (1, 2, 0)]
    return None if np.any(camera[:, 2] <= 0) else 320 * camera[:, :2] / camera[:, 2:] + (320, 180)


def render(trace, geometry_path, output):
    if read_metadata(trace)["target"] != "vq1.r1":
        raise ValueError("the measured geometry belongs to VQ1 R1")
    geometry = json.loads(geometry_path.read_text())
    motions, attitudes = {}, {}
    for state, _, gates in observations(trace):
        if state.motion is not None:
            motions[state.motion.received_at] = state.motion
        if state.attitude is not None:
            attitudes[state.attitude.received_at] = state.attitude
    if not motions or not attitudes or not gates:
        raise ValueError("recording lacks privileged pose or gate geometry")
    command = ["ffmpeg", "-v", "error", "-n", "-f", "rawvideo", "-pixel_format", "bgr24",
               "-video_size", "1280x424", "-framerate", "30", "-i", "pipe:0", "-an",
               "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    output.parent.mkdir(parents=True, exist_ok=True)
    gaps, first_ns, previous, previous_tick, decoded = [], None, None, 0, 0
    with subprocess.Popen(command, stdin=subprocess.PIPE) as writer:
        for _, frames, _ in observations(trace):
            for frame in frames:
                if frame.received_at < max(min(motions), min(attitudes)):
                    continue
                motion = motions[min(motions, key=lambda stamp: abs(stamp-frame.received_at))]
                attitude = attitudes[min(attitudes, key=lambda stamp: abs(stamp-frame.received_at))]
                gaps.append((frame.received_at-motion.received_at, frame.received_at-attitude.received_at))
                image = frame.bgr.copy()
                for width, height, color in ((1.50, 1.50, (255, 255, 0)),
                                             (geometry["opening_width_m"], geometry["opening_height_m"], (0, 255, 255))):
                    for gate in gates:
                        uv = project(gate, motion, attitude, width, height, geometry["pivot_to_center_local_ned_m"])
                        if uv is not None and np.max(np.abs(uv)) < 100000:
                            points = np.round(uv).astype(np.int32)
                            cv2.polylines(image, [points], True, color, 1, cv2.LINE_AA)
                            cv2.putText(image, str(gate.id+1), tuple(points[0]), 0, .4, color, 1, cv2.LINE_AA)
                panel = cv2.copyMakeBorder(np.hstack((frame.bgr, image)), 32, 32, 0, 0, cv2.BORDER_CONSTANT)
                labels = [("Recorded camera", (8, 21)), ("Privileged gate projection", (648, 21)),
                          (f"Frame {frame.id} | cyan: PDF 1.50 m | yellow: measured 1.48 m | receipt aligned; capture offset unknown", (8, 412))]
                for label, origin in labels:
                    cv2.putText(panel, label, origin, 0, .5, (255, 255, 255), 1, cv2.LINE_AA)
                first_ns = frame.time_ns if first_ns is None else first_ns
                tick = round((frame.time_ns-first_ns)*30/1e9)
                if previous is not None:
                    for _ in range(tick-previous_tick):
                        writer.stdin.write(previous.tobytes())
                previous, previous_tick, decoded = panel, tick, decoded+1
        if previous is not None:
            writer.stdin.write(previous.tobytes())
        writer.stdin.close()
    if writer.returncode:
        raise subprocess.CalledProcessError(writer.returncode, command)
    summary = dict(trace=str(trace.resolve()), trace_sha256=sha256(trace), geometry=str(geometry_path.resolve()),
                   geometry_sha256=sha256(geometry_path), source_sha256=sha256(Path(__file__)), frames=decoded,
                   video_frames=previous_tick+1, duration_s=(previous_tick+1)/30,
                   alignment="nearest host receipt; camera epoch and pose boot clocks are not synchronized",
                   receipt_gap_p95_ms=(np.percentile(np.abs(gaps), 95, axis=0)*1000).tolist(),
                   camera=dict(fx=320, fy=320, cx=320, cy=180, uptilt_degrees=20),
                   scope="Projected gate midplanes; no image detections, pixel-error score or parameter fitting.")
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2)+"\n")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("geometry", type=Path, help="measured VQ1 R1 geometry JSON")
    parser.add_argument("output", type=Path, help="new MP4 path; requires ffmpeg")
    args = parser.parse_args()
    print(json.dumps(render(args.trace, args.geometry, args.output)))
