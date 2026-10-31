"""Compare rotation and integrated rates within recorded native ODOMETRY packets."""

import argparse
import base64
import json
import math
from pathlib import Path

from pymavlink.dialects.v20 import common as mavlink

from common.math import Quaternion
from miniflight.attitude import gyro_step
from target.aigp.native import sha256


def measure(path, start, end):
    if not math.isfinite(start) or not math.isfinite(end) or start >= end:
        raise ValueError("start and end must be increasing finite device seconds")
    decoder, samples = mavlink.MAVLink(None), {}
    for line in path.open():
        row = json.loads(line)
        if row['event'] != 'update':
            continue
        for packet in row['telemetry']:
            message, = decoder.parse_buffer(base64.b64decode(packet['wire'], validate=True))
            if message.get_type() == 'ODOMETRY' and start <= message.time_usec*1e-6 <= end:
                if (message.frame_id, message.child_frame_id) != (1, 8):
                    raise ValueError("expected native LOCAL_NED / BODY_FRD odometry")
                samples[message.time_usec*1e-6] = message
    times = sorted(samples)
    if len(times) < 2:
        raise ValueError("the interval needs at least two odometry samples")
    def rotation(message):
        w, x, y, z = message.q
        q = Quaternion(w, -x, y, -z)  # Native quaternion -> the adapter's FRD/NED axes.
        q.normalize()
        return q
    initial = integrated = rotation(samples[times[0]])
    for a, b in zip(times, times[1:]):
        p, q = samples[a], samples[b]
        gyro = tuple(-(x+y)/2 for x, y in zip((p.rollspeed, p.pitchspeed, p.yawspeed),
                                              (q.rollspeed, q.pitchspeed, q.yawspeed), strict=True))
        integrated = gyro_step(integrated, gyro, b-a)
    actual = rotation(samples[times[-1]])
    def degrees(a, b):
        return math.degrees((a.conjugate()*b).to_rotation_vector().magnitude())
    return dict(trace=str(path), trace_sha256=sha256(path), source_sha256=sha256(Path(__file__)),
                samples=len(times), start_s=times[0], end_s=times[-1],
                max_gap_s=max(b-a for a, b in zip(times, times[1:])), observed_deg=degrees(initial, actual),
                integrated_deg=degrees(initial, integrated), residual_deg=degrees(integrated, actual))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('start', type=float, help='device seconds, inclusive')
    parser.add_argument('end', type=float, help='device seconds, inclusive')
    args = parser.parse_args()
    print(json.dumps(measure(args.trace, args.start, args.end)))
