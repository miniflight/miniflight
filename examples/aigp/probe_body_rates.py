"""Measure VQ1 body-rate/thrust pulses between settled position holds.

python -m examples.aigp.probe_body_rates thrust trace.jsonl --thrust .26 .27 .28 --duration .8
python -m examples.aigp.probe_body_rates rates trace.jsonl --thrust .266 --rate .25
python -m examples.aigp.probe_body_rates yaw trace.jsonl --thrust .266 --rate .25 --duration 2 --repeat 3
python -m examples.aigp.probe_body_rates replay trace.jsonl
python -m examples.aigp.probe_body_rates analyze trace.jsonl
"""

import argparse
import json
import math
from pathlib import Path
import signal

from miniflight import BodyRates
from target.aigp.experiments.probe import Probe, analyze_yaw, replay, run


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("thrust", "rates", "yaw", "replay", "analyze"))
    parser.add_argument("trace", type=Path)
    parser.add_argument("--thrust", type=float, nargs="+", default=(.26, .27, .28))
    parser.add_argument("--rate", type=float, default=.25, help="positive and negative pulse size on each axis, rad/s")
    parser.add_argument("--duration", type=float, default=.6, help="pulse duration in simulator seconds")
    parser.add_argument("--hz", type=float, default=50, help="command frequency, positive and below 100 Hz")
    parser.add_argument("--repeat", type=int, default=1, help="repeat the pulse sequence")
    parser.add_argument("--yaw-feedback", action="store_true", help="test VQ1 gyro-based yaw tracking correction")
    parser.add_argument("--tail", type=float, default=.5, help="final sensor window in seconds for analyze")
    args = parser.parse_args(argv)
    if args.mode == "replay":
        print(json.dumps(replay(args.trace)))
        return
    if args.mode == "analyze":
        print(json.dumps(analyze_yaw(args.trace, tail_seconds=args.tail), indent=2))
        return
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    if args.mode in ("rates", "yaw"):
        if len(args.thrust) != 1 or not math.isfinite(args.rate) or not 0 < args.rate <= 1:
            parser.error("rates requires one --thrust value and 0 < --rate <= 1")
        axes = ("yaw_rate",) if args.mode == "yaw" else ("roll_rate", "pitch_rate", "yaw_rate")
        commands = [BodyRates(**{axis: sign * args.rate}, thrust=args.thrust[0])
                    for axis in axes for sign in (1, -1)]
    else:
        commands = [BodyRates(thrust=thrust) for thrust in args.thrust]

    def stop(signum, frame):
        raise TimeoutError("probe interrupted or exceeded 180 wall-clock seconds")

    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGALRM):
        signal.signal(signum, stop)
    signal.alarm(180)
    try:
        print(json.dumps(run(args.trace, commands * args.repeat, args.duration, hz=args.hz, yaw_feedback=args.yaw_feedback)))
    finally:
        signal.alarm(0)


if __name__ == "__main__":
    main()
