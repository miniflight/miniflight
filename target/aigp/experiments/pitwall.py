"""Watch recorded signals outside the flight process; publish a local view."""

import argparse
import json
from pathlib import Path
import shutil
import time

from miniflight.diagnostics import JsonLines, write_snapshot
from target.aigp.experiments.diagnostics import Observe, snapshot


def watch(source, output, current=False, once=False):
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).parents[3] / "miniflight/pitwall.html", output / "index.html")
    reader, trace = None, None
    try:
        while True:
            path = Path(json.loads(source.read_text())["trace"]) if current and source.exists() else source
            if current and not source.exists():
                time.sleep(.5)
                continue
            if path != trace:
                if reader is not None:
                    reader.close()
                reader, observer, samples, trace = JsonLines(path), Observe(), [], path
            samples.extend(sample for row in reader.poll() if (sample := observer.consume(row)) is not None)
            view = snapshot(observer, samples, trace)
            view.update(published_at=time.time(), completed=observer.status.get("completed", False))
            write_snapshot(output / "latest.json", view)
            if once:
                return view
            time.sleep(.5)
    finally:
        if reader is not None:
            reader.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--current", action="store_true", help="source is an atomic JSON pointer to the current trace")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    watch(args.source, args.output, args.current, args.once)
