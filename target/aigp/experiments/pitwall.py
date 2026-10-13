"""Watch recorded signals outside the flight process; publish a local view."""

import argparse
import json
from pathlib import Path
import shutil
import time

from miniflight.diagnostics import JsonLines, chart_data, write_snapshot
from target.aigp.experiments.diagnostics import Observe, snapshot


def watch(source, output, current=False, once=False):
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).parents[3] / "miniflight/pitwall.html", output / "index.html")
    reader, trace, ledger, history = None, None, None, []
    reference = None
    try:
        while True:
            pointer = json.loads(source.read_text()) if current and source.exists() else dict(trace=str(source))
            path = Path(pointer["trace"])
            if current and not source.exists():
                time.sleep(.5)
                continue
            if path != trace:
                if reader is not None:
                    reader.close()
                reader, observer, samples, trace = JsonLines(path), Observe(), [], path
            frame, camera = trace.with_suffix(".frame.jpg").resolve(), output / "frame.jpg"
            if frame.exists() and (not camera.is_symlink() or camera.readlink() != frame):
                temporary = output / "frame.tmp"
                temporary.symlink_to(frame)
                temporary.replace(camera)
            samples.extend(sample for row in reader.poll() if (sample := observer.consume(row)) is not None)
            view = snapshot(observer, samples, trace)
            if "ledger" in pointer:
                if ledger is None:
                    ledger = JsonLines(pointer["ledger"])
                history.extend(ledger.poll())
                passed = sum(row["passed"] for row in history)
                view["summary"].update({"Clean trials": passed, "Failed trials": len(history)-passed})
                if history and history[-1]["trace"] == str(trace) and not history[-1]["passed"]:
                    error = history[-1]["error"]
                    view["note"] = f"Trial failed: {error['type']}: {error['message']}"
                reference = (observer.metadata.get("reference") or {}).get("duration_s", reference)
                results = [dict(trial=row["trial"], finish=row.get("finish_time_s"), reference=reference) for row in history[-100:]]
                chart, = chart_data(results, [("Native finish across trials", "s", [("Reference", "reference"), ("Clean finish", "finish")])],
                                    x_key="trial", x_label="Trial")
                chart["mode"] = "lines+markers"
                view["charts"].insert(0, chart)
            view.update(published_at=time.time(), completed=observer.status.get("completed", False))
            write_snapshot(output / "latest.json", view)
            if once:
                return view
            time.sleep(.05)
    finally:
        if reader is not None:
            reader.close()
        if ledger is not None:
            ledger.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--current", action="store_true", help="source is an atomic JSON pointer to the current trace")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    watch(args.source, args.output, args.current, args.once)
