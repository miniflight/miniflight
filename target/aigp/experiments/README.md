# AIGP experiments

These tools are optional. The simulator and gate controllers do not import them.

`probe.py` runs bounded body-rate measurements. The command-line example remains
under `examples/aigp/`:

```sh
python -m examples.aigp.probe_body_rates yaw trace.jsonl --thrust .266 --rate .25 --duration 2
python -m examples.aigp.probe_body_rates analyze trace.jsonl
python -m examples.aigp.probe_body_rates replay trace.jsonl
```

`yaw_tracking.py` contains the bounded integral correction used by the rate
probe's `--yaw-feedback` experiment. It owns the accumulated correction,
previous request, and settling timer. The gate routines do not use this extra
rate loop. See [yaw tracking](../docs/yaw-tracking.md) for measurements and limits.

## recording and replay

`recording.py` can record any controller's inputs and returned commands. It does
not change the simulator or controller interface:

```python
from target.aigp.controllers.r1_body_rates import Controller
from target.aigp.experiments.recording import RecordedClient, RecordedController, recording, replay
from target.aigp.simulator import AIGPSimulator

with recording("r1.jsonl", {"target": "vq1.r1"}) as record:
    client = RecordedClient(record, camera_port=None)
    sim = AIGPSimulator(RecordedController(Controller(), record), client=client)
    result = sim.rollout()
    record(event="result", race_finished=result is not None and result.finished,
           connection_closed=not client.connected)

updates = replay(Controller(), "r1.jsonl")
```

Use a fresh controller for replay. It checks returned commands and exceptions
against the recorded observations, without connecting to a simulator. A matching
replay does not prove race completion; the native result records that separately.
Omit `camera_port=None` to include camera images. Recordings are never overwritten.

## historical probes

`probe_vq1_motor.py` is a historical raw-MAVLink experiment. Its actuator mapping
has not been validated. It bypasses the shared runner, arms directly, and restores
idle outputs at the end rather than disarming. It is retained as research material,
not as a supported flight example or a miniflight control plane.

Use the examples under `examples/aigp/` for the supported runner lifecycle.

`vq1_arm_probe.py` is another historical diagnostic. It uses the removed
`SimulatorClient._message` method and is retained for reference only.
