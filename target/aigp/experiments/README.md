# AIGP experiments

These tools are optional. The simulator and gate controllers do not import them.

The R1 goal is six valid gates, no collision, and a native finish below 10 seconds
with a computed reference and PID. MPC comes after that.

Each experiment asks one question and changes one variable. Retain the initial
state, reference, configuration, source/binary hashes, raw signals and native
result. Device time, game time and host receipt time remain separate.

| Experiment | Question | Change | Evidence |
| --- | --- | --- | --- |
| Control signals | Where does tracking fail? | Expose the reference time and PID acceleration request. | Reference/measured motion, attitude, body rates, thrust, source age and clipping. Exact command replay. |
| Vehicle response | What force and rotation can a command produce? | One thrust or body-rate pulse at a time. | Thrust response, rotation delay and drag, checked on withheld pulses. State the measured range. |
| Gate clearance | Can the whole tilted vehicle cross? | Crossing offset or attitude at one gate. | Native opening and vehicle footprint, orientation and collision reports. Packet dimensions describe outer bounds. |
| Offline search | Which feasible motion finishes sooner? | Search duration and thrust/body-rate sequences. | Full motion under measured dynamics, ordered gate openings and actuator limits. Crossing points and times stay free. |
| Native flight | Can PID follow the computed motion? | Increase search effort or pace, one at a time. | Six valid gates, no collision, exact replay, closed connection and native finish below 10 seconds. |

Measured on 2026-10-11: PID followed a 35-second reference through all six gates
in 35.18 native seconds; all 2,068 control updates replayed exactly. The same
controller settings hit gate 4 with a 25-second reference. The PID correction
exceeded the configured acceleration cap before contact. A separate 35-second
attempt stopped on stale position data after five gates, with no collision.

That reference fixed gate centers and crossing times and minimized jerk. It
checks the follower; it does not establish the fastest feasible whole pose.
Search should choose the motion. Geometry, dynamics and measured limits define
the constraints; a preferred bend shape or gate-specific braking rule does not.

On 2026-10-12, an initial upward reference acceleration of 2 m/s² made the
vehicle gain height during its initial attitude correction. With the same
controller gains, the 32-second reference completed ten consecutive offscreen
runs in 32.129–32.171 native seconds, with no collisions and exact command/signal
replay. The [verification record](../../../test/fixtures/vq1_r1_lift_32s.validation.json)
retains the configuration and trace hashes, and notes earlier windowed failures.

```sh
python -m test.aigp_regression r1_motors --reference test/fixtures/vq1_r1_lift_32s.npz --offscreen --camera
```

Omit `--offscreen` to watch the native window. `--camera` also saves its final
image. The optional `planning` extra supports `python -m target.aigp.experiments.plan_motion`;
this planner still fixes gate centers and crossing fractions and minimizes absolute jerk.

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
from target.aigp.controllers.r1_beautiful import Controller
from target.aigp.experiments.recording import RecordedClient, RecordedController, recording, replay
from target.aigp.experiments.r1_signals import r1_signals
from target.aigp.simulator import AIGPSimulator

with recording("r1.jsonl", {"target": "vq1.r1"}) as record:
    client = RecordedClient(record, camera_port=None)
    sim = AIGPSimulator(RecordedController(Controller, record, signals=r1_signals), client=client)
    result = sim.rollout()
    record(event="result", race_finished=result is not None and result.completed,
           connection_closed=not client.connected)

updates = replay(Controller, "r1.jsonl", signals=r1_signals)
```

Use a fresh controller for replay. It checks returned commands and exceptions
against the recorded observations, without connecting to a simulator. A matching
replay does not prove race completion; the native result records that separately.
Omit `camera_port=None` to include camera images. Recordings are never overwritten.
The optional signal projection records R1's reference time, position, velocity,
acceleration and the PID request before command limits. Replay checks those values too.
