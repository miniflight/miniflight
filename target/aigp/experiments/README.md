# AIGP experiments

These tools are optional. The simulator loads recording tools when `--record` is
requested; gate controllers do not import them.

The R1 goal is six valid gates, no collision, and a native finish below 10 seconds
with a computed reference and PID. MPC comes after that.

Each experiment asks one question and changes one variable. Retain the initial
state, reference, configuration, source/binary hashes, raw signals and native
result. Device time, game time and host receipt time remain separate.
New flight recordings also capture owned native startup, process exit checks,
and shutdown. Process replay consumes those results without launching Wine.

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

The 30-second reference then exposed an arming error: the rate integral included
an IMU interval from before arming. Starting its clock from fresh feedback after
the accepted arm command gave ten consecutive clean runs in 30.129–30.187 native
seconds, with unchanged gains and exact command/signal replay. The
[30-second record](../../../test/fixtures/vq1_r1_lift_30s.validation.json) retains
all ten traces' hashes and the earlier failed attempt.

```sh
python -m test.aigp_regression r1_motors --reference test/fixtures/vq1_r1_lift_30s.npz --offscreen --camera
```

Omit `--offscreen` to watch the native window. `--camera` also saves its final
image. The optional `planning` extra supports `python -m target.aigp.experiments.plan_motion`;
this planner still fixes gate centers and crossing fractions and minimizes absolute jerk.

`plan_race` uses SLSQP to minimize finish time with free crossing points and phase
durations. It constrains thrust using fitted resistance, reserves room for the
native collision body and tracking error, and bounds speed, tilt and direction slew:

```sh
python -m target.aigp.experiments.plan_race test/fixtures/vq1_r1_lift_30s.npz /tmp/r1-plan.npz --model test/fixtures/vq1_r1_model.json
```

The model is a local fit; this search produces a candidate for native validation.

The 26.066-second searched reference passed ten consecutive native runs in
26.198–26.263 seconds, with six valid gates, zero collisions and 16,172 updates
replayed exactly. Its [validation record](../../../test/fixtures/vq1_r1_free_26s.validation.json)
retains the measured model, generation and runtime hashes, and all ten results.
Use `--reference test/fixtures/vq1_r1_free_26s.npz --offscreen --camera` with the
native regression command above to reproduce the check.

With measured resistance compensation and a 7.0 m/s planning speed cap, the
25.482-second reference completed ten consecutive reset-and-flight cycles in one
native process at 25.470–25.518 seconds. All six gates passed without collision;
14,223 updates replayed exactly. The [record](../../../test/fixtures/vq1_r1_free_25s.validation.json)
also retains two earlier reset failures. Use `test/fixtures/vq1_r1_free_25s.npz`
and `--config test/fixtures/vq1_r1_resistance.config.json` with the repeat command
below. Resistance compensation remains explicit in this configuration.

On 2026-10-13, a body-frame resistance fit and revised local thrust calibration
gave ten clean runs in 18.950–19.028 native seconds, with 10,967 updates replayed
exactly. PID gains remained 2/0/2; the rate ceiling was 1.25 rad/s. Use
`test/fixtures/vq1_r1_free_19s.npz` with
`--config test/fixtures/vq1_r1_body.config.json` in the repeat command below.
The [record](../../../test/fixtures/vq1_r1_free_19s.validation.json) retains the
force fit, held-out errors, source hashes, and earlier failed experiments.

Saved FRD body rates from the planned thrust direction reduced turn lag. The
14.237-second reference then gave ten clean resets in 14.212–14.277 native seconds,
with 8,629 updates replayed exactly. Use `test/fixtures/vq1_r1_free_14s.npz` with
`--config test/fixtures/vq1_r1_body_14s.config.json`. Its
[record](../../../test/fixtures/vq1_r1_free_14s.validation.json) retains the fit,
source hashes, all ten timings and reset counts, and earlier failed experiments.

The 9.819-second reference gave ten clean flights in 9.780–9.860 native seconds,
with six gates, zero collisions and 6,423 updates replayed exactly. The
[sub-ten record](../../../test/fixtures/vq1_r1_sub10.validation.json) keeps the
executed settings, source hashes and timings. Its saved body rates use FRD rad/s;
position, velocity and acceleration use NED metres and seconds.

Start the repeat runner, observer, and local browser server in separate terminals:

```sh
python -m target.aigp.experiments.repeat test/fixtures/vq1_r1_sub10.npz --config test/fixtures/vq1_r1_sub10.config.json --output target/aigp/.runtime/experiments/pitwall --count 10 --finish-before 10 --render
python -m target.aigp.experiments.pitwall target/aigp/.runtime/experiments/pitwall/current.json target/aigp/.runtime/experiments/pitwall/view --current
python -m http.server 8765 --bind 127.0.0.1 --directory target/aigp/.runtime/experiments/pitwall/view
```

Open `http://127.0.0.1:8765/`. Use a new output directory for each experiment.
Omit `--count` to repeat until Ctrl+C. Each trial resets the same native process,
creates a fresh controller, and runs all regression checks. The ledger retains failures.
`--finish-before` sets the native race deadline: an unfinished race stops and disarms at the limit; a finish at or above it fails.
Use `--controller r1_beautiful` to follow the same reference through native body-rate
control; `r1_motors` remains the default and uses Python rate PI. Each keeps its existing timeout.
The observer reads records in a separate process and publishes complete snapshots;
the browser displays quantities, source age, and finish times across trials.

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

The repeat runner records and replays each reset and attached flight, including
failed attempts. `miniflight.replay.Calls` checks external call order, arguments,
results and errors, and supplies the recorded clock. AIGP packet codecs and race
observations stay in `target/aigp/experiments/process_replay.py`.

Replay executes the existing `fresh_race()` and `AIGPSimulator.rollout()` functions.
The flight still uses `Vehicle.read()` to construct observations and
`Vehicle.send()` to validate commands. It checks heartbeat, reset, arm, command,
disconnect and terminal outcomes. Camera input retains the native JPEG datagrams.
No sockets are opened and no simulator is launched during replay:

```sh
python -m target.aigp.experiments.process_replay path/to/trial-00001.reset.jsonl path/to/trial-00001.jsonl
```

Commands are compared at the Target interface, before MAVLink encoding. The
native physics and launcher are outside this replay. A reproduced crash is a
matched failure, not a successful flight. The repeat runner keeps that failure
in its ledger and stops on a replay mismatch.

Older recordings lack complete clock and receive boundaries. They support the
controller replay below, not exact replay of the original runtime or reset.

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
