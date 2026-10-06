# Body-rate interface

These measurements cover VQ1 build 3391, executable SHA-256
`d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9`.
They validate the existing `BodyRates` path before implementing a gate controller.
VQ2 and physical flight controllers have not received this response validation.

## Frames

The Python API uses forward/right/down body axes and local north/east/down world
coordinates. Rates and angles are radians. Collective thrust is normalized to 0–1.
The AIGP adapter performs these build-3391 conversions:

| Data | Conversion |
| --- | --- |
| Python body-rate command → wire | Negate roll, pitch, and yaw rates |
| Wire `HIGHRES_IMU` gyro → `State.gyro` | Negate all three components |
| Wire `ATTITUDE` → `State.attitude` | Keep roll; negate pitch and yaw |
| Position, velocity, reported acceleration | Preserve wire signs |

The transmitted attitude-target mask remains 144: ignore the quaternion and enable
the vendor's physical-rad/s extension. `SimulatorClient.telemetry` keeps the raw
messages; controller inputs use the converted values.

The signs were checked against positive and negative pulses, integrated gyro versus
attitude changes, the direction of NED velocity changes, and camera heading. At one
captured yaw, the first gate appeared at horizontal pixel 358. The converted pose
predicts 358.2; the unconverted pose predicts 293.7. This checks heading, not a full
camera calibration. The camera's vertical projection remains outside this validation.

`test/fixtures/vq1_body_rates_frames.json` retains wire samples and the camera
measurement. Tests decode the recorded values and check angular kinematics, NED
motion direction, and horizontal camera projection. Controllers should not add
their own AIGP sign corrections.

Nonfinite attitude angles produce `None`, like unavailable or stale observations.
The runner accepts positive command rates below 100 Hz; the default remains 50 Hz.

## Probe

Run from the repository root after installing the normal AIGP dependencies:

```sh
python -m examples.aigp.probe_body_rates thrust thrust.jsonl --thrust .26 .27 .28 --duration .8
python -m examples.aigp.probe_body_rates rates rates.jsonl --thrust .266 --rate .25 --duration .6
python -m examples.aigp.probe_body_rates rates larger-rates.jsonl --thrust .266 --rate .75 --duration .35
python -m examples.aigp.probe_body_rates replay rates.jsonl
```

The probe runs through `AIGPSimulator`. It waits for native GO, uses `PositionNed`
to settle three metres above the initial position, applies a bounded body-rate
or thrust pulse, and recovers to the position hold. This is a measurement routine
with position assistance, not a body-rate hover or race controller.
After all pulses recover it stops through `StopIteration`; probe completion does
not claim a native race finish.

Settling requires position error below 0.15 m, speed below 0.15 m/s, roll/pitch
below 0.04 rad, and angular speed below 0.03 rad/s for 0.6 seconds. A pulse stops
if displacement from the hold exceeds 2 m, speed exceeds 5 m/s, or roll/pitch
exceeds 0.5 rad. Pulses last at most one simulator second. Missing observations,
failure to settle, and the wall-clock deadline stop the run through the shared
cleanup path.

Position/velocity commands ignore yaw and yaw rate. Switching from a yaw-rate
command to position mode retained the previous yaw request in the measured build.
The rate probe therefore explicitly sends zero body rates, at the same collective
thrust, before returning to position mode. Zero rates stop rotation; position
control subsequently levels the vehicle and removes translation.

The optional `target.aigp.experiments.recording` wrapper records every controller input, including
gate geometry, and returned command. Separate `sent` records
identify commands actually sent through the client. The log also contains arm
requests, received heartbeats/acknowledgements/collisions, source-file hashes,
the executable hash, and completion/connection-close status. Camera recording is
disabled. These are the observations consumed by the controller, not a lossless
recording of every native sensor packet.

Replay takes a fresh controller and checks its returned commands and exceptions
without connecting to the simulator or inspecting private phase/trial fields.
The probe's CLI supplies a fresh `Probe` and reports whether its sequence completed.
Existing probe traces remain readable, including those written before geometry was
recorded. A changed controller can intentionally fail replay. Choose unused trace
names: existing files are never overwritten.

## Measured response

The final rate runs used twelve pulses each: positive and negative commands on all
three axes, each followed by a zero-rate recovery pulse. Both completed without
reported collisions and replayed exactly: 1,463 updates at 0.25 rad/s and 1,299
updates at 0.75 rad/s. Cleanup sent disarm requests and closed all connections.

Absolute mean gyro values below average the positive and negative trials, excluding
the first 0.15 seconds of each pulse:

| Requested magnitude | Pulse length | Roll | Pitch | Yaw |
| --- | --- | --- | --- | --- |
| 0.25 rad/s | 0.60 s | 0.2455 | 0.2455 | 0.2207 |
| 0.75 rad/s | 0.35 s | 0.7270 | 0.7283 | 0.6593 |

The first 90% crossings occurred about 35–67 ms after the sampled request origin,
with roughly 20 ms observation spacing. This is coarse response timing, not an
isolated transport-latency measurement. Yaw briefly crosses that threshold and then
settles below the request, so the crossing is not a settling-time guarantee. The
native response scales across the tested range; the maximum rate and saturation
boundary have not been established. The adapter does not compensate for tracking
error by rescaling requested rates.

The later [yaw tracking study](yaw-tracking.md) confirms the steady deficit with
longer steps and raw-wire capture. Its experimental gyro feedback can be selected
in the rate probe; the gate routine uses the position controller's output directly.

Vertical acceleration was estimated by fitting NED down velocity against its own
device timestamps, again excluding the first 0.15 seconds:

| Thrust | Downward acceleration |
| --- | --- |
| 0.260 | +0.323 m/s² |
| 0.270 | −0.208 m/s² |
| 0.280 | −0.749 m/s² |

Three one-second trials at 0.266 produced −0.003, −0.013, and −0.015 m/s². **0.266 is
a measured hover-thrust starting point for this build**, not a universal constant
or a validated thrust curve. Those 483 recorded updates also replayed exactly.

The final runs' command-interval medians were about 19.9 ms and their 95th
percentiles were 23.8–24.3 ms. Host receipt age is distinct from sensor capture
latency; these runs do not establish a hard real-time guarantee.

The local experiment bundle is under `.runtime/measurements/body-rates/`:
`rates-04.jsonl`, `rates-05.jsonl`, and `thrust-03.jsonl` contain the final runs;
`summary.json`, `analyze.py`, and the response plots retain the analysis.
Earlier traces document the wire-frame and mode-switching investigation.

## Python position control

`miniflight.position.position_control` now closes position and attitude feedback
using VQ1 motion and attitude. It is stateless PD: position error becomes a bounded
velocity target, velocity error becomes acceleration, and the thrust direction
becomes roll/pitch targets. Euler feedback is converted to body rates, including
the cross-axis terms needed when tilted. Collective thrust compensates for the
current tilt. The simulator continues to own rate stabilization and motor mixing.

The AIGP controller supplies hover thrust 0.266 and an approximate local thrust
slope of 53.5 m/s² per unit normalized thrust, based on the pulse measurements
above. These are vehicle configuration, not generic control constants or a model
of the full thrust curve. Default limits are 4 m/s speed, 3 m/s² horizontal and
vertical acceleration, 0.35 rad desired tilt, and 0.75 rad/s body-rate magnitude.

Run the gate routine with:

```sh
python target/aigp/simulator.py vq1.r1 --controller r1_body_rates
```

`r1_body_rates` reuses the published-gate target policy, holds the initial heading,
and outputs only `BodyRates`. It requires motion, attitude, and course geometry.
Its `target_position` and `target_yaw` are the desired position and heading;
current observations remain in `state`. It sends the numerical position
controller's output directly, without the separate yaw-rate probe correction.
The existing runner handles missing observations, native GO/finish, and cleanup.
`BaseController[BodyRates]` declares its output type; deliberately mixed routines
can declare a union. The target still validates every actual returned command.

Native validation on build 3391 used the same executable hash stated above and
the 50 Hz runner with a 0.3 s observation timeout:

- A body-rate-only takeoff and fixed-point hold completed in 13.04 simulator
  seconds. Over the final ten seconds, maximum position error was 0.0508 m and
  maximum speed was 0.1756 m/s; final position error was 0.0077 m. All 601
  controller updates, including the terminal `StopIteration`, replayed exactly.
- The R1 routine completed all six gates with 2,260 body-rate commands. The
  native finish packet reported `active_gate_index=6` and
  `race_finish_time_ns=52349807739`. The recorded controller interval was
  52.20 simulator seconds. All 2,260 updates replayed exactly.

Neither run reported a collision. Both disarmed through the runner and closed
their owned simulator connections. These are VQ1 results, not VQ2 or physical
hardware validation. Camera input was disabled for these motion/attitude runs.

The local experiment harness, source hashes, traces, and summary are under
`.runtime/measurements/r1-body-rates/`: `validate.py`, `hold-01.jsonl`,
`race-01.jsonl`, and `summary.json`. The hold harness requires ten continuous
seconds below 0.25 m position error and 0.2 m/s speed, and bounds its total run.

Current end-to-end flight regression runs through
`python -m test.aigp_regression r1_body_rates` from the repository root.
It requires all six native R1 gates,
native finish, disarm and connection cleanup, then replays every recorded update.
Each run saves its trace under `.runtime/regressions/`.
`--camera` checks image delivery and records frame IDs, times, shape, and dtype.
The two race controllers use numerical observations, so their regression traces
omit image pixels. `--record-frames` enables full pixel capture for replay.
The general `RecordedController` wrapper still includes pixels by default;
callers can select `frames=False` when their controller does not use images.

After separating the core observation records from the host adapter layer, a
second native R1 run (`race-02.jsonl`) again completed all six gates using only
body-rate commands. All 2,397 updates replayed exactly and the owned connection
closed. The refactored package passed 226 unit tests and seven UDP integration
checks; 24 core checks also passed with site packages disabled (`python -S`).

The earlier `race-ownership-01.jsonl` completed all six native R1 gates without
the experimental yaw correction, and all 2,342 updates replayed exactly.
After moving the optional tools into `experiments/`, `race-arena-01.jsonl` again
completed all six native gates. All 2,312 updates replayed exactly and the owned
connection closed. Both R1 controllers use `r1_gates.gate_target` to choose the
same position targets; only their outgoing command planes differ.
