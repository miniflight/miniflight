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

Every controller input and returned command is recorded. Separate `sent` records
identify commands actually sent through the client. The log also contains arm
requests, received heartbeats/acknowledgements/collisions, source-file hashes,
the executable hash, and completion/connection-close status. Camera recording is
disabled. These are the observations consumed by the controller, not a lossless
recording of every native sensor packet.

Replay reconstructs the controller's inputs and checks every command, phase, trial,
and terminal exception without connecting to the simulator. A changed controller
can intentionally fail replay. Choose unused trace names: existing files are never
overwritten.

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

## Next controller

Start with a fixed-point hold using VQ1 motion and attitude. Keep the computation
explicit: position/velocity error → desired acceleration → attitude/thrust target
→ body rates. The simulator continues to own rate stabilization and motor mixing.
After the hold works, reuse the gate-selection policy and move through one gate,
then the complete course. No additional command plane is needed for that step.
