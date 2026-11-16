# camera + imu

The [controller model](../../../../controller.md) defines the physics, joint
drone/map state, online work, between-flight refinement and accuracy targets.
The present implementation is an experimental public-sensor course pilot.
Persistent cross-flight SLAM and relocalization remain unimplemented.

Run one gate with the joint observer and continuous reference:

```sh
python -m target.aigp.experiments.camera_imu course OUTPUT --gates 1 --speed 2 \
  --dynamics-model target/aigp/experiments/camera_imu/thrust-response.model.json \
  --joint-estimator --moving-start --gyro-attitude --joint-rim --partial-gate \
  --direct-launch --continuous-path --image-time-uncertainty --upright-attitude
```

Use `--gates 6` for the full course. Speed is a reference cap; the scorer reports
actual flight motion. The runtime receives camera pixels, public gyro and
accelerometer samples, public timing/ACKs and our own motor commands. Native pose,
course, actuator reports and race state enter the postflight scorer only. An
explicit public dynamics model is required; there is no native-fixture fallback.

The joint filter retains drone/gate covariance and rewinds delayed pixel updates.
The nominal 1.5 m aperture supplies scale with a 0.03 m consider uncertainty.
Outer-frame dimensions are learned jointly; only measured corners enter factors.
`--image-time-uncertainty` supplies one shared 10 ms epoch-uncertainty prior per
image. It is an assumption, not a measured delay or a timestamp shift.
`--upright-attitude` partitions vertical-edge information from the other pixel
rows, so the same corner noise contributes once.

Every run exports compact inputs, state outputs, scores and plots before removing
the bounded raw traces. Ordinary and poisoned-state process replays run before
deletion. Canonical estimator replay remains available from the compact files:

```sh
python -m target.aigp.experiments.camera_imu.gate_filter_events OUTPUT REPLAY
python -m target.aigp.experiments.camera_imu.gate_filter_event_score OUTPUT REPLAY
```

The first command reconstructs estimation without private fields and checks every
recorded mean and covariance hash. The second freezes those outputs, then reads
private references for scoring. `plots/`, `pipeline.json` and `pipeline-timing.csv.gz`
record observation quality, errors and compute/command timing. True exposure-to-
actuation delay remains unknown. [pipeline.spec.json](pipeline.spec.json) records
the current input contract and gaps.

Public thrust-response collection uses `--thrust-excitation train` or `validate`
with one gate and joint estimation, without `--continuous-path`. Freeze a fit
before opening the separate validation flight:

```sh
python -m target.aigp.experiments.camera_imu.thrust_response TRAIN FIT \
  --prior target/aigp/experiments/camera_imu/dynamics.model.json
python -m target.aigp.experiments.camera_imu.thrust_response VALID CHECK \
  --prior target/aigp/experiments/camera_imu/dynamics.model.json --frozen FIT/model.json
```

The public fixture `test/fixtures/thrust_response/training` reproduces the fit.
The curve covers mean commands 0.21–0.335. Its independent short-window vertical
response RMS was 0.039 m; this does not establish full-flight navigation accuracy
or actuator delay. [Calibration record](thrust-response.validation.json).

A single clean six-gate run reached 31.65 s, with position/velocity/attitude/depth
MAE 1.02 m / 0.41 m/s / 1.06 deg / 0.38 m. Other trials failed. Super-low MAE, the
five-flight robustness criterion and under 10 s remain unachieved.
[All recent development outcomes](accuracy.validation.json) and
[error overview](accuracy.validation.png) retain failures and partial flights.
[Earlier continuous trials](continuous.validation.json) remain available.

The removed instrumented, old joint-turn and immutable route-snapshot modes are
in Git history. Archived process traces require their recorded revision; current
code rejects unsupported profiles rather than replaying them through a different
controller. Earlier reports remain evidence, including the native-fitted
[course](course.validation.json) and [gravity](gravity.validation.json) comparisons.
