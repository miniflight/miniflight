# camera + imu

The [controller model](../../../../controller.md) defines the physics, joint
drone/map state, online work, between-flight refinement and accuracy targets.
The present implementation is an experimental public-sensor course pilot.
Maps can be retained between flights. Optional cold localization estimates an
unknown airborne starting pose and velocity against the saved map before gate1.

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

Use `--map-prior PREVIOUS/estimated-map.json` to retain the joint gate/rim/size
posterior. The next flight starts with fresh position and velocity, keeps map
cross-covariances and corrects the map online. Both flights must share calibration;
default startup assumes the original launch frame. Rejected pixels cannot establish a lock;
the mapped path guides acquisition until the estimator admits a real image.

Add `--unknown-start --scoring-frame FRAME.json` to test cold localization. The
harness moves the drone, discards the setup pilot, and starts a fresh controller
without its pose or history. While unlocalized, public gyro/force feedback and a
bounded pixel-bearing pitch correction maintain an acquisition view. Map-pair
hypotheses fit position, velocity and attitude over a fresh image/IMU window;
ambiguous associations wait. The search runs outside the motor-control process.
`FRAME.json` contains the prior exact-scoring metrics' `alignment` object, frozen
before this trial. It is opened by postflight scoring, never navigation.
[Cold-start trials and all MAEs](relocalization.validation.json),
[error plots](relocalization.validation.png), [startup](relocalization.startup.png).

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

Joint map reuse and admitted handoffs completed six gates in 27.38 s and 27.62 s
from opposite setup shifts. The next repeat stopped at 3/6. Position/velocity/
attitude/depth MAE on the faster lap: 1.90 m / 0.46 m/s / 0.50 deg / 0.29 m.
Low MAE, five clean repeats and under 10 s remain unachieved.
[All eight new trials](map.validation.json) and [plots](map.validation.png).
Fixed-command estimation replays raised gate-relative MAE from 0.36/0.38 m to
17.60/12.40 m when image factors were removed. Removing only the upright tilt
factor improved attitude MAE from 0.50/0.49 deg to 0.21/0.16 deg; this variant
still needs closed-loop validation. The same record retains both ablations.
[All recent development outcomes](accuracy.validation.json) and
[error overview](accuracy.validation.png) retain failures and partial flights.
[Earlier continuous trials](continuous.validation.json) remain available.

Command/body response is measured independently of navigation. Fit the small
rotational response model on an older public flight, then freeze it before a new
flight. Forecasts hold the last command and use no future gyro, force or pixels:

```sh
python -m target.aigp.experiments.camera_imu.proprioception fit OLD response.json \
  --calibration target/aigp/experiments/camera_imu/thrust-response.model.json
python -m target.aigp.experiments.camera_imu.proprioception evaluate NEW CHECK \
  --model response.json --estimates NEW-EXACT
```

`course ... --camera-gap 0.1` withholds100ms of images every3s after tracking.
The IMU and motor feedback continue. `gate_filter_events SOURCE OUTPUT
--camera-gap 0.4` instead removes image factors from the same fixed-command
flight; it tests estimation, not closed-loop completion. Finish now requires
1m of estimated clearance beyond the last gate. [All response/gap scores and
failed flights](proprioception.validation.json), [prediction plots](proprioception.response.png)
and [paired gap plots](proprioception.paired.png) retain the measurements.
Effective public-clock lags do not separate physical actuation from sensor
publication delay. The rotational response model remains a diagnostic; live
attitude propagation continues to use gyro measurements.

`--view-recovery` enables an experimental slow approach/centering recovery.
Seven200ms-gap development trials did not establish a complete lap; it is off
by default. [All recovery failures and MAEs](recovery.validation.json) and
[plots](recovery.validation.png) retain the results. [Position-derived velocity](recovery.collision.png)
supports a collision at the upper gate3 frame for the earlier10.3m/s reversal.
The public accelerometer does not represent that impulse.

The removed instrumented, old joint-turn and immutable route-snapshot modes are
in Git history. Archived process traces require their recorded revision; current
code rejects unsupported profiles rather than replaying them through a different
controller. Earlier reports remain evidence, including the native-fitted
[course](course.validation.json) and [gravity](gravity.validation.json) comparisons.
