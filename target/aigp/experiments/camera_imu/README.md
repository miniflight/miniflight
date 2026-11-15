# camera + imu

Public-sensor course pilot:

```sh
python -m target.aigp.experiments.camera_imu course OUTPUT --gates 6 --speed 5
```

```mermaid
flowchart LR
  C[Public camera pixels] --> G[Gate corners and metric pose]
  I[Public gyro and accelerometer] --> A[Local attitude]
  A --> K[Motion prediction]
  U[Our motor commands and vehicle calibration] --> K
  G --> E[Delayed position and velocity corrections]
  K --> E
  E --> M[Gate map and estimated crossings]
  M --> P[Approach and visibility waypoints]
  E --> P
  P --> R[Body rates and motor mixer]
  R --> D[Drone]
  R --> U
  D --> C
  D --> I
  N[Native pose, course and race state] --> S[Postflight scorer]
```

The pilot receives no native pose, attitude, course geometry, gate counter, GO or
finish state. Its start uses a fixed allowance after an acknowledged reset. Its
own crossing estimates advance the mission. Clock alignment uses public sensor
timestamps and packet arrival times. The course profile loads no native sensor
DLLs. Public acceleration supports initial gravity calibration and takeoff.
After the first confirmed gate, our motor commands and vehicle model predict
translation; camera velocity innovations learn a residual acceleration bias.
Position and heading use a local frame chosen at the stationary start.
`replay --poison` also falsifies native race start, validity and finish for this
profile. Raw traces are temporary and bounded; the course video and scores remain.

| Native navigation signal | Estimated replacement |
| --- | --- |
| Attitude | Gravity initialization and public gyro integration |
| Position and velocity | Gate-relative camera measurements, causal velocity fits and command-model prediction |
| Gate poses and order | Known-size image geometry and current/next-gate hypotheses |
| Gate progress | Camera-range crossing estimates propagated with estimated displacement |

Native race, pose, course, actuator and contact data remain in the recorder and
postflight scorer. The pilot receives heartbeat/ACK transport events and sensors.
Five unchanged six-gate trials produced three native-valid finishes and two
contact-free passes: 52.80 s and 47.87 s. One finish included contact; two trials
stopped after gate three. All five normal and poisoned replays matched exactly.
Across the two clean runs, local position RMS was 1.10/1.56 m, velocity RMS
0.34/0.57 m/s and attitude RMS 0.24/0.37 degrees. Timing uses estimated public
sensor clock alignment. These are direct-arena results, above the ten-second
target. Protocol, source hashes and failures: [course.validation.json](course.validation.json).
The earlier `joint` experiments below use instrumented sensor timing.

Joint position, velocity and depth feedback for one slow 20-degree turn:

```sh
python -m target.aigp.experiments.camera_imu joint OUTPUT --shift -.5
python -m target.aigp.experiments.camera_imu.joint_score OUTPUT
```

The turn runs at 1 m/s after camera initialization. Gate pose updates stop before
takeover. The same joint estimate drives the turn and braking. The second command
recomputes retained optimizer inputs; native pose is used only by the flight scorer.
Raw traces are temporary; only compact scores, optimizer inputs and a short clip remain.
The final profile passed two unchanged live repetitions. Position RMS was
0.04/0.23 m and velocity RMS 0.015/0.080 m/s. Gate pose observations stopped before
joint takeover. Four earlier failures and the endpoint limits are in
[joint.validation.json](joint.validation.json).

Return observations of the same mapped gate after a one-second gap:

```sh
python -m target.aigp.experiments.camera_imu joint OUTPUT --shift -.5 --joint-init reprojection --gate-return
```

The first and latest measured gate views remain in the fit after the gate leaves
view. Gate corrections disabled on the same sensor inputs provide a causal replay
comparison. All trials and failures are in [gate-return.validation.json](gate-return.validation.json).

One gate with camera position/velocity correcting the IMU estimate:

```sh
python -m target.aigp.experiments.camera_imu one target/aigp/.runtime/gate-estimator/run-01 --shift -.5
```

Background-map replay uses `source-lossless.mp4`, `camera-input.json` and
`gyro-input.json` in `SENSORS`. Native reference poses remain in `FLIGHT` for scoring:

```sh
python -m target.aigp.experiments.camera_imu.background SENSORS
python -m target.aigp.experiments.camera_imu.background_score FLIGHT SENSORS
```

`background SENSORS --map-uncertainty` compares covariance-weighted `multi`
against the original twelve-view fit as `uniform`, using the same map points.
The mixed fresh-turn result is in [uncertainty.validation.json](uncertainty.validation.json);
weighting remains optional.

Two fresh turn comparisons are in [multiview.validation.json](multiview.validation.json).
Those background-map comparisons ran in replay. The newer `joint` mode above
uses a continuously updated position, velocity and depth fit for live feedback.

This runs at 2 m/s. Gate size and pixels supply translation; IMU rotation puts
measurements in one local frame. A 0.5 s camera history supplies velocity.
Position and velocity corrections apply at their measurement times and carry
forward through the IMU history. Corrections require all aperture sides to be
at least 60 pixels long and reprojection RMS at most 1 pixel. The gate anchor
stays fixed after the first frame meeting those conditions.
`score.json` reports gate-relative position/velocity errors for camera, corrected
IMU and an uncorrected IMU on the same flight, plus capture-to-controller-output
age and the velocity fit's effective age. Simulator motion is read only by the
post-flight scorer. Full traces and images are temporary, capped at 128 MiB per
trace, and deleted after normal replay and poisoned-state replay. Only small
summaries and a camera-error CSV remain.

The final frozen controller passed 2/2 one-gate runs from opposite sideways
starts. Across those flights, corrected gate-relative position/velocity RMS
was 0.125 m / 0.066 m/s; uncorrected IMU on the same flights was
0.249 m / 0.107 m/s. Trusted camera observations were 0.095 m / 0.090 m/s RMS.
Median capture-to-output age was 31 ms. The velocity fit's effective time was
234 ms before its latest image, accounted for by the delayed IMU correction.
Earlier failures and exact replay checks are in
[estimation.validation.json](estimation.validation.json). These are simulator
results for one gate; IMU orientation still supplies the common frame.

From the repository root, with Python 3.11+, Wine and MinGW-w64 installed:

```sh
python -m pip install -e '.[aigp,planning]'
python -m target.aigp.experiments.camera_imu build
python -m target.aigp.experiments.camera_imu live target/aigp/.runtime/camera-imu/run-01 --shift -.5
python -m target.aigp.experiments.camera_imu replay target/aigp/.runtime/camera-imu/run-01/flight.jsonl --poison
python -m target.aigp.experiments.camera_imu score target/aigp/.runtime/camera-imu/run-01
python -m target.aigp.experiments.camera_imu video target/aigp/.runtime/camera-imu/run-01
```

Use a new output directory for each run. `video` requires `ffmpeg`.
`live` writes `score.json`; `live` and `score` exit unsuccessfully when the
two-gate acceptance checks fail. Replay reproduces the recorded outcome,
including a retained flight failure.
`build` requires `x86_64-w64-mingw32-gcc` and `x86_64-w64-mingw32-objcopy`.
Helpers are compiled and cached under `.runtime/camera-imu-builds/`.
Tested on macOS with the repository's VQ1 R1 simulator setup.

The probes require executable SHA256
`d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9`.
The runner checks it before launch and restores the normal Lua configuration
when its owned simulator stops.

Steering uses aperture pixels, a nominal 1.50 m opening, full-rate producer IMU
increments and measured capture times. Initial tilt comes from gravity with zero
local yaw. The sideways setup move uses IMU feedback, then establishes a local
origin before the approach. Both gate approaches retain the same IMU history.
The camera retains the next aperture before the handoff, and an early height
waypoint keeps it in view. Native race state handles GO and lifecycle checks.
Native pose, gate coordinates and gate count are recorded for post-flight scoring.

`replay --poison` replaces incoming native pose and gate coordinates and holds
the native gate counter at zero while comparing every controller update and
outgoing command. `score` checks native passes, crossing offsets, arming,
in-flight resets, collisions and IMU continuity. The video draws accepted image
corners in green, IMU predictions in amber and look-ahead candidates in blue.

The final prototype passed two continuous two-gate runs with the same controller.
The recordings replayed 793 and 784 commands exactly. Three earlier handoff
failures are retained in the local experiment record. Results are in
[validation.json](validation.json), with the [recorded flight](https://x.com/__miniflight__/status/2119060568414081178).

Fresh checks of this packaged entry point passed 1/2 runs. The retained failure
passed gate one but did not confirm enough look-ahead observations for gate two.
Gate-two acquisition remains intermittent.
