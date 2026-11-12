# camera + imu

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

Two fresh turn comparisons are in [multiview.validation.json](multiview.validation.json).
The background estimator runs in replay; it does not drive the flight controller.

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
