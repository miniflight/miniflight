# AIGP commands

The arena supplies native arrivals through
[`BaseController.update(telemetry, frames)`](../controllers/README.md).
`SimulatorClient.send` encodes the returned command. The runner handles arming
and disarming.

The VQ1 build-3391 native receiver implements six control modes. All six have
native flight evidence; the acceleration, attitude and motor checks below were
run on 2026-10-08. The PDF's two named messages omit the direct actuator message.

| Native request | MAVLink message | Mask | Typed adapter |
| --- | --- | --- | --- |
| Position, metres NED | `SET_POSITION_TARGET_LOCAL_NED` (84) | 3576 | `PositionNed` |
| Velocity, m/s NED | same | 3527 | `VelocityNed` |
| Acceleration, m/s² NED; force, N | same | 3135; force 3647 | raw |
| Attitude, wxyz quaternion, and thrust 0–1 | `SET_ATTITUDE_TARGET` (82) | 7 | raw |
| Body rates and thrust 0–1 | same | 144 for native rad/s | `BodyRates`, FRD conversion |
| Four direct motor controls | `SET_ACTUATOR_CONTROL_TARGET` (139) | none | raw |

`SimulatorClient.send` exposes the three typed commands; `client.mav` exposes
the raw forms. The runner owns arming, race timing, heartbeats and command cadence.
The [MAVLink definitions](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
define the fields; native support was checked separately against the executable.

Native acceleration becomes desired roll/pitch and collective thrust, followed
by the native attitude/rate loops. It has no acceleration-error feedback. Each
tilt angle is limited to ±0.4 rad (22.9°). Vertical thrust is
`clamp(mass * (9.81 - down_acceleration) / 29.9, 0, 1)`, without tilt compensation.
Force mode divides the three requested values by mass first.

Use full XYZ groups in local NED. The receiver checks bits 0, 3 and 6 for the
entire position, velocity and acceleration groups; it ignores the frame selector.
Enabled acceleration overwrites the position/velocity calculation. Otherwise,
position takes precedence over velocity. Yaw angle takes precedence over yaw rate;
ignoring both retains the previous yaw request. Rate mask 128 selects native
stick values; adding the vendor's bit 16 selects rad/s. The actuator receiver
copies controls 0–3 and ignores controls 4–7 and the group selector.

Hovering belongs to the selected plane. A fixed `PositionNed` target lets VQ1
hold position. A zero `VelocityNed` target asks it to hold zero velocity, without
specifying a fixed location. With an attitude target, the simulator stabilizes the
requested attitude; our controller must still choose thrust and tilt to hold position.
With `BodyRates`, our controller also chooses the rates needed to reach that
attitude. Zero rates request a stop in rotation. Hover needs suitable thrust and attitude.

`PositionNed` and `VelocityNed` express separate physical requests but share the
adapter's `SET_POSITION_TARGET_LOCAL_NED` encoder. The command type selects the
active fields and mask; zero-valued coordinates remain active requests.
`BodyRates` uses the attitude-target encoder. Masks and frame conversions stay
inside the adapter; `BaseController.update` returns the original command values.

## source boundary

the local betaflight snapshot is release `2026.6.2` at `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`

its main PID task calls receiver command processing then the rate controller
then mixing and motor output in `src/main/fc/core.c` at `taskMainPidLoop`
`src/main/fc/rc.c` applies channel scaling rate curves and smoothing
`src/main/flight/pid.c` compares the requested rate to the filtered gyro in degrees per second
`src/main/flight/mixer.c` combines axis corrections with throttle
`src/main/drivers/motor.c` dispatches outputs to the selected ESC driver

the serial boundary is different from those internal firmware functions
`MSP_SET_RAW_RC` supplies receiver channels through `src/main/rx/msp.c`
the MAVLink receiver accepts `RC_CHANNELS_OVERRIDE` through `src/main/rx/mavlink.c`
neither is a physical body rate setpoint without the configured channel mapping and rate curve
`MSP_SET_MOTOR` writes `motor_disarmed` values used by the disarmed mixer path
it sets motor test outputs while disarmed

`MSP_RAW_IMU` returns accelerometer ADC counts and gyro degrees per second without a device timestamp
a serial adapter must resolve sensor scaling frames and timing before exposing vehicle observations
those wire values cannot be passed through as this API's timestamped SI samples

this release also has optional navigation and MAVLink mission support
there is no betaflight Python target implemented here

## simulator boundary

the installed VQ1 and VQ2 build 3391 executables were checked against the existing Ghidra analysis

| binary | sha256 |
| --- | --- |
| vq1 | `d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9` |
| vq2 | `68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b` |

VQ2 retains the body rate attitude local NED and actuator input handlers
its attitude local position and odometry output workers are stubs in both R1 and R2
the client exposes the three existing position velocity and body rate command paths
it decodes native telemetry

The acceleration, attitude and motor forms still use the
[raw wire interface](wiring.md#the-remaining-wire-interface). The 2026-10-08 audit
traced dispatch at `0x14104bf70`, NED at `0x14104b8f0`, and attitude at
`0x14104b4f0`. VQ2 has matching NED receiver control flow and constants;
its flight response was not measured.

VQ1 tests sent commands at 50 Hz after a 0.6-second steady position hold, then
returned to that hold between pulses. Actual outgoing packets and native pose,
IMU and motor packets are in `.runtime/measurements/control-planes-20261008/`
with the probe source, binary/source hashes, summaries and disassembly.
All completed traces have exact controller replay, one arm/disarm pair and no collisions.

| Native test | Observed response |
| --- | --- |
| Velocity ±1 m/s north for 1.5 s | End velocity about ±0.915 m/s |
| Acceleration ±2 m/s² north/east | Late velocity slopes about ±1.8 m/s² |
| Acceleration +1 / −1 m/s² down | +1.56 / −1.28 m/s² |
| Acceleration +8 m/s² north | 22.94° pitch; +3.48 m/s² |
| Force +1 N north | 7.23° pitch; +1.18 m/s² |
| Raw quaternion pitch +5° / −5° | FRD pitch −5.01° / +5.01° |
| Four motor values 0.22 / 0.30 | Reported outputs about 0.22 / 0.30; descent / ascent |

These are measured transients, not exact acceleration tracking. The attitude
probe covers pitch; complete quaternion axis conversion and motor corner order
remain uncalibrated. Reported motor values are not labelled RPM.

the bundled [specification](VQ1-Technical-Specification-00.02.pdf) defines the NED and body frames
the [vendor controller](reference/PyAIPilotExample-v4/controller.py) defines the build 3390 radian extension
the adapter preserves that bit and the existing NED masks
standard message fields are defined by [MAVLink](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)

Live VQ1 build-3391 pulses exposed angular sign differences on the wire.
The adapter negates all three transmitted body rates. Received fields keep their
wire signs; controllers apply the input mappings needed by their flight routines.
`SimulatorClient.telemetry` is a read-only view of the latest original MAVLink
messages, keyed by message name. A held view follows later packets.
The [measurements and captured regressions](body-rates.md) establish this conversion
against attitude changes, NED motion, and the camera heading; VQ2 has not received
the same physical-response validation.

The runner passes ordered packet arrivals and completed camera frames to the
controller. `r1_gates` returns `PositionNed`; `r1_body_rates` returns `BodyRates`.

## track input

`TrackInfo` arrives on the packet that completes a native course transfer.
Its gates retain raw NED base positions, wxyz orientations and overall bounds.
The controller checks the geometry and derives opening centers if needed.
`SimulatorClient.gates` caches the last complete transfer, including redacted values.

The captured VQ1 build 3391 packet is preserved in `test/fixtures/vq1_track.json`.
