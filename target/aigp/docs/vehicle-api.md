# AIGP commands

The arena supplies native arrivals through
[`BaseController.update(telemetry, frames)`](../controllers/README.md).
`SimulatorClient.send` encodes the returned command. The runner handles arming
and disarming.

The PDF names two command messages. Their MAVLink definitions describe five
basic request forms: position, velocity, acceleration/force, attitude plus thrust,
and body rates plus thrust. Masks select the active fields. The adapter exposes:

| command | units | loop closed by the target |
| --- | --- | --- |
| `PositionNed` | metres in local NED | position and lower loops |
| `VelocityNed` | metres per second in local NED | velocity and lower loops |
| `BodyRates` | radians per second in FRD and collective thrust 0 to 1 | angular rate and motor mixing |

AIGPSimulator owns race timing heartbeats command cadence and process lifetime

`SET_ATTITUDE_TARGET` can carry desired attitude as a wxyz quaternion and
collective thrust, or desired body rates and thrust. TRPY uses the attitude form:
roll, pitch and yaw are angles, converted to a quaternion on the wire.
`BodyRates` supplies angular speeds in rad/s.
The native receiver has an attitude branch. Raw requests can be written through
`client.mav`; this form has no typed command or native flight verification here.
See the [attitude message](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET).

`SET_POSITION_TARGET_LOCAL_NED` carries position, velocity, acceleration/force,
and optional heading fields. Our two NED command types select position or
velocity and ignore the other fields. Raw acceleration/force requests are available
through `client.mav`; this form has no typed command or native flight verification. See the [NED message](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED).

Hovering belongs to the selected plane. A fixed `PositionNed` target lets VQ1
hold position. A zero `VelocityNed` target asks it to hold zero velocity, without
specifying a fixed location. With TRPY, the simulator would stabilize the requested
attitude; our controller must still choose thrust and tilt to hold position.
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

the broader attitude acceleration and direct actuator inputs have no typed commands here
the [raw wire interface](wiring.md#the-remaining-wire-interface) exposes their packet fields
their presence in the parser is not a tested control contract
reported motor output channels are retained with the wire active mask and values without claiming RPM units

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
