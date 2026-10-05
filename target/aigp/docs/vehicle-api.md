# vehicle api

`Vehicle` is a connection to a vehicle, not a controller or a simulator runner

Observation records are defined in `miniflight.state`; AIGP depends on those core
records. Public imports from `miniflight` and the previous `miniflight.vehicle`
observation imports remain valid. The host connection interface `Target` stays
in the adapter layer under `target/`; the core does not import it. `Vehicle`
delegates to the supplied connection without requiring a target base class.
The core imports no NumPy runtime; camera adapters own pixel storage. `State`
is an observation snapshot with independent sample clocks, not an estimator
result or controller memory. Gate data, race status, launch/cleanup, and AIGP
recording remain under `target/aigp`.

```python
from miniflight import Vehicle
from target.aigp.simulator import SimulatorClient

vehicle = Vehicle(SimulatorClient())
vehicle.connect()
try:
    state = vehicle.read()
    print(state.gyro, state.acceleration)
    print(vehicle.commands)
finally:
    vehicle.disconnect()
```

`read` waits for a fresh IMU sample and updates `vehicle.state`
reading `vehicle.position` or `vehicle.velocity` never does IO
optional observations remain `None` until reported
each slower observation keeps its own timestamp and receipt time

`send` accepts one explicit command
`arm` and `disarm` are separate requests
`commands` lists the target's supported command types
an unsupported command raises `NotImplementedError` without emulation

The PDF names two command messages. Their MAVLink definitions describe five
basic request forms: position, velocity, acceleration/force, attitude plus thrust,
and body rates plus thrust. Masks select the active fields; these are not five
separate messages. The adapter currently exposes three forms:

| command | units | loop closed by the target |
| --- | --- | --- |
| `PositionNed` | metres in local NED | position and lower loops |
| `VelocityNed` | metres per second in local NED | velocity and lower loops |
| `BodyRates` | radians per second in FRD and collective thrust 0 to 1 | angular rate and motor mixing |

the corresponding convenience methods are `position_ned` `velocity_ned` and `body_rates`
they each send one command and do not run background loops
AIGPSimulator owns race timing heartbeats command cadence and process lifetime

`SET_ATTITUDE_TARGET` can carry desired attitude as a wxyz quaternion and
collective thrust, or desired body rates and thrust. TRPY uses the attitude form:
roll, pitch and yaw are angles, converted to a quaternion on the wire. It is not
the `BodyRates` form, whose three rotational fields are rad/s.
The native receiver has an attitude branch, but that form is not exposed or
flight-verified by this adapter yet.
See the [attitude message](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET).

`SET_POSITION_TARGET_LOCAL_NED` carries position, velocity, acceleration/force,
and optional heading fields. Our two NED command types select position or
velocity and ignore the other fields. The acceleration/force form is not exposed
or flight-verified here. See the [NED message](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED).

Hovering belongs to the selected plane. A fixed `PositionNed` target lets VQ1
hold position. A zero `VelocityNed` target asks it to hold zero velocity, without
specifying a fixed location. With TRPY, the simulator would stabilize the requested
attitude; our controller must still choose thrust and tilt to hold position.
With `BodyRates`, our controller also chooses the rates needed to reach that
attitude. Zero rates alone do not level a tilted drone, and zero thrust does not hover.

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
it is a motor test interface rather than an armed flight control plane

`MSP_RAW_IMU` returns accelerometer ADC counts and gyro degrees per second without a device timestamp
a serial adapter must resolve sensor scaling frames and timing before exposing vehicle observations
those wire values cannot be passed through as this API's timestamped SI samples

this release also has optional navigation and MAVLink mission support
that does not make its mission interface the same as streamed local NED setpoints
there is no betaflight Python target implemented here

## simulator boundary

the installed VQ1 and VQ2 build 3391 executables were checked against the existing Ghidra analysis

| binary | sha256 |
| --- | --- |
| vq1 | `d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9` |
| vq2 | `68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b` |

VQ2 retains the body rate attitude local NED and actuator input handlers
its attitude local position and odometry output workers are stubs in both R1 and R2
the vehicle adapter exposes the three existing position velocity and body rate command paths
it converts received telemetry without adding a pose estimator or new telemetry requests

the broader attitude acceleration and direct actuator input paths are not part of this seed
their presence in the parser is not a tested control contract
reported motor output channels are retained with the wire active mask and values without claiming RPM units

the bundled [specification](VQ1-Technical-Specification-00.02.pdf) defines the NED and body frames
the [vendor controller](reference/PyAIPilotExample-v4/controller.py) defines the build 3390 radian extension
the adapter preserves that bit and the existing NED masks
standard message fields are defined by [MAVLink](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)

Live VQ1 build-3391 pulses exposed angular sign differences on the wire.
The adapter negates all three transmitted body rates and received gyro components.
It preserves reported roll and negates reported pitch and yaw for `State.attitude`.
Position, velocity, and reported acceleration keep their wire signs.
Raw `SimulatorClient.telemetry` retains the original messages.
The [measurements and captured regressions](body-rates.md) establish this conversion
against attitude changes, NED motion, and the camera heading; VQ2 has not received
the same physical-response validation.

`r1_gates` consumes `State.motion.position` and returns `PositionNed`
`r1_body_rates` consumes VQ1 motion and attitude and returns `BodyRates`
`zero` returns `BodyRates`
`AIGPSimulator` owns the connection and calls `client.read`, `controller.update`, then `client.send`
controllers implement `BaseController.update(state, gate_index, gates)`
the `BaseController` type parameter declares one output plane or an explicit union
the target validates each actual returned command independently of that annotation
native `RaceStatus` packets stay inside the AI-GP simulator and client
optional observations older than the simulator timeout are `None` at controller update
the client and generic vehicle API retain the original timestamped observations

## track input

`SimulatorClient.gates` exposes the last complete usable track as immutable gate values
`DATA_TRANSMISSION_HANDSHAKE` announces its byte count and chunks
`ENCAPSULATED_DATA` type 2 supplies those chunks, grouped by transfer ID
each gate retains its published NED base as `origin` and normalized wxyz `orientation`
the adapter derives `center` from that origin using orientation and half-height
reported `width` and `height` are overall bounds, not opening clearance
the whole track is cached between transfers; it is not a fresh per-cycle observation
an incomplete replacement keeps the last complete track available
`AIGPSimulator` passes the gates separately from `State` to the controller
missing or withheld geometry stays `None`; no course coordinates are synthesized

The captured VQ1 build 3391 packet is preserved in `test/fixtures/vq1_track.json`.
Its decoded centers match the previously flight-tested R1 coordinates.
This validates that build's geometry; it does not establish VQ2 geometry availability.
