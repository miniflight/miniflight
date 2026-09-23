# vehicle api

`Vehicle` is a connection to a vehicle, not a controller or a simulator runner

```python
from miniflight import Vehicle
from target.aigp import SimulatorClient

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

| command | units | loop closed by the target |
| --- | --- | --- |
| `PositionNed` | metres in local NED | position and lower loops |
| `VelocityNed` | metres per second in local NED | velocity and lower loops |
| `BodyRates` | radians per second in FRD and collective thrust 0 to 1 | angular rate and motor mixing |

the corresponding convenience methods are `position_ned` `velocity_ned` and `body_rates`
they each send one command and do not run background loops
the aigp harness owns race timing heartbeats command cadence and process lifetime

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

`r1_gates` consumes `State.motion.position` and returns `PositionNed`
`zero` returns `BodyRates`
both use `Vehicle.read` and `Vehicle.send` through the same race controlled loop
