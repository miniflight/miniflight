# miniflight

Python control routines and vehicle adapters. The current target is the AI-GP
flight simulator. `r1_beautiful` reads privileged position and attitude, computes
PID feedback, and sends body rates and thrust. VQ1 controls angular rates,
motor mixing, and physics.

The core spine is [`run(receive, update, send, hz=50)`](miniflight/loop.py).
`receive(now)` supplies a tuple of controller inputs, or `None` while waiting.
`update(*inputs)` computes a command; `send(command)` delivers it at the external
system's supported control boundary. Inputs retain their own timestamps.
The adapter owns decoding, calibration, freshness checks, startup and shutdown.
`target/aigp` supplies that edge; the core loop and numerical routines do not import it.
`Vehicle` is the program-facing surface for reading signals and issuing commands.
`Target` is the backend boundary to extend for hardware and later code generation;
compiler lowering is not implemented yet. Live AIGP flights and process replay
use the same `Vehicle` receive and send path.
Programs use `from miniflight import vehicle`, then operations such as
`vehicle.body_rates(...)`. Startup binds one Target; shutdown releases it.

The [velocity program](examples/velocity_ned.py) reads `State`, keeps its own
start time, and returns `VelocityNed` commands. It first climbs at least one metre
above its starting position, then requests northward motion for two seconds.
It finishes after requesting zero velocity and observing a new motion sample
below 0.1 m/s. The maneuver must complete within eight device seconds.
It imports no target code.
Its [AIGP wrapper](examples/aigp/velocity_ned.py) waits for native GO, then passes
`vehicle.state` to the program:

```sh
python3.11 -m target.aigp.simulator vq1.r1 --controller examples.aigp.velocity_ned
```

Add `--record /tmp/velocity.jsonl` to retain the native process, packets, commands,
and result. Replay it with `python3.11 -m target.aigp.experiments.process_replay /tmp/velocity.jsonl`.

Install the [runtime](target/aigp/README.md#setup), then run from the repository root:

```sh
git lfs install
git lfs pull
python3.11 -m pip install -e ".[aigp]"
python3.11 -m target.aigp.simulator vq1.r1 --controller r1_beautiful
```

The runner waits for GO, arms, calls the controller at 50 Hz, and sends its
commands. Native finish or Ctrl+C ends control and closes the simulator it launched.

The controller implements `update(telemetry, frames, race_state)`:

- `telemetry`: new MAVLink packets, with original fields and source timestamps.
- `frames`: new camera images. Either tuple can be empty.
- `race_state`: native GO, validity, gate progress, race time, and gate centers.

The runner calls `vehicle.read()` once per tick. Controllers read `vehicle.state`
and retain only their own control history, such as integrals. The target decodes
observations and keeps their separate sample clocks. Return a command, or `None`
while waiting for required startup input.
`BodyRates` uses forward/right/down axes, rad/s, and collective thrust in [0, 1].
Position and velocity commands use north/east/down axes, metres, and m/s.

By default, `r1_beautiful` aims beyond the next gate. A supplied `reference(t)`
returns desired position, velocity, and acceleration for PID to follow.
The current goal is six valid R1 gates in under 10 native seconds, before MPC.

[Setup and targets](target/aigp/README.md) ·
[Write a controller](target/aigp/controllers/README.md) ·
[Commands](target/aigp/docs/vehicle-api.md) ·
[Experiments](target/aigp/experiments/README.md)
