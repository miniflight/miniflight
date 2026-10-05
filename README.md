python api for your vehicle

The numerical core and observation records use only the Python standard library.
Commands live in `miniflight.control` and timestamped observations in
`miniflight.state`. Controller calculations such as
`miniflight.position.position_control` take numerical inputs and return commands
without transport or simulator state. `Vehicle` keeps the public connection API
without importing a target implementation or base class. Connection adapters,
race routines, and simulator lifecycle belong under `target/`.

Install the simulator dependencies with `pip install -e ".[aigp]"`.
Plotting, joystick, and Gym dependencies are available with `pip install -e ".[examples]"`.

[VQ1 / VQ2 simulator setup and running instructions](target/aigp/README.md)

[Write and run a controller](target/aigp/controllers/README.md)

End-to-end regressions run with the simulator dependencies installed:

```sh
python -m test.aigp_udp_smoke
python -m test.aigp_regression r1_gates
python -m test.aigp_regression r1_body_rates
```

The UDP checks use a child simulator fixture to exercise the complete protocol,
runner, gate progression, recording and shutdown path. The native checks launch
VQ1 R1 and require all six gates, native finish, disarm, connection cleanup and
exact replay. Native traces are saved under `target/aigp/.runtime/regressions/`.
