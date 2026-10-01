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

Core checks, including numerical replay of recorded native flight inputs, run
without third-party packages:

```sh
python -S -m unittest test.test_core test.test_position test.test_vehicle
```
