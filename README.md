# miniflight

Miniflight is a companion-computer flight stack.

A `FlightProgram` returns one explicit flight command. A target link sends that
command to a simulator or a real vehicle.

The companion stack uses world NED, body FRD, SI units, and `w, x, y, z`
quaternions. Target adapters own all wire-format and frame conversions.

## Replay

The first recorded Miniflight flight passed all six VQ1 gates without a
collision. Replay it without the simulator:

```bash
python3 -m miniflight replay test/data/vq1_six_gates.mflog
```

Replay starts `ThreadGatesProgram` in a fresh process. It feeds the recorded
gate sequence to that process and compares every new command.

The simulator screen, login, network, and physics are not replay dependencies.
VQ1 only produced the checked-in trace. Another simulator can produce the same
target-independent flight records.

## VQ1

The Anduril VQ1 adapter submits every command altitude that VQ1 implements.
`ThreadGatesProgram` sends `PositionNed` without local control or mixing.

```bash
PYTHONPATH=. python3 examples/aigp/r1_center.py
```

## Configurator

The configurator reads a connected Betaflight controller over USB. It is
read-only by default.

```bash
python3 -m config.serve
```

Open `http://127.0.0.1:8002`.

Remove propellers before bench work. Keep the vehicle disarmed.

## Firmware

`target/stm32` contains the separate C firmware experiment. The Python package
does not implement motor mixing, motor physics, or a board HAL.

## Tests

```bash
python3 -m pytest -q test
```
