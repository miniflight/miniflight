# aigp

A simulator arena for Python controllers. `simulator.py` is the controller loop
and command-line entrypoint. `client.py` receives packets and encodes commands;
`native.py` installs and owns the native process. Controller code lives in
`controllers/`.

Use Python 3.11 or newer and install dependencies once from the repository root:

```sh
python -m pip install -e ".[aigp]"
```

Then enter the arena directory. The commands below run from here:

```sh
cd target/aigp
python simulator.py vq1.r1 --controller r1_gates
python simulator.py vq1.r1 --controller r1_body_rates
python simulator.py vq2.r1 --controller zero
python simulator.py vq2.r2 --controller zero
```

Both R1 controllers aim through the same six gates. `r1_gates` returns
`PositionNed` and lets VQ1 control position. `r1_body_rates` uses the same gate
selection, controls position and attitude in Python, and returns `BodyRates`.
VQ1 then controls angular rates and motors. Both continue until native finish.
`zero` sends zero thrust; it does not hover.

The simulator calls `controller.update(state, gate_index, gates)` and sends the
returned command. It owns observations, timing, arming, native start/finish,
and cleanup. The controller owns its target choices and control calculations.
See [the controller interface](controllers/README.md) to add a controller.
Read [the simulator wiring](docs/wiring.md) for the launch, packet, native control,
and sensor paths.

The same arena can be used from Python:

```python
from target.aigp.simulator import AIGPSimulator
from target.aigp.controllers.r1_body_rates import Controller

sim = AIGPSimulator(Controller(), "vq1.r1")
result = sim.rollout()
```

Create a fresh controller and simulator for each run. Each simulator accepts
one `rollout()` attempt, including attempts that fail or are interrupted.

Runnable examples remain under `examples/aigp/`: `thread_gates` runs the position
baseline and `velocity_ned` sends a short velocity request. The body-rate
counterpart is `controllers/r1_body_rates.py`, run with the command above.

Optional probes, recording, replay, and yaw diagnostics are grouped under
[experiments](experiments/README.md). The simulator and gate controllers do not
import these tools.

## setup

macOS needs [Game Porting Toolkit](https://github.com/Gcenx/homebrew-wine) in `/Applications`.

```sh
brew install git-lfs python@3.11
brew install --cask gcenx/wine/game-porting-toolkit
```

Linux needs an x86_64 desktop with Wine, wineserver, git lfs, and Python 3.11 or newer.
Linux support is experimental and has not been tested on a Linux host.

```sh
git lfs install
git clone https://github.com/miniflight/miniflight.git
cd miniflight
git lfs pull
```

`simulator.py` verifies and extracts the selected archive, launches the AIGP executable
through Wine, runs the controller, and shuts down its own simulator process.
`config/`, `archives/`, and `docs/` contain its assets and references;
`.runtime/` holds generated local data.

Run one simulator at a time. Ctrl+C stops the controller and its owned simulator.
Omit `--controller` to open just the simulator. Use `--attach --controller zero`
with the same Python command to connect to an existing simulator without taking
ownership of its process.
`WINE` and `WINESERVER` select a different Wine installation.
Pass Unreal command-line options after `--`. Options before `--` belong to
this Python runner and must use their full names.

[Vehicle API](docs/vehicle-api.md) · [VQ1 specification](docs/VQ1-Technical-Specification-00.02.pdf) · [VQ2 specification](docs/VQ2-Technical-Specification-00.03.pdf)
