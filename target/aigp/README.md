# aigp

Run AI-GP with one Python controller.

```sh
python target/aigp/simulator.py vq1.r1 --controller r1_gates
python target/aigp/simulator.py vq2.r1 --controller zero
python target/aigp/simulator.py vq2.r2 --controller zero
```

The implementation is [simulator.py](simulator.py). Use Python 3.11 or newer and install
its dependencies once from the repository root:

```sh
python -m pip install -e ".[aigp]"
```

The same simulator can be used from Python:

```python
from target.aigp.simulator import AIGPSimulator
from target.aigp.controllers.r1_gates import Controller

sim = AIGPSimulator(Controller(), "vq1.r1")
result = sim.rollout()
```

The simulator calls `controller.update(state, gate_index, gates)` and sends its returned
command. It owns the connection, timing, arming, native start/finish, and cleanup.
`r1_gates` uses position commands. `zero` sends zero thrust; it does not hover.
[Write a controller](controllers/README.md).

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

[Vehicle API](docs/vehicle-api.md) · [Specification](docs/VQ1-Technical-Specification-00.02.pdf)
