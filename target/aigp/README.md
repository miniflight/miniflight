# aigp

run the simulator and a python controller with one command

## setup

macos needs [game porting toolkit](https://github.com/Gcenx/homebrew-wine) in `/Applications`

```sh
brew install git-lfs uv
brew install --cask gcenx/wine/game-porting-toolkit
```

linux needs an x86_64 desktop with [wine](https://gitlab.winehq.org/wine/wine/-/wikis/Download),
`wineserver`, git lfs and [uv](https://docs.astral.sh/uv/getting-started/installation/)
linux support is experimental and has not been tested on a linux host

```sh
git lfs install
git clone https://github.com/miniflight/miniflight.git
cd miniflight
git lfs pull
```

## run

```sh
./target/aigp/run vq1.r1 --controller r1_gates
./target/aigp/run vq2.r1 --controller zero
./target/aigp/run vq2.r2 --controller zero
```

run one at a time and use ctrl+c to stop both processes
omit `--controller` to run just the simulator
the first run prepares python and extracts the selected archive

`r1_gates` flies the six vq1 gates using position control
`zero` sends zero thrust and does not hover
add your controller to `controllers/` and select it by name

the runner finds wine automatically
set `WINE` and `WINESERVER` to use a different installation

## code

`client.py` handles udp and `runner.py` owns the simulator and controller lifecycle
`install.py` verifies and extracts the archives
`config/` holds the existing simulator configuration and `docs/` holds the references
`.runtime/` is generated local data and is not committed

[controllers](controllers/README.md) · [specification](docs/VQ1-Technical-Specification-00.02.pdf) · [python example](docs/reference/PyAIPilotExample-v4)
