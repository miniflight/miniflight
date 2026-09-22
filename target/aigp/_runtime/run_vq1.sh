#!/bin/zsh
set -eu

readonly SCRIPT_DIR="${0:A:h:h}"
readonly SIM_DIR="$SCRIPT_DIR/.runtime/vq1"
readonly SHIPPING="$SIM_DIR/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"
readonly ARENA_URL="/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase"
source "$SCRIPT_DIR/_runtime/wine.sh"
source "$SCRIPT_DIR/_runtime/python.sh"

check_python
check_wine
prepare_python "$SCRIPT_DIR"
"$SCRIPT_DIR/.runtime/client-venv/bin/python" "$SCRIPT_DIR/_runtime/vq1.py"

export WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER=1
export WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer="

run_wine "$SIM_DIR" "$SCRIPT_DIR/.runtime/vq1-wine" "$SHIPPING" "$ARENA_URL" \
  -windowed -ResX=1280 -ResY=720 -nosound -NoSplash "$@"
