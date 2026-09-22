#!/bin/zsh
set -eu

readonly SCRIPT_DIR="${0:A:h}"
readonly SIM_DIR="$SCRIPT_DIR/.runtime/vq2"
readonly SHIPPING="$SIM_DIR/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"

MODE=r1
if [[ "${1:-}" == "--mode" ]]; then
  if (( $# < 2 )); then
    print -ru2 -- "--mode requires r1 or r2"
    exit 2
  fi
  MODE="$2"
  shift 2
fi

case "$MODE" in
  r1) readonly ARENA_URL="/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase" ;;
  r2) readonly ARENA_URL="/Game/levelsMaster/MAP_arsenal_master?game=/Script/DCGame.GameModeRaceBase" ;;
  *)
    print -u2 "unknown VQ2 mode: $MODE (expected r1 or r2)"
    exit 2
    ;;
esac

source "$SCRIPT_DIR/wine.sh"
source "$SCRIPT_DIR/python.sh"

check_python
check_wine
prepare_python "$SCRIPT_DIR"
"$SCRIPT_DIR/.runtime/client-venv/bin/python" "$SCRIPT_DIR/extract_vq2.py"

export MINIFLIGHT_VQ2_MODE="$MODE"
export WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER=1
export WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer="

run_wine "$SIM_DIR" "$SCRIPT_DIR/.runtime/vq2-wine" "$SHIPPING" "$ARENA_URL" \
  -windowed -ResX=1280 -ResY=720 -nosound -NoSplash "$@"
