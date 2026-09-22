#!/bin/zsh
set -eu

readonly SCRIPT_DIR="${0:A:h}"
readonly SIM_DIR="$SCRIPT_DIR/.runtime/vq1"
readonly SHIPPING="$SIM_DIR/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"
readonly ARENA_URL="/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase"
readonly WINE="/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wine64"
readonly WINESERVER="/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wineserver"

if [[ ! -x "$WINE" || ! -x "$WINESERVER" ]]; then
  print -u2 "Install Game Porting Toolkit.app in /Applications first."
  exit 1
fi

"${PYTHON:-python3}" "$SCRIPT_DIR/extract_vq1.py"

export WINEPREFIX="$SCRIPT_DIR/.runtime/vq1-wine"
export WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER=1
export WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer="

mkdir -p "$WINEPREFIX"

stop_vq1() {
  trap - EXIT INT TERM HUP
  print "\nStopping the simulator..."
  "$WINESERVER" -k >/dev/null 2>&1 || true
}

"$WINESERVER" -k >/dev/null 2>&1 || true

cd "$SIM_DIR"
trap '' INT TERM HUP
"$WINE" "$SHIPPING" "$ARENA_URL" \
  -windowed -ResX=1280 -ResY=720 -nosound -NoSplash "$@" &

readonly SIM_PID=$!
trap stop_vq1 EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

set +e
wait "$SIM_PID"
SIM_STATUS=$?
exit "$SIM_STATUS"
