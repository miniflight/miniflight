readonly WINE="/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wine64"
readonly WINESERVER="/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wineserver"

check_wine() {
  if [[ ! -x "$WINE" || ! -x "$WINESERVER" ]]; then
    print -u2 "Install Game Porting Toolkit.app in /Applications first."
    return 1
  fi
}

stop_wine() {
  trap - EXIT INT TERM HUP
  print "\nStopping the simulator..."
  "$WINESERVER" -k >/dev/null 2>&1 || true
}

run_wine() {
  local sim_dir="$1"
  export WINEPREFIX="$2"
  shift 2

  mkdir -p "$WINEPREFIX"
  "$WINESERVER" -k >/dev/null 2>&1 || true

  cd "$sim_dir"
  trap '' INT TERM HUP
  "$WINE" "$@" &

  local sim_pid=$!
  trap stop_wine EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP

  set +e
  wait "$sim_pid"
  local sim_status=$?
  exit "$sim_status"
}
