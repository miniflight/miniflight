check_python() {
  if ! command -v uv >/dev/null 2>&1; then
    print -u2 "Install uv first: brew install uv"
    return 1
  fi
}

prepare_python() {
  local base="$1"
  local repo="${base:h:h}"
  local legacy="$repo/target/aigp/.runtime"
  if [[ -d "$legacy" && ! -e "$base/.runtime" && ! -L "$base/.runtime" ]]; then
    ln -s "$legacy" "$base/.runtime"
  fi
  local venv="$base/.runtime/client-venv"
  local interpreter="$venv/bin/python"

  export UV_CACHE_DIR="$base/.runtime/uv-cache"
  export UV_PYTHON_INSTALL_DIR="$base/.runtime/uv-python"
  if [[ -e "$venv" || -L "$venv" ]] &&
     ! "$interpreter" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 11))' 2>/dev/null; then
    local backup="$(mktemp -d "$base/.runtime/client-venv-backup-XXXXXX")"
    mv "$venv" "$backup/client-venv"
    print "Preserved previous Python environment at $backup/client-venv"
  fi
  if [[ ! -x "$interpreter" ]]; then
    uv venv --python 3.11 "$venv"
  fi
  uv pip install --python "$interpreter" --editable "${repo}[aigp]"
}
