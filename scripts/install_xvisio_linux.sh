#!/usr/bin/env bash
# One-shot installer for the xvisio Python SDK on Ubuntu/Debian.
#
# Usage (from a clone):
#   bash scripts/install_xvisio_linux.sh
#   bash scripts/install_xvisio_linux.sh --python python3.12 --venv ./.venv
#   bash scripts/install_xvisio_linux.sh --editable
#   bash scripts/install_xvisio_linux.sh --teleop
#
# Usage (without cloning — installs xvisio from PyPI):
#   curl -fsSL -O <GIST_URL>/install_xvisio_linux.sh
#   bash install_xvisio_linux.sh
#
# What this script does:
#   1. Installs system packages (udev rules + XVSDK driver) using setup_host.sh
#   2. Creates a Python virtual environment
#   3. Installs xvisio from PyPI (or editable from a local clone)
#   4. Optionally installs EmbodiK solver/example support for teleop
#   When run outside a clone, it downloads only the host setup script and driver
#   assets needed for step 1 instead of cloning the full repository.
#
# Flags:
#   --python PATH      Python interpreter to use (default: python3)
#   --venv   PATH      Virtual environment path (default: ./.venv)
#   --editable         Editable install from repo (default: pip install xvisio)
#   --teleop           Also install EmbodiK solver/example dependencies for teleop
#   --host-assets MODE Host setup asset source when outside a clone: minimal (default) or clone
#   --skip-host-setup  Skip sudo setup_host.sh (udev rules + XVSDK driver)
#   --help             Show this help message

set -euo pipefail

PYTHON="${PYTHON:-python3}"
VENV_DIR="./.venv"
EDITABLE=false
INSTALL_TELEOP=false
SKIP_HOST_SETUP=false
HOST_ASSETS_MODE="minimal"
REPO_URL="https://github.com/robodreamer/xvisiotech_camera_examples.git"
RAW_BASE_URL="https://raw.githubusercontent.com/robodreamer/xvisiotech_camera_examples/main"
TMP_CLONE=""

# ── Argument parsing ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --python)   PYTHON="$2";           shift 2 ;;
    --venv)     VENV_DIR="$2";         shift 2 ;;
    --editable) EDITABLE=true;         shift   ;;
    --teleop)   INSTALL_TELEOP=true;   shift   ;;
    --host-assets)
      HOST_ASSETS_MODE="$2"
      if [[ "$HOST_ASSETS_MODE" != "minimal" && "$HOST_ASSETS_MODE" != "clone" ]]; then
        echo "ERROR: --host-assets must be 'minimal' or 'clone'" >&2
        exit 1
      fi
      shift 2
      ;;
    --skip-host-setup) SKIP_HOST_SETUP=true; shift ;;
    --help|-h)
      grep '^#' "$0" | sed 's/^# \?//'
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

# ── Helpers ───────────────────────────────────────────────────────────────────
green() { printf '\033[32m%s\033[0m\n' "$*"; }
red()   { printf '\033[31m%s\033[0m\n' "$*" >&2; }
step()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

download_file() {
  local url="$1"
  local dest="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL "$url" -o "$dest"
  elif command -v wget >/dev/null 2>&1; then
    wget -q "$url" -O "$dest"
  else
    red "ERROR: Need curl or wget to download host setup assets."
    exit 1
  fi
}

download_host_setup_assets() {
  local dest="$1"
  mkdir -p "${dest}/scripts" "${dest}/drivers"

  echo "Downloading host setup assets from ${RAW_BASE_URL}..."
  download_file "${RAW_BASE_URL}/scripts/setup_host.sh" "${dest}/scripts/setup_host.sh"
  download_file "${RAW_BASE_URL}/drivers/99-xvisio.rules" "${dest}/drivers/99-xvisio.rules"
  download_file "${RAW_BASE_URL}/drivers/XVSDK_jammy_amd64_20250227.deb" \
    "${dest}/drivers/XVSDK_jammy_amd64_20250227.deb"
  chmod +x "${dest}/scripts/setup_host.sh"
}

persist_teleop_runtime_env() {
  local activate_file="${VENV_DIR}/bin/activate"
  local lib_dir="$1"
  local begin_marker="# >>> xvisio teleop runtime env >>>"
  local end_marker="# <<< xvisio teleop runtime env <<<"

  if [[ ! -f "$activate_file" ]]; then
    return
  fi

  sed -i "/${begin_marker}/,/${end_marker}/d" "$activate_file"
  cat >> "$activate_file" <<EOF

${begin_marker}
export LD_LIBRARY_PATH="${lib_dir}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
${end_marker}
EOF
}

cleanup() {
  if [[ -n "$TMP_CLONE" && -d "$TMP_CLONE" ]]; then
    rm -rf "$TMP_CLONE"
  fi
}
trap cleanup EXIT

# ── Guard: avoid accidentally installing into the wrong active venv ──────────
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  ACTIVE_VENV="$(readlink -f "$VIRTUAL_ENV")"
  TARGET_VENV="$(readlink -m "$VENV_DIR")"
  if [[ "$ACTIVE_VENV" != "$TARGET_VENV" ]]; then
    red "ERROR: A different virtual environment is currently active ($VIRTUAL_ENV)."
    red "Target venv for this install is: $VENV_DIR"
    red "Please deactivate it first:  deactivate"
    red "Then re-run this script, or pass --venv \"$VIRTUAL_ENV\" to update the active environment."
    exit 1
  fi
  echo "Using active target virtual environment: $VIRTUAL_ENV"
fi

# ── Detect repo context ───────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd 2>/dev/null || echo "")"

if [[ -f "${REPO_ROOT}/scripts/setup_host.sh" && -d "${REPO_ROOT}/drivers" ]]; then
  IN_REPO=true
else
  IN_REPO=false
fi

# ── Step 1: Host setup (udev + XVSDK driver) ──────────────────────────────────
if [[ "$SKIP_HOST_SETUP" == false ]]; then
  step "Host setup (udev rules + XVSDK driver)"
  echo "This step requires sudo. Running: sudo setup_host.sh"

  if [[ "$IN_REPO" == true ]]; then
    # Running from inside the cloned repo
    sudo bash "${REPO_ROOT}/scripts/setup_host.sh"
  else
    TMP_CLONE="$(mktemp -d)"
    if [[ "$HOST_ASSETS_MODE" == "clone" ]]; then
      # Running standalone, but user requested the full repository checkout.
      echo "No local clone detected. Cloning repo to a temporary directory..."
      git clone --depth=1 "$REPO_URL" "$TMP_CLONE"
    else
      # Running standalone (downloaded via curl) — download only required host assets.
      echo "No local clone detected. Downloading setup_host.sh and driver assets..."
      download_host_setup_assets "$TMP_CLONE"
    fi
    sudo bash "${TMP_CLONE}/scripts/setup_host.sh"
  fi

  green "✓ Host setup complete"
else
  echo "Skipping host setup (--skip-host-setup)."
fi

# ── Step 2: Python version check ─────────────────────────────────────────────
step "Checking Python version"

py_version_ok() {
  local py="$1"
  local major minor
  major=$("$py" -c "import sys; print(sys.version_info.major)" 2>/dev/null) || return 1
  minor=$("$py" -c "import sys; print(sys.version_info.minor)" 2>/dev/null) || return 1
  [[ "$major" -gt 3 || ( "$major" -eq 3 && "$minor" -ge 10 ) ]]
}

if ! command -v "$PYTHON" &>/dev/null; then
  red "ERROR: Python interpreter not found: $PYTHON"
  exit 1
fi

if ! py_version_ok "$PYTHON"; then
  PY_VERSION=$("$PYTHON" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
  echo "Python $PY_VERSION is too old (xvisio requires >= 3.10). Looking for a newer version..."

  FOUND_PY=""
  for candidate in python3.12 python3.11 python3.10; do
    if command -v "$candidate" &>/dev/null && py_version_ok "$candidate"; then
      FOUND_PY="$candidate"
      break
    fi
  done

  if [[ -n "$FOUND_PY" ]]; then
    echo "Found $FOUND_PY — switching to it."
    PYTHON="$FOUND_PY"
    # ensure the venv module is available for this interpreter
    if ! "$PYTHON" -m venv --help &>/dev/null; then
      PY_PKG="${PYTHON##*/}-venv"   # e.g. python3.10-venv
      echo "$PY_PKG not installed — installing..."
      sudo apt-get install -y "$PY_PKG"
    fi
  else
    echo "No suitable Python found. Installing Python 3.10 via deadsnakes PPA..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt-get update -q
    sudo apt-get install -y python3.10 python3.10-venv python3.10-distutils
    PYTHON="python3.10"
  fi
fi

PY_VERSION=$("$PYTHON" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
green "✓ Python $PY_VERSION ($PYTHON)"

# ── Step 3: Python virtual environment ────────────────────────────────────────
step "Python virtual environment"
if [[ -d "$VENV_DIR" ]]; then
  VENV_PY="${VENV_DIR}/bin/python"
  VENV_ACTIVATE="${VENV_DIR}/bin/activate"
  if [[ ! -x "$VENV_PY" || ! -f "$VENV_ACTIVATE" ]]; then
    echo "Existing venv at $VENV_DIR is incomplete — recreating."
    rm -rf "$VENV_DIR"
  elif ! py_version_ok "$VENV_PY"; then
    VENV_VER=$("$VENV_PY" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "unknown")
    echo "Existing venv at $VENV_DIR uses Python $VENV_VER (too old) — recreating with $PYTHON..."
    rm -rf "$VENV_DIR"
  else
    echo "Venv already exists at $VENV_DIR — reusing."
  fi
fi

if [[ ! -d "$VENV_DIR" ]]; then
  "$PYTHON" -m venv "$VENV_DIR"
  green "✓ Created venv at $VENV_DIR"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install -U pip -q

# ── Step 4: Check C++ compiler ───────────────────────────────────────────────
step "Checking C++ compiler"
if ! dpkg -l build-essential 2>/dev/null | grep -q "^ii"; then
  echo "build-essential not found — installing..."
  sudo apt-get install -y build-essential
  green "✓ build-essential installed"
else
  echo "C++ compiler already present."
fi

# ── Step 5: Install xvisio ────────────────────────────────────────────────────
step "Installing xvisio"
if [[ "$EDITABLE" == true ]]; then
  if [[ "$IN_REPO" == false ]]; then
    red "ERROR: --editable requires running from inside the cloned repository."
    exit 1
  fi
  pip install -e "${REPO_ROOT}" --no-build-isolation
  green "✓ Editable install from ${REPO_ROOT}"
else
  pip install xvisio
  green "✓ Installed xvisio from PyPI"
fi

# ── Step 6: Optional EmbodiK teleop support ───────────────────────────────────
if [[ "$INSTALL_TELEOP" == true ]]; then
  step "Installing EmbodiK teleop support"
  echo "Installing EmbodiK system dependencies..."
  sudo apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    libeigen3-dev \
    liburdfdom-dev

  echo "Installing EmbodiK Python build dependencies into ${VENV_DIR}..."
  pip install pin scikit-build-core nanobind cmake ninja

  echo "Configuring clean EmbodiK build/runtime paths from the PyPI pin wheel..."
  unset LD_LIBRARY_PATH DYLD_LIBRARY_PATH CMAKE_PREFIX_PATH pinocchio_DIR || true
  PIN_PREFIX="$(python -c 'import pinocchio, pathlib; print(pathlib.Path(pinocchio.__file__).resolve().parents[4])')"
  CMEEL_LIB_DIR="${PIN_PREFIX}/lib"
  export CMAKE_PREFIX_PATH="${PIN_PREFIX}"
  export LD_LIBRARY_PATH="${CMEEL_LIB_DIR}"
  echo "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
  persist_teleop_runtime_env "$CMEEL_LIB_DIR"

  echo "Installing EmbodiK solver and example dependencies..."
  pip install --no-build-isolation "embodik[examples]"
  python -c "import embodik; print('embodik', embodik.__version__)"
  green "✓ EmbodiK teleop support installed"
fi

# ── Step 7: Verify ────────────────────────────────────────────────────────────
step "Verifying installation"
python -c "import xvisio; print('xvisio', xvisio.__version__)"
green "✓ Installation complete"

if [[ "$INSTALL_TELEOP" == true ]]; then
  cat <<'EOF'

Next steps:
  1. If added to plugdev group above, log out and log back in
  2. Connect your Xvisio tracking camera or Seer controller
  3. Activate the venv:
       source .venv/bin/activate
  4. Copy and run the EmbodiK Panda teleop example:
       embodik-examples --copy
       cd embodik_examples
       python 03_teleop_ik.py --robot panda
     Seer controller only: add --controller-port /dev/ttyUSB0 if needed
EOF
else
  cat <<'EOF'

Next steps:
  1. If added to plugdev group above, log out and log back in
  2. Connect your XR-50 or DS-80 device via USB
  3. Activate the venv and test:
       source .venv/bin/activate
       python -c "import xvisio; print(xvisio.discover())"
EOF
fi
