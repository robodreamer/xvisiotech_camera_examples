#!/usr/bin/env bash
# One-shot installer for the xvisio Python SDK on Ubuntu/Debian.
#
# Usage (from a clone):
#   bash scripts/install_xvisio_linux.sh
#   bash scripts/install_xvisio_linux.sh --python python3.12 --venv ./.venv
#   bash scripts/install_xvisio_linux.sh --editable
#
# Usage (without cloning — installs xvisio from PyPI):
#   curl -fsSL -O <GIST_URL>/install_xvisio_linux.sh
#   bash install_xvisio_linux.sh
#
# What this script does:
#   1. Installs system packages (udev rules + XVSDK driver) using setup_host.sh
#   2. Creates a Python virtual environment
#   3. Installs xvisio from PyPI (or editable from a local clone)
#
# Flags:
#   --python PATH      Python interpreter to use (default: python3)
#   --venv   PATH      Virtual environment path (default: ./.venv)
#   --editable         Editable install from repo (default: pip install xvisio)
#   --skip-host-setup  Skip sudo setup_host.sh (udev rules + XVSDK driver)
#   --help             Show this help message

set -euo pipefail

PYTHON="${PYTHON:-python3}"
VENV_DIR="./.venv"
EDITABLE=false
SKIP_HOST_SETUP=false
REPO_URL="https://github.com/robodreamer/xvisiotech_camera_examples.git"
TMP_CLONE=""

# ── Argument parsing ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --python)   PYTHON="$2";           shift 2 ;;
    --venv)     VENV_DIR="$2";         shift 2 ;;
    --editable) EDITABLE=true;         shift   ;;
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

cleanup() {
  if [[ -n "$TMP_CLONE" && -d "$TMP_CLONE" ]]; then
    rm -rf "$TMP_CLONE"
  fi
}
trap cleanup EXIT

# ── Guard: must not run inside an active venv ────────────────────────────────
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  red "ERROR: A virtual environment is currently active ($VIRTUAL_ENV)."
  red "Please deactivate it first:  deactivate"
  red "Then re-run this script."
  exit 1
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
    # Running standalone (downloaded via curl) — clone repo to temp dir first
    echo "No local clone detected. Cloning repo to a temporary directory..."
    TMP_CLONE="$(mktemp -d)"
    git clone --depth=1 "$REPO_URL" "$TMP_CLONE"
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
  if ! py_version_ok "$VENV_PY"; then
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

# ── Step 6: Verify ────────────────────────────────────────────────────────────
step "Verifying installation"
python -c "import xvisio; print('xvisio', xvisio.__version__)"
green "✓ Installation complete"

cat <<'EOF'

Next steps:
  1. If added to plugdev group above, log out and log back in
  2. Connect your XR-50 or DS-80 device via USB
  3. Activate the venv and test:
       source .venv/bin/activate
       python -c "import xvisio; print(xvisio.discover())"
EOF
