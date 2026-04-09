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

# ── Step 2: Python virtual environment ────────────────────────────────────────
step "Python virtual environment"
if [[ ! -d "$VENV_DIR" ]]; then
  "$PYTHON" -m venv "$VENV_DIR"
  green "✓ Created venv at $VENV_DIR"
else
  echo "Venv already exists at $VENV_DIR — reusing."
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install -U pip -q

# ── Step 3: Install xvisio ────────────────────────────────────────────────────
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

# ── Step 4: Verify ────────────────────────────────────────────────────────────
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
