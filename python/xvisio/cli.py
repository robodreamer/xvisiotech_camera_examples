#!/usr/bin/env python3
"""Command-line interface for xvisio package setup.

Note: XVSDK installation is system-level (udev rules + `.deb`), so this must be
run with sudo.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def _repo_root_candidates() -> list[Path]:
    """Candidate roots for locating `scripts/` and `ubuntu-drivers/`.

    Works for:
    - Source checkout
    - Editable install (scikit-build-core editable.mode=inplace)

    For a wheel-only install, these directories may not exist.
    """

    candidates: list[Path] = []

    # This file is at <repo>/python/xvisio/cli.py in this repository.
    candidates.append(Path(__file__).resolve().parents[2])

    try:
        import xvisio as _xvisio_mod

        # Typically:
        # - editable: <repo>/python/xvisio/__init__.py
        # - wheel:    <site-packages>/xvisio/__init__.py
        candidates.append(Path(_xvisio_mod.__file__).resolve().parents[2])
    except Exception:
        pass

    # De-dupe while preserving order
    uniq: list[Path] = []
    for p in candidates:
        if p not in uniq:
            uniq.append(p)
    return uniq


def _find_repo_path(relative_path: str) -> Path | None:
    for root in _repo_root_candidates():
        p = root / relative_path
        if p.exists():
            return p
    return None


def get_setup_script() -> Path | None:
    """Locate `scripts/setup_host.sh` (source checkout / editable install)."""

    return _find_repo_path("scripts/setup_host.sh")


def get_drivers_dir() -> Path | None:
    """Locate `ubuntu-drivers/` (source checkout / editable install)."""

    return _find_repo_path("ubuntu-drivers")


def setup_host() -> None:
    """Run the host setup script to install system dependencies."""

    print("=== Xvisio Host Setup ===")
    print("")

    if os.geteuid() != 0:
        print("ERROR: This command requires sudo privileges.")
        print("Please run: sudo xvisio-setup")
        sys.exit(1)

    script = get_setup_script()
    drivers_dir = get_drivers_dir()

    if script is None or drivers_dir is None:
        print("ERROR: Could not locate driver install assets in this environment.")
        print("This command currently expects a source checkout or editable install")
        print("that contains `scripts/` and `ubuntu-drivers/`.")
        print("")
        print("Fix:")
        print("  sudo ./scripts/setup_host.sh")
        sys.exit(1)

    print(f"Found setup script: {script}")
    print(f"Found drivers dir:  {drivers_dir}")
    print("")

    env = os.environ.copy()
    env["XVISIO_DRIVERS_DIR"] = str(drivers_dir)

    print("Running host setup...")
    try:
        subprocess.run(["bash", str(script)], env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"ERROR: setup_host.sh failed with exit code {e.returncode}")
        raise

    print("")
    print("=== Setup Complete ===")
    print("Next steps:")
    print("1. If you were added to the plugdev group, log out and log back in")
    print("2. Connect your XR-50 device via USB")
    print("3. Test: python3 -c 'import xvisio; print(xvisio.discover())'")


if __name__ == "__main__":
    setup_host()
