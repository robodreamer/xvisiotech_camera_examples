#!/usr/bin/env python3
"""Command-line interface for xvisio package setup.

Note: XVSDK installation is system-level (udev rules + `.deb`), so this must be
run with sudo.
"""

from __future__ import annotations

import argparse
import os
import shutil
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


def _find_examples_dir() -> Path | None:
    """Find the examples directory in the installed package."""
    # Examples are installed via CMake to share/xvisio/examples
    # With scikit-build-core, this is relative to the package directory
    try:
        import xvisio as _xvisio_mod

        # Get the package directory
        xvisio_pkg_dir = Path(_xvisio_mod.__file__).parent
        site_packages_dir = xvisio_pkg_dir.parent

        # Check multiple possible locations (in order of likelihood)
        candidates = [
            # CMake install location: share/xvisio/examples relative to package dir
            # (scikit-build-core installs CMake files relative to wheel.install-dir)
            xvisio_pkg_dir / "share" / "xvisio" / "examples",
            # At site-packages root (if installed via MANIFEST.in)
            site_packages_dir / "examples",
            # Relative to package parent (for editable installs from source)
            xvisio_pkg_dir.parent.parent / "examples",
            # System share location (traditional CMake install)
            site_packages_dir.parent.parent / "share" / "xvisio" / "examples",
            # From repo root candidates (source checkout / editable)
            *_repo_root_candidates(),
        ]

        for candidate in candidates:
            if isinstance(candidate, Path) and candidate.exists() and candidate.is_dir():
                # Verify it has example files
                if any(candidate.glob("*.py")):
                    return candidate.resolve()
    except Exception:
        pass

    # Fallback: check all sys.path entries
    for site_packages in sys.path:
        try:
            site_path = Path(site_packages)
            if not site_path.exists():
                continue

            # Check various locations
            candidates = [
                site_path / "xvisio" / "share" / "xvisio" / "examples",
                site_path / "xvisio" / "examples",
                site_path / "examples",
                site_path.parent.parent / "share" / "xvisio" / "examples",
            ]

            for candidate in candidates:
                if candidate.exists() and candidate.is_dir():
                    if any(candidate.glob("*.py")):
                        return candidate.resolve()
        except Exception:
            continue

    return None


def examples_cmd(argv: list[str] | None = None) -> int:
    """Entry point for `xvisio-examples`."""
    parser = argparse.ArgumentParser(
        prog="xvisio-examples",
        description=(
            "Locate or copy Xvisio example scripts.\n\n"
            "Examples are included in the pip package and can be run directly.\n"
            "Use --copy to copy them to a local directory for editing."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--copy",
        metavar="DEST",
        nargs="?",
        const="xvisio_examples",  # Default if --copy is used without argument
        type=str,
        help="Copy examples to the specified directory (default: ./xvisio_examples)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available example scripts.",
    )

    args = parser.parse_args(argv)

    examples_dir = _find_examples_dir()

    if examples_dir is None:
        print("ERROR: Could not find examples directory in installed package.", file=sys.stderr)
        print("Examples may not be included in this installation.", file=sys.stderr)
        print("Try: pip install --force-reinstall xvisio", file=sys.stderr)
        print("\nAlternatively, clone the repository:", file=sys.stderr)
        print("  git clone https://github.com/xvisiotech/xvisiotech_camera_examples.git", file=sys.stderr)
        return 1

    if args.list:
        print(f"Examples directory: {examples_dir}")
        print("\nAvailable examples:")
        for py_file in sorted(examples_dir.glob("*.py")):
            if py_file.name != "__init__.py":
                print(f"  - {py_file.name}")
        return 0

    if args.copy:
        # --copy was used (with or without destination argument)
        dest = Path(args.copy).expanduser().resolve()

        try:
            if dest.exists():
                if not dest.is_dir():
                    print(f"ERROR: {dest} exists but is not a directory", file=sys.stderr)
                    return 1
                response = input(f"Directory {dest} already exists. Overwrite? [y/N]: ")
                if response.lower() != "y":
                    print("Cancelled.")
                    return 0
                shutil.rmtree(dest)

            shutil.copytree(examples_dir, dest)
            print(f"✓ Copied examples to {dest}")
            print(f"\nTo run an example:")
            print(f"  cd {dest}")
            print(f"  python demo_pose_imu.py")
            return 0

        except Exception as e:
            print(f"ERROR: Failed to copy examples: {e}", file=sys.stderr)
            return 1

    # Default: just show location
    print(f"Examples directory: {examples_dir}")
    print("\nTo run an example:")
    print(f"  python {examples_dir}/demo_pose_imu.py")
    print("\nTo copy examples to a local directory:")
    print("  xvisio-examples --copy")
    return 0


if __name__ == "__main__":
    setup_host()
