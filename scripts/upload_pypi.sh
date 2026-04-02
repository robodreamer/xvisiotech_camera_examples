#!/bin/bash
# Wrapper script for uploading to PyPI with credentials from ~/.pypirc
# Note: Only source distributions (.tar.gz) are uploaded because:
# - PyPI rejects linux_x86_64 wheels (only manylinux wheels are accepted)
# - Users will build from source, which works on their specific system

set -e

REPO="${1:-pypi}"
VERSION="$(python3 - <<'PY'
import tomllib
from pathlib import Path

data = tomllib.loads(Path("pyproject.toml").read_text())
print(data["project"]["version"])
PY
)"
SDIST="dist/xvisio-${VERSION}.tar.gz"

if [ ! -f "$SDIST" ]; then
    echo "ERROR: Expected source distribution not found: $SDIST"
    echo "Run: pixi run build-dist"
    exit 1
fi

echo "Keeping only current version artifact(s) in dist/: xvisio-${VERSION}*"
find dist -maxdepth 1 -type f ! -name "xvisio-${VERSION}*" -delete

# Use config file if it exists, otherwise use environment variables
if [ -f "$HOME/.pypirc" ]; then
    if [ "$REPO" = "testpypi" ]; then
        echo "Uploading source distribution to TestPyPI..."
        twine upload --repository testpypi --config-file "$HOME/.pypirc" "$SDIST"
    else
        echo "Uploading source distribution to PyPI..."
        twine upload --config-file "$HOME/.pypirc" "$SDIST"
    fi
else
    echo "Warning: ~/.pypirc not found. Using environment variables if set."
    echo "Set TWINE_USERNAME=__token__ and TWINE_PASSWORD=<token> if needed."
    if [ "$REPO" = "testpypi" ]; then
        twine upload --repository testpypi "$SDIST"
    else
        twine upload "$SDIST"
    fi
fi

