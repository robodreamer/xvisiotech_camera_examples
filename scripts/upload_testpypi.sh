#!/bin/bash
# Wrapper script for uploading to TestPyPI with credentials from ~/.pypirc
# Note: TestPyPI only accepts source distributions (sdist), not platform-specific wheels

set -e

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

# TestPyPI doesn't accept platform-specific wheels, so upload only source distribution
if [ -f "$HOME/.pypirc" ]; then
    echo "Uploading source distribution to TestPyPI..."
    twine upload --repository testpypi --config-file "$HOME/.pypirc" --verbose "$SDIST" 2>&1 || {
        echo ""
        echo "Note: If you see 'File already exists', the version was already uploaded."
        echo "Bump the version with: pixi run version --bump patch"
        exit 1
    }
else
    echo "Warning: ~/.pypirc not found. Using environment variables if set."
    echo "Set TWINE_USERNAME=__token__ and TWINE_PASSWORD=<token> if needed."
    twine upload --repository testpypi --verbose "$SDIST"
fi

