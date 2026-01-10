# Publishing Guide

This guide explains how to build and publish xvisio wheels to PyPI.

## Prerequisites

1. **PyPI Account**: Create accounts on [PyPI](https://pypi.org/account/register/) and [TestPyPI](https://test.pypi.org/account/register/)

2. **API Tokens**: Generate API tokens:
   - PyPI: https://pypi.org/manage/account/token/
   - TestPyPI: https://test.pypi.org/manage/account/token/

3. **Configure Credentials**: Create `~/.pypirc`:
   ```ini
   [distutils]
   index-servers =
       pypi
       testpypi

   [pypi]
   username = __token__
   password = pypi-<your-token-here>

   [testpypi]
   repository = https://test.pypi.org/legacy/
   username = __token__
   password = pypi-<your-testpypi-token-here>
   ```

## Version Management

### Check Current Version
```bash
pixi run version
```

### Set Specific Version
```bash
pixi run version 0.2.0
```

### Bump Version
```bash
# Bump patch version (0.1.0 -> 0.1.1)
pixi run version --bump patch

# Bump minor version (0.1.0 -> 0.2.0)
pixi run version --bump minor

# Bump major version (0.1.0 -> 1.0.0)
pixi run version --bump major
```

## Building Distributions

### Build Wheel Only
```bash
pixi run build-wheel
```

### Build Source Distribution Only
```bash
pixi run build-sdist
```

### Build Both (Recommended)
```bash
pixi run build-dist
```

Built distributions will be in the `dist/` directory.

## Publishing

### 1. Test on TestPyPI First (Recommended)

**Note:** TestPyPI only accepts source distributions (`.tar.gz`), not platform-specific wheels. The upload script automatically handles this.

```bash
# Build distributions (includes both wheel and sdist)
pixi run build-dist

# Upload to TestPyPI (only uploads source distribution)
pixi run upload-testpypi

# Test installation (will build from source)
# Note: Use --extra-index-url to also check PyPI for build dependencies
pip install --index-url https://pypi.org/simple/ --extra-index-url https://test.pypi.org/simple/ xvisio
```

### 2. Publish to PyPI

```bash
# Build distributions
pixi run build-dist

# Upload to PyPI
pixi run upload-pypi
```

### 3. Verify Installation

```bash
# Create a clean virtual environment
python -m venv test_env
source test_env/bin/activate  # or `test_env\Scripts\activate` on Windows

# Install from PyPI
pip install xvisio

# Test import
python -c "import xvisio; print(xvisio.__version__)"
```

## Complete Release Workflow

```bash
# 1. Update version
pixi run version --bump minor  # or patch/major

# 2. Build distributions
pixi run build-dist

# 3. Test on TestPyPI
pixi run upload-testpypi
pip install --index-url https://pypi.org/simple/ --extra-index-url https://test.pypi.org/simple/ xvisio

# 4. If tests pass, publish to PyPI
pixi run upload-pypi

# 5. Create git tag
git tag v$(pixi run version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
git push origin --tags
```

## Troubleshooting

### "Package already exists" Error

If you try to upload the same version twice, PyPI will reject it. Either:
- Bump the version: `pixi run version --bump patch`
- Or delete the old release on PyPI (if it was a test release)

### Authentication Errors

Make sure your `~/.pypirc` is configured correctly with valid API tokens.

### Build Dependencies Not Found When Installing from TestPyPI

If you see errors like `ERROR: Could not find a version that satisfies the requirement scikit-build-core`, this is because TestPyPI doesn't contain build dependencies. Use `--extra-index-url` to also check PyPI:

```bash
pip install --index-url https://pypi.org/simple/ --extra-index-url https://test.pypi.org/simple/ xvisio==0.1.1
```

This tells pip to look in PyPI first (for build dependencies like `scikit-build-core` and `nanobind`), then fall back to TestPyPI for the `xvisio` package itself.

### Build Errors

If build fails, check:
- System dependencies are installed: `sudo ./scripts/setup_host.sh`
- CMake and build tools are available in pixi environment
- All source files are present

## Notes

- **System Setup Still Required**: Even after publishing to PyPI, users still need to run `sudo xvisio-setup` for system-level setup (udev rules, driver installation)
- **Platform-Specific**: Wheels are Linux-specific due to dependency on `/usr/lib/libxvsdk.so`
- **TestPyPI Limitation**: TestPyPI only accepts source distributions (`.tar.gz`), not platform-specific wheels. The upload script automatically uploads only the source distribution to TestPyPI.
- **Version Format**: Follows semantic versioning (MAJOR.MINOR.PATCH)

