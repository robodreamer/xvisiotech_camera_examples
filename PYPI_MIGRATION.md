# PyPI Migration Guide

This document provides instructions for migrating documentation from TestPyPI to PyPI when the package is ready for production release.

## Quick Migration Steps

1. **Search and replace** in all documentation files:
   - `test.pypi.org` → `pypi.org`
   - `TestPyPI` → `PyPI`
   - Remove `--extra-index-url` flags (no longer needed)
   - Remove version pinning (use `xvisio>=0.1.0` instead of `xvisio==0.1.1`)

2. **Uncomment PyPI sections** marked with `<!-- TODO: Uncomment when publishing to PyPI -->`

3. **Remove or archive TestPyPI sections** marked with `<!-- TODO: When publishing to PyPI, replace/remove -->`

## Files to Update

- `README.md` - Main installation instructions
- `INSTALL.md` - Detailed installation guide
- `PIP_INSTALL.md` - Pip-specific installation guide
- `PUBLISHING.md` - Publishing workflow (update upload commands)

## Search Patterns

### Find TestPyPI references:
```bash
grep -r "test.pypi.org" README.md INSTALL.md PIP_INSTALL.md
grep -r "TestPyPI" README.md INSTALL.md PIP_INSTALL.md
grep -r "TODO.*PyPI" README.md INSTALL.md PIP_INSTALL.md
```

### Example replacements:

**Installation command:**
```bash
# Before (TestPyPI)
pip install -i https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ xvisio==0.1.1

# After (PyPI)
pip install xvisio
```

**Requirements.txt:**
```bash
# Before (TestPyPI)
--extra-index-url https://pypi.org/simple/
--index-url https://test.pypi.org/simple/
xvisio==0.1.1

# After (PyPI)
xvisio>=0.1.0
```

**pyproject.toml:**
```toml
# Before (TestPyPI)
[[tool.pip.index]]
name = "testpypi"
url = "https://test.pypi.org/simple/"
extra = true

[project]
dependencies = ["xvisio==0.1.1"]

# After (PyPI)
[project]
dependencies = ["xvisio>=0.1.0"]
```

## Checklist

- [ ] Update README.md installation section
- [ ] Update INSTALL.md installation section
- [ ] Update PIP_INSTALL.md installation section
- [ ] Update integration examples in PIP_INSTALL.md
- [ ] Update installation methods summary table
- [ ] Remove TestPyPI-specific notes
- [ ] Update PUBLISHING.md with PyPI upload instructions
- [ ] Test installation from PyPI in clean environment
- [ ] Update version number if needed
- [ ] Commit and tag release

