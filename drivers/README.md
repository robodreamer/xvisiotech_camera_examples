# XVSDK Drivers

Pre-built XVSDK `.deb` packages for Ubuntu. The `setup_host.sh` script selects the right one automatically via `lsb_release -cs`.

## Available drivers

| File | Ubuntu target | XVSDK version |
|------|--------------|---------------|
| `XVSDK_jammy_amd64_20250227.deb` | 22.04 (Jammy) | 3.x |
| `XVSDK_noble_amd64_20260406.deb` | 24.04 (Noble) | 3.2.0 |

> **Note on `noble` filename:** The original file from the vendor was named `XVSDK_bionic_amd64_20260406.deb`.
> glibc symbol analysis (max required: `GLIBC_2.25`) confirmed it is an Ubuntu 18.04-era build, but the
> vendor confirmed it is supported on Ubuntu 24.04. It was renamed to `noble` so the setup script can
> auto-select it correctly.

## Driver differences (noble vs jammy)

### New in noble

**IR Tracking Camera support** — the most significant addition:
- New `IrTrackingCamera` class with dual IR camera streaming, LED control, exposure time and temperature APIs
- New types: `IrTrackingImage`, `IrTrackingTemperature`
- C interface additions: `xv_start_irtracking()`, `xv_stop_irtracking()`, `xv_get_irtracking_image()` and camera2 equivalents

**New parameter structs:**
- `ExposureParam` (time in µs + gain) — now exposed on color and fisheye cameras via `getExposure()`
- `ResolutionParam`, `RoiParam` — resolution and ROI query on IR tracking camera
- `ClampData` — timestamped clamp/sync data, with `ClampStream` interface

**New fisheye/stereo controls:**
- `enableIrGamma(bool)` and `isEnableIrGamma()` on the stereo camera

**Bundled SuiteSparse** — the noble driver ships its own copies of:
`libamd`, `libblas`, `libcamd`, `libccolamd`, `libcholmod`, `libcolamd`, `libcxsparse`,
`libgfortran`, `liblapack`, `libmetis`, `libspqr`, `libsuitesparseconfig`

This means `libsuitesparse-dev` does **not** need to be installed separately on noble.
`setup_host.sh` detects this automatically.

**New C-interface types:**
- `RGBD_Image`, `RGBPointcloud_Image` structs + callback typedefs
- `libxvisio-CInterface-multi-wrapper.so` (multi-device C wrapper)

### Removed in noble (present in jammy)
- `/usr/bin/multi-devices`, `/usr/bin/multi-devices-c-interface`, `/usr/bin/player`, `/usr/bin/recorder` prebuilt demo binaries

## Python API and example compatibility

The existing Python API (`python/xvisio/`) and all examples are **fully compatible** with both drivers.

The C++ wrapper (`cpp_core/`) only calls four SDK interfaces — `slam()`, `imuSensor()`, `fisheyeCameras()`, `wirelessController()` — none of which changed between jammy and noble. All noble driver additions are purely additive (new `IrTrackingCamera` class, new parameter structs, new methods on existing classes). The wrapper consumes `xv::Device` objects from the SDK; it does not subclass any SDK interfaces, so the new pure-virtual method additions have no impact.

If new noble-only features (e.g. IR tracking camera, exposure control) are to be exposed in Python in the future, they can be added to `cpp_core/` and `python_bindings/` without affecting the existing API surface.

## Adding a driver for a new Ubuntu release

1. Name the file `XVSDK_<codename>_amd64_<date>.deb` where `<codename>` matches `lsb_release -cs` output (e.g. `oracular`, `plucky`).
2. Add it to git via LFS (tracked automatically by `.gitattributes`): `git add drivers/XVSDK_<codename>_amd64_<date>.deb`
3. No changes to `setup_host.sh` are needed — it discovers drivers by codename glob.
4. If the new driver bundles SuiteSparse (check: `dpkg-deb --fsys-tarfile <file> | tar -t | grep libcholmod`), the script will skip the system install automatically.
