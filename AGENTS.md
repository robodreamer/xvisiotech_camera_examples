# xvisiotech_camera_examples (Agent Guide)

## Mission

Work primarily on the ROS-free `xvisio` Python SDK with safe, minimal, test-backed changes.

## Quick Start Checklist

- [ ] Confirm target area (Python SDK vs ROS2).
- [ ] Prefer small scoped changes.
- [ ] Add/update tests for behavior changes.
- [ ] Run relevant checks (`pixi run test`, then `pixi run build` when needed).
- [ ] Update docs/changelog for user-visible changes.

## Source of Truth: What to Edit

- `python/xvisio/` - public Python API (main maintained surface).
- `python/xvisio/_highlevel.py` - high-level behavior and compatibility shims.
- `python/xvisio/types.py` - public dataclasses.
- `python_bindings/src/bindings.cpp` - nanobind C++/Python bridge.
- `cpp_core/` - XVSDK-facing low-level wrapper.
- `test/` - pytest tests (mock backend + optional hardware tests).

## Layering Rules

- Put product behavior in `python/xvisio/` when possible.
- Keep `python_bindings/` thin (expose native calls/types, avoid policy/business logic).
- Keep `cpp_core/` focused on device access/marshaling.
- Preserve backward compatibility unless user explicitly asks for breaking change.

## ROS2 Boundary

- ROS2 code is under `xvsdk-ros2/`.
- Do not change ROS2 launch/nodes unless the request explicitly targets ROS2 behavior.

## Validation Harness

Default validation:

1. Run targeted tests for touched behavior (if available).
2. Run `pixi run test`.
3. Run `pixi run build` for build sanity when API/bindings/native code changed.

Hardware validation:

- Use `XVISIO_HARDWARE=1 pixi run test` only with connected hardware.
- Never claim hardware validation unless it was actually executed.

## System Constraints

- XVSDK is a system dependency (`/usr/lib/libxvsdk.so`).
- Host setup (`scripts/setup_host.sh` or `xvisio-setup`) requires `sudo` and mutates host state.
- Do not run privileged setup unless the user explicitly asks.

## Docs and Release Expectations

When public behavior changes:

- Update `README.md` and/or `python/README.md` as needed.
- Add release notes in `CHANGELOG.md` under Python SDK section.

Release workflow skill:

- `.cursor/skills/xvisio-release-workflow/SKILL.md`

Typical release flow:

1. `pixi run test`
2. `pixi run build`
3. `pixi run version --bump <patch|minor|major>`
4. Update `CHANGELOG.md`
5. `pixi run build-dist`
6. `pixi run upload-pypi` (unless user asks to skip publish)

## Safety Guardrails

- Avoid destructive git commands unless explicitly requested.
- Do not revert unrelated dirty files.
- Avoid adding dependencies unless clearly necessary.
- Report exactly what was validated and what was not.
