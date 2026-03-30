---
name: xvisio-release-workflow
description: Executes xvisio push and release workflows with Pixi commands, version bumping, changelog updates, and publish safeguards. Use when the user asks to push code, cut a release, bump versions, update CHANGELOG.md, run build-dist, or upload to PyPI in this repo.
---

# xvisio Release Workflow

Use this skill for repository-specific push and release operations in `xvisiotech_camera_examples`.

## Triggers

Apply this skill when the user asks for:
- push workflow steps after code changes
- patch/minor/major release steps
- version bump + changelog + publish flow
- release checks for PyPI/TestPyPI

## Push Workflow (Any Code Change)

1. Validate with Pixi:
   - `pixi run test`
   - Use focused tests only when changes are tightly scoped.
2. Build sanity check:
   - `pixi run build`
3. Bump version:
   - Determine bump type:
     - **patch** (`0.x.Y`): bug fixes, docs, refactors without new API.
     - **minor** (`0.X.0`): new public API/features/examples.
     - **major** (`X.0.0`): breaking API changes.
   - Run `pixi run version --bump <patch|minor|major>` (updates `pyproject.toml`).
4. Update `CHANGELOG.md`:
   - Add new section under `## Python SDK (pip package: xvisio)` using:
     - `### [<version>] - <YYYY-MM-DD>`
   - Summarize under `#### Added`, `#### Changed`, `#### Fixed` as applicable.
5. Stage and commit:
   - `git add <files>`
   - `git commit -m "<message>"`
6. Push branch:
   - `git push`
7. Publish to PyPI (unless user explicitly says "push only" or "do not publish"):
   - `pixi run build-dist`
     - This task clears old `dist/` artifacts before building.
   - `pixi run upload-pypi`
     - Upload scripts publish only `dist/xvisio-<current-version>.tar.gz`.
   - Verify uploaded version:
     - `curl -sSf https://pypi.org/pypi/xvisio/json | python -c "import json,sys; print(json.load(sys.stdin)['info']['version'])"`
     - Confirm it matches `pyproject.toml`.

## Optional TestPyPI Validation (Recommended for risky releases)

Run before PyPI publish when user asks for extra validation:
- `pixi run build-dist`
- `pixi run upload-testpypi`
- Smoke install:
  - `pip install --index-url https://pypi.org/simple/ --extra-index-url https://test.pypi.org/simple/ xvisio==<version>`

## Release Guardrails

- Prefer small, focused commits.
- Always use Pixi tasks from this repo (`pixi run ...`) to keep environment/tooling consistent.
- Do not publish if tests or build fail.
- PyPI rejects duplicate versions; bump again before re-upload if needed.
- Follow repository git safety (no destructive git commands unless explicitly requested).

## Output Format

When executing this workflow, report:
1. Commands run
2. Files changed
3. Validation results (`test`, `build`, `build-dist`)
4. Publish status (`upload-testpypi` / `upload-pypi`, success/skip, and verified live version)
