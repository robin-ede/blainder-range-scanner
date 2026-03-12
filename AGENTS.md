# AGENTS Guide
## Scope
- This repository is a Blender 5.0+ extension for simulating LiDAR, sonar, and ToF scans.
- Primary source lives in `range_scanner/`; workflow and validation entrypoints live in `scripts/`.
- The repo also contains a real pytest suite under `tests/` for validation/reporting code that can run without Blender.

## Higher-Priority Local Rules
- No `.cursorrules` file was found.
- No `.cursor/rules/` directory was found.
- No `.github/copilot-instructions.md` file was found.
- If any of those files are added later, treat them as higher priority than this document.

## Repository Map
- `range_scanner/__init__.py` wires Blender extension registration.
- `range_scanner/ui/user_interface.py` contains most Blender UI, operators, and config loading.
- `range_scanner/scanners/generic.py` is the shared scan/export/validation pipeline.
- `range_scanner/scanners/lidar.py` and `range_scanner/scanners/sonar.py` hold sensor-specific logic.
- `range_scanner/validation/` contains alignment, degradation, fusion, and evaluation utilities.
- `scripts/run_fixed_scene_validation_batch.py` is the main fixed-scene validation runner.
- `scripts/run_fixed_scene_validation_trial.py` runs one trial inside Blender.
- `scripts/visualize_validation_report.py` renders PNG summaries from report JSON.
- `tests/conftest.py` stubs Blender modules so many validation modules can be tested under plain Python.

## Environment Expectations
- Target runtime is Blender 5.0+ using the manifest-based extension layout in `range_scanner/blender_manifest.toml`.
- Many production modules import Blender-only APIs such as `bpy`, `bmesh`, and `mathutils`.
- Use plain `python3` or `pytest` only for modules/scripts that do not require a live Blender runtime at import time.
- Use Blender-launched Python for scripts that import `bpy` directly, such as scene creation or trial execution scripts.
- Common macOS Blender binary path in repo docs: `/Applications/Blender.app/Contents/MacOS/Blender`.
- Common macOS Blender Python path in repo docs: `/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11`.

## Install And Build Commands
- Install extension dependencies into Blender Python:
```bash
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 -m pip install -r range_scanner/requirements.txt
```
- Install extra validation/plotting deps often used locally:
```bash
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 -m pip install numpy scipy matplotlib
```
- Build the Blender extension zip:
```bash
blender --command extension build --source-dir range_scanner --output-dir .
```
- Build the headless Docker image:
```bash
docker build --platform linux/amd64 -t blainder-range-scanner:blender-5.0.1 .
```

## Linting / Static Checks
- There is no committed Ruff, Black, Flake8, isort, mypy, tox, or nox config at repo root.
- `pytest.ini` sets `pythonpath = .` and `testpaths = tests`.
- `.gitignore` includes `.ruff_cache/`, but Ruff is not configured as a canonical repo tool.
- Prefer targeted compile checks for Python-only files:
```bash
python3 -m py_compile scripts/visualize_validation_report.py
python3 -m py_compile range_scanner/validation/evaluation.py
```
- The Docker image also compile-checks key scripts with Blender's Python during `docker build`.

## Test Commands
- Run the whole Python test suite:
```bash
pytest -v
```
- The CI workflow installs deps with `uv` and runs:
```bash
uv venv --python 3.11
uv pip install numpy scipy open3d==0.18.0 matplotlib pytest -r range_scanner/requirements.txt
uv run pytest tests/ -v
```
- Run one test file:
```bash
pytest tests/test_evaluation_metrics.py -v
```
- Run one test class:
```bash
pytest tests/test_fusion.py::TestVoxelFuseHits -v
```
- Run one test function:
```bash
pytest tests/test_visualize_report.py::TestPlotRmseComparison::test_produces_png_from_full_report -q
```
- Current suite size is about 248 collected tests; use `pytest --collect-only -q` to inspect node IDs.

## Blender-Backed Validation Commands
- Canonical fixed-scene validation batch:
```bash
python3 scripts/run_fixed_scene_validation_batch.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --scene example_scenes/sonar_example.blend \
  --config scripts/fixed_scene_validation.example.json
```
- Run one fixed-scene trial inside Blender:
```bash
"/Applications/Blender.app/Contents/MacOS/Blender" path/to/scene.blend \
  --background \
  --python scripts/run_fixed_scene_validation_trial.py -- \
  --config /tmp/one_trial.json
```
- Run animated path validation:
```bash
python3 scripts/run_animated_path_validation.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --base-scene generated_scenes/dual_sensor_scene_42.blend \
  --config scripts/animated_path_validation.example.json
```
- Generate PNGs from report JSON:
```bash
python3 scripts/visualize_validation_report.py path/to/report.json
```

## Closest Equivalent To "Run One Test"
- For Python-only logic, use pytest node IDs; this repo supports real single-test execution.
- For Blender pipeline regressions, the closest single-scenario run is one trial via `scripts/run_fixed_scene_validation_trial.py`.
- When isolating Blender behavior, create a minimal temp JSON with exactly one trial and the narrowest scene/config that still hits the changed code path.

## Code Style Observed In This Repo
- Use 4-space indentation and preserve the touched file's existing style.
- Prefer small procedural helpers and straightforward data-flow over large abstractions.
- Keep edits local; do not refactor unrelated old Blender UI code just to modernize it.
- Existing code mixes newer style with legacy sections; consistency with the surrounding file matters most.
- Comments are sparse; add them only when a block is hard to understand without context.

## Imports
- Keep Blender-only imports in Blender-facing modules.
- Avoid adding top-level `bpy` imports to scripts that should stay runnable under plain Python.
- In test code, rely on `tests/conftest.py` stubs rather than adding Blender guards everywhere.
- Follow local import ordering conventions in older files instead of rewriting the whole block.
- If a new dependency is optional, prefer `try/except ImportError` with graceful fallback when practical.

## Formatting And Data Shapes
- The codebase is lightly typed; most production functions are unannotated.
- Prefer clear names and stable dict shapes over broad typing refactors.
- When writing JSON/report payloads, convert NumPy scalars and Blender values to Python `int`/`float` explicitly.
- Preserve existing schema keys in reports and config JSON unless all downstream consumers are updated too.
- Keep output dictionaries JSON-serializable and deterministic in key meaning.

## Naming Conventions
- New internal Python helpers should use `snake_case`.
- Constants are uppercase, for example `SWAPPABLE_PROPERTIES`.
- Blender operator/UI class names follow Blender conventions such as `WM_OT_LOAD_PRESET`.
- Blender property names and config JSON keys often use existing camelCase names; preserve them exactly.
- Script naming follows existing patterns: `run_*.py`, `generate_*.py`, `*_to_obj.py`, `make_*_scene.py`, `*.example.json`.

## Error Handling
- Fail early on missing scene objects, config fields, and required files.
- Use `ValueError` for invalid argument/config combinations and `RuntimeError` when a workflow cannot proceed.
- Many Blender-facing paths use `print(...)` for operator feedback; match the local pattern unless there is a clear reason to centralize logging.
- In validation/report generation, prefer structured warning entries in JSON plus concise console warnings.
- Preserve graceful-skip behavior for optional analytics or absent report sections.

## Testing Guidance For Agents
- If you change `range_scanner/validation/` or plotting/report utilities, start with targeted pytest files.
- If you change Blender-only execution paths, run the smallest representative Blender-backed workflow you can.
- If you change report schema or validation output fields, also verify `scripts/visualize_validation_report.py` and fixture-based tests still pass.
- `tests/conftest.py` mocks `bpy`, `bmesh`, and `mathutils`; keep new pure logic separable so it remains testable without Blender.

## Working With Generated Artifacts
- Generated `.blend`, report JSON, CSV, PNG, and metadata files are usually not source of truth.
- Keep reusable configs in `scripts/` as `*.example.json`; put ad hoc outputs in ignored/generated locations.
- Do not commit large generated outputs unless the user explicitly asks for them.

## Practical Agent Workflow
- Read the nearest relevant script, test file, and example config before changing workflow behavior.
- Prefer pytest for pure Python validation code and Blender runs for end-to-end scanner behavior.
- After edits, run the smallest command that validates the touched path, then scale up only if needed.
- In final handoff, mention any schema changes, skipped optional paths, new dependencies, and unverified Blender-only code paths.
