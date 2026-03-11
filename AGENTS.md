# AGENTS Guide

## Scope

- This repository is a Blender 5.0+ extension for simulating LiDAR, sonar, and ToF scans.
- Most production code lives in `range_scanner/`.
- Most automation and validation entrypoints live in `scripts/`.
- There is no dedicated Python packaging config (`pyproject.toml`, `setup.py`, `tox.ini`, `pytest.ini`) at repo root.

## Repository-Specific Rules

- No `.cursorrules` file was found.
- No `.cursor/rules/` directory was found.
- No `.github/copilot-instructions.md` file was found.
- If those files are added later, update this document and treat them as higher-priority repo guidance.

## Environment Expectations

- Primary target: Blender 5.0+ using the extension format in `range_scanner/blender_manifest.toml`.
- Many modules import Blender-only APIs such as `bpy`, `bmesh`, and `mathutils`.
- Expect plain `python3` to work for `scripts/*.py` utilities that do not import Blender at module import time.
- Expect Blender-launched Python for anything that imports `bpy` directly.
- Recommended local Blender path on macOS in repo docs: `/Applications/Blender.app/Contents/MacOS/Blender`.

## Important Paths

- Extension code: `range_scanner/`
- UI and Blender registration: `range_scanner/ui/user_interface.py`
- Scanner pipeline: `range_scanner/scanners/generic.py`
- Validation logic: `range_scanner/validation/`
- Headless workflow scripts: `scripts/`
- Example scene: `example_scenes/sonar_example.blend`
- Generated scenes and outputs are usually ignored by git.

## Install / Build Commands

- Install Blender-side Python dependencies on macOS:
```bash
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 -m pip install PyYAML laspy h5py pypng pascal-voc-writer numpy
```
- Install repo-listed extension dependencies from `range_scanner/requirements.txt` into Blender Python if needed:
```bash
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 -m pip install -r range_scanner/requirements.txt
```
- Build the Blender extension zip from source:
```bash
blender --command extension build --source-dir range_scanner --output-dir .
```
- Compile-check a regular Python script:
```bash
python3 -m py_compile scripts/visualize_validation_report.py
```
- Compile-check a module that does not require importing Blender-only symbols at compile time:
```bash
python3 -m py_compile range_scanner/validation/evaluation.py
```

## Test / Validation Commands

- There is no formal `pytest` or `unittest` suite in this repository today.
- The practical test strategy is running Blender-backed validation workflows plus targeted `py_compile` checks.
- Run one fixed-scene batch validation:
```bash
python3 scripts/run_fixed_scene_validation_batch.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --scene example_scenes/sonar_example.blend \
  --config scripts/fixed_scene_validation.example.json
```
- Run one animated path validation batch:
```bash
python3 scripts/run_animated_path_validation.py \
  --blender "/Applications/Blender.app/Contents/MacOS/Blender" \
  --base-scene generated_scenes/dual_sensor_scene_42.blend \
  --config scripts/animated_path_validation.example.json
```
- Run a single trial inside Blender using a prepared trial JSON:
```bash
"/Applications/Blender.app/Contents/MacOS/Blender" path/to/scene.blend \
  --background \
  --python scripts/run_fixed_scene_validation_trial.py -- \
  --config /tmp/one_trial.json
```
- Generate visualization PNGs from report JSON files:
```bash
python3 scripts/visualize_validation_report.py path/to/*_report.json
```

## Closest Equivalent To "Run One Test"

- There is no single-test runner like `pytest tests/foo.py::test_bar`.
- The closest equivalent is a one-trial validation run with `scripts/run_fixed_scene_validation_trial.py`.
- To isolate one scenario, create a temporary JSON containing exactly one `trial_name`, `primary`, `secondary`, `common`, and optional pose/water-profile data.
- For quick regressions, prefer the smallest `.blend` scene and the narrowest config that still exercises the changed code path.

## Linting / Formatting Reality

- No repo-level Black, Ruff, Flake8, isort, or mypy config was found.
- `.gitignore` includes `.ruff_cache/`, but no Ruff config is committed.
- Do not assume an autoformatter is canonical unless the user asks for one.
- Match the surrounding file style first; consistency with the touched file matters more than external defaults.

## Code Style Observed In This Repo

- Python uses 4-space indentation.
- Functions are usually module-level and organized procedurally.
- Imports are often simple and pragmatic rather than strictly grouped/sorted.
- Existing Blender-facing code mixes `snake_case` and older camelCase names.
- Preserve public/config/property names exactly when they are part of Blender UI state or JSON schemas.
- New internal helper functions should prefer `snake_case`.
- Constants are uppercase, e.g. `SWAPPABLE_PROPERTIES`.
- Short private helpers often use a leading underscore.
- Existing code generally avoids heavy abstraction unless reuse is obvious.

## Imports

- Keep Blender-only imports (`bpy`, `bmesh`, `mathutils`) in modules that are intended to run inside Blender.
- For command-line scripts that must run outside Blender, avoid importing Blender modules at top level unless the script is explicitly Blender-launched.
- Follow local file conventions when editing older modules instead of reordering every import block.
- If you introduce a new optional dependency, guard it with `try/except ImportError` when graceful degradation is reasonable.

## Types And Data Shapes

- The codebase is only lightly typed; most functions are unannotated.
- Prefer clear variable names and shape comments over adding large typing refactors.
- Be explicit when converting NumPy scalars or Blender values to JSON-safe Python scalars using `float()` or `int()`.
- Validation/report code should emit JSON-serializable dictionaries with stable keys.
- Do not rename existing report keys without updating downstream scripts and summaries.

## Naming Conventions

- Blender operator and UI class names follow Blender conventions such as `WM_OT_LOAD_PRESET`.
- Config JSON keys mirror Blender property names and often use camelCase, e.g. `scannerObject`, `dataFilePath`, `noiseRelativeOffset`.
- Script filenames follow role-based patterns already documented in `scripts/README.md`:
  - `run_*.py` for workflows
  - `generate_*.py` for scene/config generation
  - `*_to_obj.py` for conversions
  - `make_*_scene.py` for scene prep

## Error Handling

- Fail early on missing required inputs, especially scene objects, config fields, and output files.
- Raise `RuntimeError` or `ValueError` with concrete context when a workflow cannot proceed.
- For optional analytics or visualization features, graceful skip behavior is preferred over crashing.
- When writing reports, prefer structured warning entries in the JSON plus concise console output.
- Preserve existing behavior where warnings are accumulated and execution continues when safe.

## Blender-Specific Guidance

- Scene mutations can be destructive; batch runners intentionally launch fresh Blender processes per trial.
- Do not assume object names are universal across scenes; inspect scene contents when wiring configs.
- Keep extension compatibility with Blender 5.0+ and the manifest-based extension layout.
- Avoid moving Blender registration logic out of `range_scanner/__init__.py` and `range_scanner/ui/user_interface.py` without a strong reason.

## Validation And Reporting Guidance

- Changes in `range_scanner/validation/` usually need an end-to-end report regeneration check.
- If a change affects output schema, also verify `scripts/run_fixed_scene_validation_batch.py` and `scripts/visualize_validation_report.py` still parse reports.
- Older reports may miss newer keys; visualization and summary tools should skip absent data cleanly.
- Keep output size in mind when adding stored samples or per-point data to JSON.

## Working With Generated Artifacts

- Generated reports, CSVs, PNGs, `.blend` files, and metadata are typically ignored.
- Do not commit generated outputs unless the user explicitly requests it.
- Canonical reusable configs in `scripts/` should remain as `*.example.json` templates.
- Prefer putting ad hoc outputs under scene-relative `output/` folders or ignored generated directories.

## Recommended Agent Workflow

- Read the nearest relevant script and one example config before changing workflow behavior.
- For Blender pipeline changes, run the smallest representative validation scenario you can.
- For pure Python utility changes, run `python3 -m py_compile` first, then a real script invocation.
- Summarize any schema changes, skipped data cases, and new dependencies in your final response.
