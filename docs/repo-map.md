# Repository Map

## Top Level

- `README.md`
  - Main entry point and quick start.
- `docs/`
  - Focused project and workflow documentation.
- `scripts/`
  - Headless scene generation, validation, and real-data tooling.
- `range_scanner/`
  - Blender extension source code.
- `example_scenes/`
  - Curated Blender example scenes. The main retained example is `sonar_example.blend`.
- `images/`
  - README and GUI documentation images.

## Core Source

- `range_scanner/ui/user_interface.py`
  - Main GUI and script entry points.
- `range_scanner/scanners/generic.py`
  - Shared scan execution path, export orchestration, and validation hooks.
- `range_scanner/scanners/lidar.py`
  - Lidar-style scanning logic.
- `range_scanner/scanners/sonar.py`
  - Sonar/side-scan logic.
- `range_scanner/validation/evaluation.py`
  - Metrics, report construction, error reduction, distribution checks.
- `range_scanner/validation/alignment.py`
  - Reference and blind registration utilities.
- `range_scanner/validation/fusion.py`
  - Reconstruction/fusion helpers.
- `range_scanner/validation/degradation.py`
  - Maritime degradation models.

## Script Categories

See `scripts/README.md` for the detailed map. High-level groupings:

- validation runners
- scene generators
- path/animation helpers
- processed bathymetry ingestion
- real point-cloud extraction/comparison

## Ignored / Generated Areas

- `generated_scenes/`
- `generated_scenes_multi_seed/`
- local vendor clones under `external/`

These are intentionally ignored because they contain reproducible artifacts,
large downloads, reports, and experiment outputs rather than source.
