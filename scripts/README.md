# Scripts Guide

This folder contains the headless workflows used for scene generation, validation,
and real-bathymetry experiments.

## Main Entry Points

- `run_fixed_scene_validation_batch.py`
  - Canonical fixed-scene validation runner.
- `run_fixed_scene_validation_trial.py`
  - Runs one fixed-scene trial in Blender.
- `run_pose_then_noise_validation.py`
  - Pose sweep followed by noise sweep.
- `run_multi_seed_pose_then_noise_validation.py`
  - Repeats the pose/noise workflow across generated scenes.
- `run_animated_path_validation.py`
  - Path-based animated validation.
- `create_animated_validation_scene.py`
  - Applies waypoint/path configs to a `.blend` scene.

## Scene Generation

- `generate_seeded_dual_sensor_scene.py`
  - Builds one deterministic synthetic scene.
- `generate_seeded_dual_sensor_batch.py`
  - Generates multiple seeded scenes.

## Real Bathymetry / Real Data Helpers

- `find_noaa_bathy_candidates.py`
  - Finds NOAA survey candidates and ranks them by shallow-water suitability.
- `download_noaa_survey_assets.py`
  - Scrapes linked NOAA survey assets from a survey page.
- `processed_bathy_to_obj.py`
  - Converts processed bathymetry (`.nc`, `.tif`, `.bag`) into OBJ + reference depths.
- `prepare_reference_depth_distribution.py`
  - Builds reference depth arrays for distribution comparisons.
- `extract_kongsberg_all_points.py`
  - Extracts real point clouds from Kongsberg `.all` files using the vendored `pyall` clone.
- `pointcloud_to_obj.py`
  - Builds a gridded mesh from a real point cloud.
- `compare_real_vs_synthetic_points.py`
  - Compares real and synthetic point clouds on the same geometry.

## Scene Prep Utilities

- `import_obj_to_blender_scene.py`
  - Imports an OBJ into a clean Blender scene.
- `make_scan_ready_real_bathy_scene.py`
  - Adds water surface and co-mounted sensor placement for scanning.
- `make_review_real_bathy_scene.py`
  - Creates an easier-to-inspect review scene.
- `path_generation.py`
  - Shared geometry-based path generation helpers.
- `generate_geometry_path_config.py`
  - Generates path JSON from a scene object's bounds.

## Example / Template Configs

Keep and reuse the `*.example.json` files as the public templates. Non-example
JSON files in this folder are working configs created during development and
should only be kept if they represent a canonical workflow.

Current canonical templates:

- `fixed_scene_validation.example.json`
- `fixed_scene_noise_sweep.example.json`
- `fixed_scene_pose_sweep.example.json`
- `pose_then_noise_validation.example.json`
- `multi_seed_pose_then_noise_validation.example.json`
- `animated_accumulated_validation.example.json`
- `animated_path_validation.example.json`
- `degraded_animated_path_validation.example.json`
- `real_bathy_validation.example.json`
- `rainier_patch_underwater_points.example.json`

## Naming Conventions

- `run_*.py`
  - main workflow runners
- `generate_*.py`
  - scene/config generation helpers
- `*_to_obj.py`
  - geometry conversion utilities
- `make_*_scene.py`
  - Blender scene preparation helpers
- `*.example.json`
  - public, reusable config templates

## Notes

- Generated outputs belong under ignored output folders like `generated_scenes/`.
- `extract_kongsberg_all_points.py` expects a local clone of the real GitHub
  `pyall` reader under `external/pyall-github/`, which is ignored by git.
