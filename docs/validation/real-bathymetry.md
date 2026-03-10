# Real Bathymetry Validation

This workflow supports two real-data validation paths:

1. processed bathymetry -> mesh -> synthetic scan
2. real sounder points -> local patch -> synthetic rescan -> point-cloud comparison

## 1. Find A Survey Candidate

Use:

```bash
python scripts/find_noaa_bathy_candidates.py --source nearshore --top 10
```

For shallow coastal validation, Chesapeake Bay Office surveys are currently the
best fit because they are shallow, compact, and morphologically similar to the
target use case.

## 2. Processed Bathymetry Path

Preferred when the survey offers a processed grid (`.nc`, `.tif`, `.bag`).

Convert a crop into an OBJ plus reference depth grid:

```bash
python scripts/processed_bathy_to_obj.py \
  --input path/to/bathy.nc \
  --output-dir output/real_bathy/crop \
  --center-lon <lon> \
  --center-lat <lat> \
  --size-m 300 \
  --prefix seabed_crop
```

Then import it into Blender:

```bash
python scripts/import_obj_to_blender_scene.py --obj ... --output ...
python scripts/make_scan_ready_real_bathy_scene.py --input ... --output ...
```

## 3. Real Sounder Point Path

This is narrower and more experimental, but stronger for arguing that synthetic
points resemble real measured points.

### Extract Rainier Kongsberg points

Requires a local clone of the real GitHub `pyall` reader in `external/pyall-github/`.

```bash
python scripts/extract_kongsberg_all_points.py \
  --input generated_scenes/.../rainier_sample.all.mb58 \
  --output-prefix generated_scenes/.../rainier_sample_points \
  --max-pings 200
```

### Build a local patch mesh from real points

```bash
python scripts/pointcloud_to_obj.py \
  --input generated_scenes/.../rainier_patch_points.npy \
  --output-dir generated_scenes/.../rainier_patch \
  --cell-size 0.25 \
  --prefix rainier_patch_mesh
```

### Create a scan-ready scene

```bash
python scripts/import_obj_to_blender_scene.py --obj ... --output ...
python scripts/make_scan_ready_real_bathy_scene.py --input ... --output ...
```

### Run synthetic scan on the same geometry

Use `scripts/rainier_patch_underwater_points.example.json` as the current
working example for an underwater, single-sensor comparison run.

### Compare real vs synthetic points

```bash
python scripts/compare_real_vs_synthetic_points.py \
  --real-points generated_scenes/.../rainier_patch_points.npy \
  --mesh-meta generated_scenes/.../rainier_patch_mesh_metadata.json \
  --synthetic-las generated_scenes/.../rainier_patch_underwater_points_baseline_multi_sensor.las \
  --output generated_scenes/.../compare_baseline.json
```

## Metrics We Currently Use

- bounding-box agreement
- depth range and depth statistics
- point density per square metre
- nearest-neighbour spacing statistics
- KS statistic on depth distribution
- KS statistic on spacing distribution
- symmetric nearest-neighbour cloud distance (Chamfer-style)

## Current Interpretation Guidance

- processed bathymetry validation is the stable, repeatable path
- real sounder point extraction is the stronger realism argument, but more
  format-specific and experimental
- together they support the claim that the synthetic pipeline is grounded in
  both real seabed morphology and real sampled point behavior
