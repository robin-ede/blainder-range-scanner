# Real Bathymetry Validation

This workflow supports two real-data validation paths:

1. processed bathymetry → mesh → synthetic scan
2. real sounder points → local patch → synthetic rescan → point-cloud comparison

---

## 1. Find A Survey Candidate

```bash
python scripts/find_noaa_bathy_candidates.py --source nearshore --top 10
```

For shallow coastal validation, Chesapeake Bay Office surveys are currently the
best fit because they are shallow, compact, and morphologically similar to the
target use case.

---

## 2. Processed Bathymetry Path

Preferred when the survey offers a processed grid (`.nc`, `.tif`, `.bag`).

```bash
# Define your working directory
export BATHY_DIR="path/to/your/bathy_output"
mkdir -p "$BATHY_DIR"
```

Convert a crop into an OBJ plus reference depth grid:

```bash
python scripts/processed_bathy_to_obj.py \
  --input path/to/bathy.nc \
  --output-dir "$BATHY_DIR" \
  --center-lon <lon> \
  --center-lat <lat> \
  --size-m 300 \
  --prefix seabed_crop
```

Import into Blender and add sensors:

```bash
python scripts/import_obj_to_blender_scene.py \
  --obj "$BATHY_DIR/seabed_crop.obj" \
  --output "$BATHY_DIR/seabed_crop.blend"

python scripts/make_scan_ready_real_bathy_scene.py \
  --input "$BATHY_DIR/seabed_crop.blend" \
  --output "$BATHY_DIR/seabed_crop_scan_ready.blend"
```

---

## 3. Real Sounder Point Path

Narrower and more experimental, but stronger for arguing that synthetic
points resemble real measured points.

### Extract Kongsberg points

Requires a local clone of the [`pyall`](https://github.com/guardiangeomatics/pyall) reader in `external/pyall-github/`:

```bash
git clone https://github.com/guardiangeomatics/pyall.git external/pyall-github
```

```bash
python scripts/extract_kongsberg_all_points.py \
  --input path/to/survey.all.mb58 \
  --output-prefix path/to/output/survey_points \
  --max-pings 200
```

### Build a local patch mesh from real points

```bash
python scripts/pointcloud_to_obj.py \
  --input path/to/output/survey_points.npy \
  --output-dir path/to/output/patch \
  --cell-size 0.25 \
  --prefix patch_mesh
```

### Create a scan-ready scene and run synthetic scan

```bash
python scripts/import_obj_to_blender_scene.py \
  --obj path/to/output/patch/patch_mesh.obj \
  --output path/to/output/patch/patch.blend

python scripts/make_scan_ready_real_bathy_scene.py \
  --input path/to/output/patch/patch.blend \
  --output path/to/output/patch/patch_scan_ready.blend
```

Use `scripts/fixed_scene_validation.example.json` as a starting point,
adapting the scene path and output settings for an underwater, single-sensor comparison run.

### Compare real vs synthetic points

```bash
python scripts/compare_real_vs_synthetic_points.py \
  --real-points path/to/output/survey_points.npy \
  --mesh-meta path/to/output/patch/patch_mesh_metadata.json \
  --synthetic-las path/to/output/patch/baseline_multi_sensor.las \
  --output path/to/output/patch/compare_baseline.json
```

---

## Metrics We Currently Use

- bounding-box agreement
- depth range and depth statistics
- point density per square metre
- nearest-neighbour spacing statistics
- KS statistic on depth distribution
- KS statistic on spacing distribution
- symmetric nearest-neighbour cloud distance (Chamfer-style)

---

## Current Interpretation Guidance

- processed bathymetry validation is the stable, repeatable path
- real sounder point extraction is the stronger realism argument, but more
  format-specific and experimental
- together they support the claim that the synthetic pipeline is grounded in
  both real seabed morphology and real sampled point behavior
