# Reproducing the Charter Validation Evidence

This document provides step-by-step commands to reproduce every live Blender result
used as charter validation evidence.
Pre-generated output files are committed under `validation/reports/` and `validation/images/`
so reviewers can inspect results without re-running Blender.

---

## Prerequisites

```bash
# 1. Clone the repo
git clone <repo-url>
cd blainder-range-scanner

# 2. Install Python test dependencies (no Blender needed for pytest)
uv venv --python 3.11
uv pip install numpy scipy open3d==0.18.0 matplotlib pytest -r range_scanner/requirements.txt

# 3. Install range-scanner deps into Blender's Python (needed for Blender runs)
/Applications/Blender.app/Contents/Resources/5.0/python/bin/python3.11 \
    -m pip install -r range_scanner/requirements.txt
```

Set `BLENDER` to your Blender 5.0+ binary for the commands below:

```bash
export BLENDER="/Applications/Blender.app/Contents/MacOS/Blender"
```

---

## 1. Pure-Python pytest suite (TO1–TO5, D4, 6.1)

Covers all acceptance-threshold tests, statistics tests, fusion tests, and
visualization tests without requiring Blender.

```bash
pytest tests/ -v
# Expected: 252 passed
```

Key test classes and what they prove:

| Test file / class | Charter item |
|---|---|
| `test_acceptance_thresholds.py::TestTO3DepthDistribution` | TO3: KS D < 0.15, bin dev < 10% |
| `test_acceptance_thresholds.py::TestTO4ErrorReduction` | TO4: baseline ≥ 2 m, post-fusion ≤ 0.15 m, ≥ 80% reduction |
| `test_acceptance_thresholds.py::TestTO5Performance` | TO5: ≤ 10 min, ≤ 8 GB |
| `test_acceptance_thresholds.py::TestPointDensity` | 6.1: point density wired into report, passes threshold |
| `test_evaluation_metrics.py` | D4: RMSE/MAE/P95/KS correctness |
| `test_visualize_report.py` | Demo2, Demo3: PNG generation verified |
| `test_degradation.py` | D2: GNSS drift, sea-state, Gaussian noise ranges |
| `test_fusion.py` + `test_fusion_reconstruction.py` | D3: voxel fusion, medium separation |

---

## 2. D2 / TO3 — NOAA depth distribution validation (live Blender)

**Pre-generated result:** `validation/reports/noaa_cb_distribution_validation_report.json`
KS=0.099 (threshold 0.15) ✓, max\_bin\_dev=0.070 (threshold 0.10) ✓, passes\_all=true

**To reproduce, define a working directory first:**

```bash
export BATHY_DIR="generated_scenes/real_bathy/chesapeake_300m"
mkdir -p "$BATHY_DIR"
```

### 2a. Obtain the NOAA Chesapeake Bay netCDF

Use the included discovery script to locate and download the survey:

```bash
python3 scripts/find_noaa_bathy_candidates.py \
    --bbox-lon -76.35 -76.27 --bbox-lat 38.70 38.75 \
    --year 2017 --format netcdf --download \
    --output-dir "$BATHY_DIR"

# Rename the downloaded .nc file to a known name:
mv "$BATHY_DIR"/*.nc "$BATHY_DIR/chesapeake_bay_M130_2017.nc"
```

The script queries the NOAA NCEI ArcGIS feature service for surveys matching the
Chesapeake Bay 300 m crop footprint.
If automated download is unavailable, search manually at
`https://www.ncei.noaa.gov/maps/bathymetry/` for survey ID `M130` (2017,
Chesapeake Bay) and place the netCDF at `$BATHY_DIR/chesapeake_bay_M130_2017.nc`.

### 2b. Build the 300 m crop OBJ + metadata

```bash
python3 scripts/processed_bathy_to_obj.py \
    --input "$BATHY_DIR/chesapeake_bay_M130_2017.nc" \
    --output-dir "$BATHY_DIR" \
    --center-lon -76.309411135 --center-lat 38.72346898 --size-m 300 \
    --prefix seabed_crop_300m
```

### 2c. Import the OBJ into a Blender scene

```bash
$BLENDER --background --python scripts/import_obj_to_blender_scene.py -- \
    --obj "$BATHY_DIR/seabed_crop_300m.obj" \
    --output "$BATHY_DIR/seabed_crop_300m.blend"
```

### 2d. Add sensors to produce the scan-ready scene

```bash
$BLENDER --background --python scripts/make_scan_ready_real_bathy_scene.py -- \
    --input "$BATHY_DIR/seabed_crop_300m.blend" \
    --output "$BATHY_DIR/seabed_crop_300m_scan_ready.blend"
```

### 2e. Prepare reference depth array (already committed)

The committed file `validation/reference_data/cb_noaa_reference_depths.npy` contains 1140 depth
values extracted from the same netCDF crop.  To regenerate it:

```bash
python3 scripts/prepare_reference_depth_distribution.py convert \
    --input "$BATHY_DIR/chesapeake_bay_M130_2017.nc" \
    --format xyz \
    --output validation/reference_data/cb_noaa_reference_depths.npy \
    --min-depth -7.0 --max-depth 0.0
```

### 2f. Run the validation

```bash
python3 scripts/run_fixed_scene_validation_batch.py \
    --blender "$BLENDER" \
    --scene "$BATHY_DIR/seabed_crop_300m_scan_ready.blend" \
    --config scripts/real_bathy_noaa_degraded_validation.json
```

Output report written to `$BATHY_DIR/output/noaa_cb_baseline_multi_sensor_report.json`.

Verify:

```bash
python3 -c "
import json, os
r = json.load(open(os.environ['BATHY_DIR'] + '/output/noaa_cb_baseline_multi_sensor_report.json'))
dd = r['depth_distribution_comparison']
print('ks_statistic:', dd['ks_statistic'])       # expect < 0.15
print('max_bin_deviation:', dd['max_bin_deviation'])  # expect < 0.10
print('passes_all:', dd['passes_all'])            # expect True
"
```

---

## 3. TO4 — Degradation → Blind-ICP fusion error reduction (live Blender)

**Pre-generated result:** `validation/reports/degraded_gnss_full_sea_state_multi_sensor_report.json`
positional\_degradation=2.01 m ✓, post\_fusion\_rmse=0.129 m ✓, error\_reduction=93.6% ✓

> **Note:** The top-level `"status": "fail"` in this report reflects the **pre-fusion**
> degraded RMSE (0.41 m) failing the 0.15 m threshold — this is the intentional
> degraded baseline that TO4 requires to be ≥ 2 m positional error.
> The charter acceptance result is in `technical_objectives.TO4.passes_all`, which is `true`.

**To reproduce:**

```bash
export DEGRADED_DIR="generated_scenes/degraded"
mkdir -p "$DEGRADED_DIR"
```

### 3a. Generate the base scene (seed 42)

```bash
$BLENDER --background --python scripts/generate_seeded_dual_sensor_scene.py -- \
    --seed 42 \
    --output-dir "$DEGRADED_DIR"
```

This creates `$DEGRADED_DIR/dual_sensor_scene_42.blend`.

### 3b. Run the degraded animated path validation

```bash
python3 scripts/run_animated_path_validation.py \
    --blender "$BLENDER" \
    --base-scene "$DEGRADED_DIR/dual_sensor_scene_42.blend" \
    --config scripts/degraded_animated_path_validation.example.json
```

This generates the animated scene and runs both `degraded_gnss_2m` and
`degraded_gnss_full_sea_state` trials under `$DEGRADED_DIR/output/`.

Verify TO4 (can also inspect the pre-generated report without re-running):

```bash
python3 -c "
import json
r = json.load(open('validation/reports/degraded_gnss_full_sea_state_multi_sensor_report.json'))
to4 = r['technical_objectives']['TO4']
print('passes_all:', to4['passes_all'])                                              # True
print('positional_degradation_magnitude_m:', to4['positional_degradation_magnitude_m'])  # >= 2.0
print('post_fusion_rmse:', to4['post_fusion_rmse'])                                  # <= 0.15
print('positional_error_reduction_pct:', to4['positional_error_reduction_pct'])      # >= 80
perf = r['performance']
print('wall_time_minutes:', perf['wall_time_minutes'])                               # <= 10
print('peak_memory_gb:', perf['peak_memory_gb'])                                     # <= 8
"
```

---

## 4. Generating demo PNGs from committed reports

The five PNGs in `validation/images/` were generated with:

```bash
# TO3/D2 proof plots (depth histogram, RMSE, heatmap)
python3 scripts/visualize_validation_report.py \
    validation/reports/noaa_cb_distribution_validation_report.json

# TO4 proof plots (RMSE comparison, error heatmap)
python3 scripts/visualize_validation_report.py \
    validation/reports/degraded_gnss_full_sea_state_multi_sensor_report.json
```

---

## 5. Summary of committed evidence

| Charter item | Evidence file | Re-run command |
|---|---|---|
| TO1 reproducibility | `tests/test_error_distribution.py` | `pytest tests/test_error_distribution.py` |
| TO2 noise ranges | `tests/test_degradation.py` | `pytest tests/test_degradation.py` |
| TO3 KS / bin deviation | `validation/reports/noaa_cb_distribution_validation_report.json` | Section 2 above |
| TO4 error reduction | `validation/reports/degraded_gnss_full_sea_state_multi_sensor_report.json` | Section 3 above |
| TO5 performance | `validation/reports/degraded_gnss_full_sea_state_multi_sensor_report.json` (performance block) + `tests/test_acceptance_thresholds.py::TestTO5Performance` | `pytest tests/test_acceptance_thresholds.py -k TO5` |
| D2 NOAA distribution | `validation/reports/noaa_cb_distribution_validation_report.json` | Section 2 above |
| D4 statistics framework | `tests/test_evaluation_metrics.py` | `pytest tests/test_evaluation_metrics.py` |
| 6.1 point density | `tests/test_acceptance_thresholds.py::TestPointDensity` | `pytest tests/test_acceptance_thresholds.py -k density` |
| Demo2 RMSE PNG | `validation/images/degraded_gnss_full_sea_state_multi_sensor_report_rmse_comparison.png` | Section 4 above |
| Demo3 heatmap PNG | `validation/images/degraded_gnss_full_sea_state_multi_sensor_report_error_heatmap.png` | Section 4 above |
