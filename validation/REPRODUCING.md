# Reproducing the Charter Validation Evidence

This document provides step-by-step commands to reproduce every live Blender result
used as charter validation evidence.
Pre-generated output files are committed under `docs/validation/` and `validation/images/`
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

**To reproduce:**

### 2a. Obtain the NOAA Chesapeake Bay netCDF

Download the M130 2017 survey from NOAA National Centers for Environmental Information
and place it at:

```
generated_scenes/animated_paths/output/real_bathy/chesapeake_bay_M130_2017.nc
```

(The exact file can be located via `scripts/find_noaa_bathy_candidates.py`.)

### 2b. Build the 300 m crop OBJ + metadata

```bash
mkdir -p generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small
python3 scripts/processed_bathy_to_obj.py \
    --input generated_scenes/animated_paths/output/real_bathy/chesapeake_bay_M130_2017.nc \
    --output-dir generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small \
    --center-lon -76.309411135 --center-lat 38.72346898 --size-m 300 \
    --prefix seabed_crop_300m
```

### 2c. Import the OBJ into a Blender scene

```bash
$BLENDER --background --python scripts/import_obj_to_blender_scene.py -- \
    --obj generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/seabed_crop_300m.obj \
    --output generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/seabed_crop_300m.blend
```

### 2d. Add sensors to produce the scan-ready scene

```bash
$BLENDER --background --python scripts/make_scan_ready_real_bathy_scene.py -- \
    --input generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/seabed_crop_300m.blend \
    --output generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/seabed_crop_300m_auto_fast.blend
```

### 2e. Prepare reference depth array (already committed)

The committed file `validation/reference_data/cb_noaa_reference_depths.npy` contains 1140 depth
values extracted from the same netCDF crop.  To regenerate it:

```bash
python3 scripts/prepare_reference_depth_distribution.py convert \
    --input generated_scenes/animated_paths/output/real_bathy/chesapeake_bay_M130_2017.nc \
    --format xyz \
    --output validation/reference_data/cb_noaa_reference_depths.npy \
    --min-depth -7.0 --max-depth 0.0
```

### 2f. Run the validation

```bash
python3 scripts/run_fixed_scene_validation_batch.py \
    --blender "$BLENDER" \
    --scene generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/seabed_crop_300m_auto_fast.blend \
    --config scripts/real_bathy_noaa_degraded_validation.json
```

Output report: `generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/output/noaa_cb_degraded_validation/noaa_cb_baseline_multi_sensor_report.json`

Verify:

```bash
python3 -c "
import json
r = json.load(open('generated_scenes/animated_paths/output/real_bathy/ld_11_04_cb_hacr_crop_small/output/noaa_cb_degraded_validation/noaa_cb_baseline_multi_sensor_report.json'))
dd = r['depth_distribution_comparison']
print('ks_statistic:', dd['ks_statistic'])   # expect < 0.15
print('max_bin_deviation:', dd['max_bin_deviation'])  # expect < 0.10
print('passes_all:', dd['passes_all'])        # expect True
"
```

---

## 3. TO4 — Degradation → Blind-ICP fusion error reduction (live Blender)

**Pre-generated result:** `generated_scenes/degraded_animated_paths/output/degraded_animated_trials/straight_push_degraded/degraded_gnss_full_sea_state_multi_sensor_report.json`
positional\_degradation=2.01 m ✓, post\_fusion\_rmse=0.028 m ✓, error\_reduction=93.3% ✓

**To reproduce:**

### 3a. Generate the base scene (seed 42)

```bash
$BLENDER --background --python scripts/generate_seeded_dual_sensor_scene.py -- \
    --seed 42 \
    --output-dir generated_scenes
```

This creates `generated_scenes/dual_sensor_scene_42.blend`.

### 3b. Run the degraded animated path validation

```bash
python3 scripts/run_animated_path_validation.py \
    --blender "$BLENDER" \
    --base-scene generated_scenes/dual_sensor_scene_42.blend \
    --config scripts/degraded_animated_path_validation.example.json
```

This generates the `straight_push_degraded` animated scene and runs both
`degraded_gnss_2m` and `degraded_gnss_full_sea_state` trials.

Verify TO4:

```bash
python3 -c "
import json
r = json.load(open('generated_scenes/degraded_animated_paths/output/degraded_animated_trials/straight_push_degraded/degraded_gnss_full_sea_state_multi_sensor_report.json'))
er = r['error_reduction']
print('positional_degradation_magnitude_m:', er['positional_degradation_magnitude_m'])  # >= 2.0
print('post_fusion_rmse:', er['post_fusion_rmse'])                                       # <= 0.15
print('error_reduction_pct:', er['error_reduction_pct'])                                 # >= 80
print('meets_80pct_reduction:', er['meets_80pct_reduction'])                             # True
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
    generated_scenes/degraded_animated_paths/output/degraded_animated_trials/straight_push_degraded/degraded_gnss_full_sea_state_multi_sensor_report.json
```

---

## 5. Summary of committed evidence

| Charter item | Evidence file | Re-run command |
|---|---|---|
| TO1 reproducibility | `tests/test_error_distribution.py` | `pytest tests/test_error_distribution.py` |
| TO2 noise ranges | `tests/test_degradation.py` | `pytest tests/test_degradation.py` |
| TO3 KS / bin deviation | `validation/reports/noaa_cb_distribution_validation_report.json` | Section 2 above |
| TO4 error reduction | `generated_scenes/degraded_animated_paths/.../degraded_gnss_full_sea_state_multi_sensor_report.json` | Section 3 above |
| TO5 performance | `tests/test_acceptance_thresholds.py::TestTO5Performance` | `pytest tests/test_acceptance_thresholds.py -k TO5` |
| D2 NOAA distribution | `validation/reports/noaa_cb_distribution_validation_report.json` | Section 2 above |
| D4 statistics framework | `tests/test_evaluation_metrics.py` | `pytest tests/test_evaluation_metrics.py` |
| 6.1 point density | `tests/test_acceptance_thresholds.py::TestPointDensity` | `pytest tests/test_acceptance_thresholds.py -k density` |
| Demo2 RMSE PNG | `validation/images/degraded_gnss_full_sea_state_multi_sensor_report_rmse_comparison.png` | Section 4 above |
| Demo3 heatmap PNG | `validation/images/degraded_gnss_full_sea_state_multi_sensor_report_error_heatmap.png` | Section 4 above |
