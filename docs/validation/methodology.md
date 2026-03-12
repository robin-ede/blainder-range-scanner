# Validation Methodology

This document describes the statistical tests, QA metrics, and evaluation
procedures used in Project M&M to assess the accuracy and realism of synthetic
sensor data and the effectiveness of the sensor fusion pipeline. It is intended
to provide transparency to the MerLion Advisory sponsor and to ensure the
validation process is repeatable and aligned with the project charter.

---

## 1. Overview

The validation framework operates across five logical stages that mirror the
system architecture (see `docs/architecture.md`):

| Stage | What is validated | Key metric |
|---|---|---|
| Ground truth | Scene geometry is deterministic and reproducible | Seed-based reproduction |
| Pre-fusion | Raw degraded point cloud accuracy vs. ground truth | Surface-distance RMSE |
| Fusion | Blind ICP alignment corrects drift to within target | Post-fusion RMSE, error reduction % |
| Distribution | Synthetic depth distribution matches real-world reference | KS statistic D, bin deviation |
| Performance | Pipeline completes within practical resource limits | Wall time, peak memory |

All results are written to a structured JSON report (`*_report.json`) alongside
the scan output. The report schema is version-controlled (schema_version = 1)
with stable keys to support longitudinal comparison.

---

## 2. Acceptance Criteria (Charter Requirements)

The following thresholds are the formal pass/fail criteria for Project M&M.
Each is directly traceable to a Technical Objective (TO) in the charter.

### TO3 — Statistical Realism

| Criterion | Threshold | Report key | Test |
|---|---|---|---|
| KS two-sample statistic D | < 0.15 | `depth_distribution_comparison.passes_ks_threshold` | `TestTO3StatisticalRealism` |
| Maximum per-bin density deviation | < 10% (< 0.10) | `depth_distribution_comparison.passes_bin_deviation_threshold` | `TestTO3StatisticalRealism` |
| Both pass simultaneously | — | `depth_distribution_comparison.passes_all` | `TestTO3StatisticalRealism` |

### TO4 — Fusion Improvement

| Criterion | Threshold | Report key | Test |
|---|---|---|---|
| Positional degradation baseline | ≥ 2.0 m | `technical_objectives.TO4.meets_positional_baseline_2m` | `TestTO4FusionImprovement` |
| Post-fusion RMSE (blind ICP stage) | ≤ 0.15 m | `technical_objectives.TO4.passes_post_fusion_rmse_target` | `TestTO4FusionImprovement` |
| Error reduction (positional) | ≥ 80% | `technical_objectives.TO4.meets_80pct_reduction` | `TestTO4FusionImprovement` |
| Alignment backend | Open3D ICP | `technical_objectives.TO4.passes_backend_requirement` | `TestTO4FusionImprovement` |
| All TO4 criteria pass | — | `technical_objectives.TO4.passes_all` | `TestTO4FusionImprovement` |

### TO5 — Performance Constraints

| Criterion | Threshold | Report key | Test |
|---|---|---|---|
| Wall-clock time | ≤ 10 minutes | `performance.passes_time_threshold` | `TestTO5Performance` |
| Peak Python heap memory | ≤ 8 GB | `performance.passes_memory_threshold` | `TestTO5Performance` |

All criteria are asserted automatically by the pytest suite in
`tests/test_acceptance_thresholds.py` against the committed fixture
`tests/fixtures/sample_passing_report.json`.

---

## 3. Metric Definitions

### 3.1 Surface-Distance RMSE

The primary accuracy metric. For each evaluated point, the nearest-surface
distance to the ground-truth mesh is computed using Blender's BVH tree
(`mathutils.bvhtree.BVHTree.find_nearest`). RMSE is then:

```
RMSE = sqrt( mean( d_i^2 ) )
```

where `d_i` is the nearest-surface distance for point `i`.

RMSE is computed at each pipeline stage to show the trajectory from degraded
input to corrected output.

### 3.2 MAE, Median, P95, Max

Reported alongside RMSE for each stage to characterise the full error
distribution:

- **MAE** — mean absolute error (less sensitive to outliers than RMSE)
- **Median** — robust central tendency
- **P95** — 95th percentile (worst-case practical bound)
- **Max** — single largest error (outlier check)

### 3.3 KS Two-Sample Statistic

The Kolmogorov–Smirnov (KS) two-sample test compares the depth distributions of
the synthetic point cloud and a reference (real-world or raster-derived) cloud:

```
D = sup_x | F_synthetic(x) - F_reference(x) |
```

`D = 0` means the distributions are identical; `D = 1` means they are
completely non-overlapping. The charter threshold is `D < 0.15`.

Computed in `range_scanner/validation/evaluation.py:compare_depth_distributions`
using `scipy.stats.ks_2samp` when available, with a manual CDF implementation
as fallback.

### 3.4 Per-Bin Density Deviation

The depth range is divided into fixed bins. For each bin, the absolute difference
between normalised synthetic and reference histogram densities is computed:

```
deviation_k = | density_synthetic_k - density_reference_k |
```

`max_bin_deviation` must be < 0.10 (< 10%) across all bins.

### 3.5 Positional Degradation Magnitude

The charter TO4 baseline requirement (≥ 2.0 m) is evaluated against the
**maximum per-frame positional correction** applied by the reference aligner
— i.e., the measured sensor-position drift magnitude. This is the physically
correct interpretation: the aligner literally corrected 2+ metres of positional
drift. Surface-distance RMSE is not used for this check because mesh curvature
can make RMSE appear small even when the actual position offset is large.

### 3.6 Error Reduction Percentage

```
error_reduction_pct = (degraded_rmse - post_fusion_rmse) / degraded_rmse × 100
```

Computed at the positional level for TO4 (using the positional degradation
magnitude as the denominator) to give the physically meaningful recovery
fraction.

---

## 4. Pipeline Stages and Their Reports

Each stage below writes metrics into the JSON report under a named key. The
`visualize_validation_report.py` RMSE chart shows all stages side by side.

| Stage key | What it measures | Corresponding code |
|---|---|---|
| `pre_fusion_combined_metrics` | Raw degraded hits (noise + drift) vs. ground truth | `evaluation.evaluate_hits_against_targets` |
| `post_processing_fusion` | Voxel-fused raw hits vs. ground truth | `fusion.voxel_fuse_hits` |
| `blind_post_processing_fusion` | **TO4 official stage** — Blind ICP + voxel fusion | `alignment.align_hits_blind` + `fusion.voxel_fuse_aligned_hits` |
| `medium_separated_reconstruction` | Separate above/below-water voxel fusion | `fusion.reconstruct_medium_separated_map` |
| `medium_height_grid_reconstruction` | XY grid height reconstruction | `fusion.reconstruct_medium_height_grid_map` |
| `alignment_correction` | Reference-corrected points (oracle, not blind) | `alignment.align_hits_to_reference` |
| `blind_alignment_correction` | Blind ICP-corrected points only | `alignment.align_hits_blind` |
| `corrected_grid_reconstruction` | Reference-corrected grid | `fusion.reconstruct_medium_height_grid_from_points` |
| `blind_corrected_grid_reconstruction` | Blind ICP-corrected grid | `fusion.reconstruct_medium_height_grid_from_points` |

The **TO4 official stage** is always `blind_post_processing_fusion` because it
uses only information available without ground-truth access (blind ICP
registration), making it the operationally relevant fusion output.

---

## 5. Reprojection Heatmap

The reprojection error heatmap (`*_error_heatmap.png`) visualises the spatial
distribution of surface-distance errors across the XY plane of the scan area.

**Data source:** `blind_post_processing_fusion.spatial_error_samples` — up to
8,000 `[x, y, z, distance]` records stored during evaluation.

**Rendering:** XY space is divided into a 60 × 60 grid; mean surface-distance
error is computed per cell and rendered using the `RdYlGn_r` colourmap (red =
high error, green = low error). The acceptance threshold (0.15 m) is marked on
the colourbar.

This plot directly satisfies the demo-component requirement (charter Section 7.3,
"Reprojection Heatmap Output") by making localised misalignments immediately
visible to stakeholders.

---

## 6. Reproducibility Controls

All results are reproducible by design:

| Control | Mechanism |
|---|---|
| Scene geometry | Integer seed passed to `generate_seeded_dual_sensor_scene.py` |
| Noise sampling | `noiseSeed` stored in JSON config and `error_distribution.setSeed()` |
| Degradation parameters | Fully serialised in `experiment_config` block of report |
| Scanner configuration | Fully serialised in `scanner_config` block of report |
| Pipeline version | Git commit hash (not yet auto-captured; recommended for future) |

To reproduce a run: retrieve the `experiment_config` and `scanner_config` from
the report JSON, apply them to the same `.blend` scene, and run with the same
`noiseSeed`.

---

## 7. Per-Sensor and Per-Region Breakdowns

In addition to combined metrics, the framework computes:

- **Per-sensor metrics** (`per_sensor_pre_fusion_metrics`): separate RMSE/MAE/P95
  for each scanner object (LiDAR vs. sonar), enabling identification of
  sensor-specific accuracy gaps.

- **Per-region metrics** (`regional_pre_fusion_metrics`): separate metrics for
  points classified as `above_water` vs. `below_water` using the configurable
  `surfaceHeight` parameter. This validates that both sensors are contributing
  correctly to their respective operating domains.

---

## 8. QA Checks Performed Automatically by `pytest`

The following QA checks run without Blender and complete in < 5 seconds:

| Test file | What it checks |
|---|---|
| `tests/test_acceptance_thresholds.py` | All TO3/TO4/TO5 threshold flags against the committed passing fixture |
| `tests/test_evaluation_metrics.py` | RMSE, MAE, KS, histogram, and error-reduction computation correctness |
| `tests/test_evaluation_report.py` | Report assembly, JSON serialisation, schema structure |
| `tests/test_evaluation_spatial.py` | Spatial sample collection and medium-classification logic |
| `tests/test_degradation.py` | GNSS drift and sea-state offset functions |
| `tests/test_alignment.py` | ICP alignment helpers |
| `tests/test_fusion.py` | Voxel fusion weight functions |
| `tests/test_fusion_reconstruction.py` | Grid reconstruction |
| `tests/test_visualize_report.py` | All three demo-component PNG outputs generated correctly |
| `tests/test_error_distribution.py` | Gaussian noise seed and distribution |
| `tests/test_fresnel.py` | Fresnel optical equation helpers |

Run all checks with:

```bash
pytest
```

---

## 9. Interpreting Results

A trial is considered **fully passing** when all of the following are true in
the report JSON:

```json
{
  "status": "pass",
  "depth_distribution_comparison": { "passes_all": true },
  "technical_objectives": {
    "TO4": { "passes_all": true }
  },
  "performance": {
    "passes_time_threshold": true,
    "passes_memory_threshold": true
  }
}
```

The RMSE bar chart (`*_rmse_comparison.png`) provides a visual summary: all
post-fusion bars in the green zone with the annotation showing ≥ 80% reduction
confirms TO4 at a glance. The heatmap confirms no systematic spatial
misalignments. The histogram confirms TO3 distribution realism.
