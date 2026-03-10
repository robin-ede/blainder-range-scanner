import bmesh
import bpy
import json
import math
import os
from datetime import datetime, timezone

import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

_scipy_stats = None
_SCIPY_AVAILABLE = False
try:
    from scipy import stats as _scipy_stats  # type: ignore[no-redef]

    _SCIPY_AVAILABLE = True
except ImportError:
    pass


def _metric_summary_from_values(values):
    if len(values) == 0:
        return {
            "point_count": 0,
            "rmse": None,
            "mae": None,
            "median": None,
            "p95": None,
            "max": None,
        }

    values = np.asarray(values, dtype=float)
    return {
        "point_count": int(values.size),
        "rmse": float(math.sqrt(np.mean(values**2))),
        "mae": float(np.mean(np.abs(values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def _build_bvh_trees(targets, depsgraph):
    trees = []

    for target in targets:
        bm = bmesh.new()
        bm.from_object(target, depsgraph=depsgraph)
        bm.transform(target.matrix_world)
        trees.append((target, BVHTree.FromBMesh(bm)))
        bm.free()

    return trees


def _nearest_distance(point, trees):
    closest_distance = None

    for _, tree in trees:
        location, _, _, distance = tree.find_nearest(Vector(point))
        if location is None or distance is None:
            continue
        if closest_distance is None or distance < closest_distance:
            closest_distance = float(distance)

    return closest_distance


def _point_medium(point, water_surface_height):
    if water_surface_height is None:
        return "unknown"
    if point[2] >= water_surface_height:
        return "above_water"
    return "below_water"


def _metric_summary(distances):
    return _metric_summary_from_values(distances)


def _sensor_counts(hits):
    counts = {}
    for hit in hits:
        sensor_id = hit.sensor_id or "unknown"
        counts[sensor_id] = counts.get(sensor_id, 0) + 1
    return counts


def evaluate_hits_against_targets(
    context,
    hits,
    targets,
    rmse_threshold=0.15,
    use_noisy_measurements=False,
    water_surface_height=None,
):
    depsgraph = context.evaluated_depsgraph_get()
    trees = _build_bvh_trees(targets, depsgraph)

    distances = []
    per_sensor_distances = {}
    regional_distances = {"above_water": [], "below_water": []}

    for hit in hits:
        point_location = hit.location
        if use_noisy_measurements and hit.noiseLocation is not None:
            point_location = hit.noiseLocation

        point = (point_location.x, point_location.y, point_location.z)
        distance = _nearest_distance(point, trees)
        if distance is None:
            continue

        distances.append(distance)
        sensor_id = hit.sensor_id or "unknown"
        per_sensor_distances.setdefault(sensor_id, []).append(distance)
        medium = _point_medium(point, water_surface_height)
        if medium in regional_distances:
            regional_distances[medium].append(distance)

    metrics = _metric_summary(distances)
    per_sensor_metrics = {
        sensor_id: _metric_summary(sensor_distances)
        for sensor_id, sensor_distances in per_sensor_distances.items()
    }
    regional_metrics = {
        region: _metric_summary(region_distances)
        for region, region_distances in regional_distances.items()
    }

    threshold_pass = metrics["rmse"] is not None and metrics["rmse"] <= float(
        rmse_threshold
    )

    return {
        "evaluation_mode": "pre_fusion_combined_accuracy",
        "evaluated_point_variant": (
            "noise_adjusted_points" if use_noisy_measurements else "raw_hit_points"
        ),
        "ground_truth_source": {
            "selection_mode": "visible_mesh_targets",
            "target_count": len(targets),
            "target_names": [target.name for target in targets],
        },
        "hit_counts": {
            "total": len(hits),
            "by_sensor": _sensor_counts(hits),
            "evaluated": metrics["point_count"],
        },
        "metrics": metrics,
        "per_sensor_pre_fusion_metrics": per_sensor_metrics,
        "regional_metrics": regional_metrics,
        "acceptance": {
            "rmse_threshold_m": float(rmse_threshold),
            "passes_rmse_threshold": threshold_pass,
        },
    }


def evaluate_points_against_targets(
    context,
    points,
    targets,
    rmse_threshold=0.15,
    water_surface_height=None,
    store_spatial_samples=False,
    max_spatial_samples=8000,
):
    depsgraph = context.evaluated_depsgraph_get()
    trees = _build_bvh_trees(targets, depsgraph)

    distances = []
    regional_distances = {"above_water": [], "below_water": []}
    spatial_samples = [] if store_spatial_samples else None

    for point in points:
        distance = _nearest_distance(point, trees)
        if distance is not None:
            distances.append(distance)
            medium = _point_medium(point, water_surface_height)
            if medium in regional_distances:
                regional_distances[medium].append(distance)
            if store_spatial_samples:
                spatial_samples.append(
                    [float(point[0]), float(point[1]), float(point[2]), float(distance)]
                )

    if spatial_samples is not None and len(spatial_samples) > max_spatial_samples:
        step = max(1, len(spatial_samples) // max_spatial_samples)
        spatial_samples = spatial_samples[::step]

    metrics = _metric_summary_from_values(distances)
    regional_metrics = {
        region: _metric_summary(region_distances)
        for region, region_distances in regional_distances.items()
    }
    threshold_pass = metrics["rmse"] is not None and metrics["rmse"] <= float(
        rmse_threshold
    )

    result = {
        "metrics": metrics,
        "regional_metrics": regional_metrics,
        "acceptance": {
            "rmse_threshold_m": float(rmse_threshold),
            "passes_rmse_threshold": threshold_pass,
        },
    }
    if store_spatial_samples:
        result["spatial_error_samples"] = spatial_samples
    return result


def compute_depth_histogram(depths, bin_edges):
    """Compute a normalised histogram over fixed bin edges.

    Parameters
    ----------
    depths : array-like of float
        Z-coordinate values (or distances) for each point.
    bin_edges : array-like of float
        Monotonically increasing bin boundaries (len = n_bins + 1).

    Returns
    -------
    dict with keys:
        bin_edges, counts (raw), density (normalised to sum-1).
    """
    depths = np.asarray(depths, dtype=float)
    counts, edges = np.histogram(depths, bins=bin_edges)
    total = float(counts.sum())
    density = (counts / total).tolist() if total > 0 else [0.0] * len(counts)
    return {
        "bin_edges": [float(e) for e in edges],
        "counts": counts.tolist(),
        "density": density,
    }


def compare_depth_distributions(
    synthetic_depths,
    reference_depths,
    bin_edges=None,
    ks_threshold=0.15,
    bin_deviation_threshold=0.10,
):
    """Compare synthetic and reference depth distributions.

    Computes:
    - Kolmogorov–Smirnov two-sample test statistic D (R14 criterion: D < 0.15)
    - Per-bin absolute density deviation and max deviation (R14: < 10%)
    - Pass/fail for each criterion

    Parameters
    ----------
    synthetic_depths : array-like of float
        Depth values from the synthetic sensor simulation.
    reference_depths : array-like of float
        Depth values from a real-world reference dataset (e.g. NOAA/USGS).
    bin_edges : array-like of float or None
        If None, uses 20 uniform bins spanning the combined range.
    ks_threshold : float
        KS statistic D threshold (default 0.15 per R14).
    bin_deviation_threshold : float
        Max per-bin density deviation threshold (default 0.10 = 10% per R14).

    Returns
    -------
    dict with full distribution comparison results.
    """
    synthetic_depths = np.asarray(synthetic_depths, dtype=float)
    reference_depths = np.asarray(reference_depths, dtype=float)

    if len(synthetic_depths) == 0 or len(reference_depths) == 0:
        return {
            "status": "insufficient_data",
            "synthetic_point_count": int(len(synthetic_depths)),
            "reference_point_count": int(len(reference_depths)),
            "ks_statistic": None,
            "ks_p_value": None,
            "passes_ks_threshold": None,
            "max_bin_deviation": None,
            "passes_bin_deviation_threshold": None,
            "passes_all": None,
        }

    # Determine bin edges from the combined range if not provided
    if bin_edges is None:
        combined_min = float(min(synthetic_depths.min(), reference_depths.min()))
        combined_max = float(max(synthetic_depths.max(), reference_depths.max()))
        bin_edges = np.linspace(combined_min, combined_max, 21)  # 20 bins

    synthetic_hist = compute_depth_histogram(synthetic_depths, bin_edges)
    reference_hist = compute_depth_histogram(reference_depths, bin_edges)

    # Per-bin absolute density deviation
    syn_density = np.asarray(synthetic_hist["density"], dtype=float)
    ref_density = np.asarray(reference_hist["density"], dtype=float)
    bin_deviations = np.abs(syn_density - ref_density).tolist()
    max_bin_deviation = float(np.max(bin_deviations))
    passes_bin_deviation = max_bin_deviation < bin_deviation_threshold

    # KS test
    if _SCIPY_AVAILABLE:
        ks_result = _scipy_stats.ks_2samp(synthetic_depths, reference_depths)
        ks_statistic = float(ks_result.statistic)
        ks_p_value = float(ks_result.pvalue)
    else:
        # Manual KS statistic (no p-value) when scipy is unavailable
        all_values = np.sort(np.concatenate([synthetic_depths, reference_depths]))
        n_syn = len(synthetic_depths)
        n_ref = len(reference_depths)
        cdf_syn = (
            np.searchsorted(np.sort(synthetic_depths), all_values, side="right") / n_syn
        )
        cdf_ref = (
            np.searchsorted(np.sort(reference_depths), all_values, side="right") / n_ref
        )
        ks_statistic = float(np.max(np.abs(cdf_syn - cdf_ref)))
        ks_p_value = None

    passes_ks = ks_statistic < ks_threshold

    return {
        "status": "computed",
        "synthetic_point_count": int(len(synthetic_depths)),
        "reference_point_count": int(len(reference_depths)),
        "scipy_available": _SCIPY_AVAILABLE,
        "ks_threshold": float(ks_threshold),
        "ks_statistic": ks_statistic,
        "ks_p_value": ks_p_value,
        "passes_ks_threshold": passes_ks,
        "bin_deviation_threshold": float(bin_deviation_threshold),
        "max_bin_deviation": max_bin_deviation,
        "bin_deviations": bin_deviations,
        "passes_bin_deviation_threshold": passes_bin_deviation,
        "passes_all": passes_ks and passes_bin_deviation,
        "synthetic_histogram": synthetic_hist,
        "reference_histogram": reference_hist,
    }


def compute_error_reduction(
    degraded_rmse,
    best_post_fusion_rmse,
    max_positional_correction_m=None,
    positional_baseline_threshold_m=2.0,
):
    """Compute RMSE error reduction between degraded baseline and post-fusion output.

    The "degraded baseline ≥ 2m" criterion is evaluated against the *maximum
    positional correction* that the reference aligner had to apply (i.e. the
    measured sensor-position drift magnitude), not against surface-distance RMSE.
    This is the correct interpretation: the aligner literally corrected
    ``max_positional_correction_m`` metres of drift; surface-distance RMSE can be
    smaller because the mesh curves back toward drifted points.

    Parameters
    ----------
    degraded_rmse : float or None
        Surface-distance RMSE of the raw degraded point cloud vs. ground truth.
    best_post_fusion_rmse : float or None
        Surface-distance RMSE of the best post-fusion output vs. ground truth.
    max_positional_correction_m : float or None
        Maximum per-frame positional correction applied by the reference aligner
        (from ``alignment_correction.translation_summary.max_translation_m``).
        Used to evaluate the ≥ 2m degraded-baseline requirement.
    positional_baseline_threshold_m : float
        Threshold for the positional baseline check (default 2.0 m).

    Returns
    -------
    dict with full error-reduction statistics and baseline pass/fail flags.
    """
    pos_correction = (
        float(max_positional_correction_m)
        if max_positional_correction_m is not None
        else None
    )
    meets_positional_baseline = (
        pos_correction >= positional_baseline_threshold_m
        if pos_correction is not None
        else None
    )

    if degraded_rmse is None or best_post_fusion_rmse is None or degraded_rmse == 0.0:
        return {
            "degraded_surface_rmse": degraded_rmse,
            "post_fusion_rmse": best_post_fusion_rmse,
            "positional_degradation_magnitude_m": pos_correction,
            "positional_baseline_threshold_m": positional_baseline_threshold_m,
            "meets_positional_baseline_2m": meets_positional_baseline,
            "error_reduction_pct": None,
            "meets_80pct_reduction": None,
            # Legacy key kept for backward-compat with batch CSV extractor
            "degraded_rmse": degraded_rmse,
            "meets_degraded_baseline_2m": meets_positional_baseline,
        }

    reduction_pct = ((degraded_rmse - best_post_fusion_rmse) / degraded_rmse) * 100.0

    return {
        "degraded_surface_rmse": float(degraded_rmse),
        "post_fusion_rmse": float(best_post_fusion_rmse),
        "positional_degradation_magnitude_m": pos_correction,
        "positional_baseline_threshold_m": positional_baseline_threshold_m,
        "meets_positional_baseline_2m": meets_positional_baseline,
        "error_reduction_pct": float(reduction_pct),
        "meets_80pct_reduction": reduction_pct >= 80.0,
        # Legacy key kept for backward-compat with batch CSV extractor
        "degraded_rmse": float(degraded_rmse),
        "meets_degraded_baseline_2m": meets_positional_baseline,
    }


def build_validation_report(properties, results, warnings=None):
    primary_location = None
    primary_rotation = None
    secondary_location = None
    secondary_rotation = None

    if properties.scannerObject is not None:
        primary_location = list(properties.scannerObject.location)
        primary_rotation = [
            float(value) for value in properties.scannerObject.rotation_euler
        ]

    if properties.scannerObject2 is not None:
        secondary_location = list(properties.scannerObject2.location)
        secondary_rotation = [
            float(value) for value in properties.scannerObject2.rotation_euler
        ]

    scanner_config = {
        "primary": {
            "object": properties.scannerObject.name
            if properties.scannerObject
            else None,
            "type": properties.scannerType,
            "location": primary_location,
            "rotation_euler_rad": primary_rotation,
        },
        "secondary": {
            "object": properties.scannerObject2.name
            if properties.scannerObject2
            else None,
            "type": properties.scannerType2,
            "location": secondary_location,
            "rotation_euler_rad": secondary_rotation,
        },
    }

    noise_config = {
        "add_noise": bool(properties.addNoise),
        "noise_type": properties.noiseType,
        "mu": float(properties.mu),
        "sigma": float(properties.sigma),
        "constant_noise": bool(properties.addConstantNoise),
        "absolute_offset": float(properties.noiseAbsoluteOffset),
        "relative_offset": float(properties.noiseRelativeOffset),
    }

    report = {
        "schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "evaluation_mode": results["evaluation_mode"],
        "evaluated_point_variant": results["evaluated_point_variant"],
        "aggregation_scope": (
            "accumulated_over_animation_frames"
            if properties.enableAnimation
            else "single_frame_snapshot"
        ),
        "scenario": {
            "data_file_name": properties.dataFileName,
            "data_file_path": bpy.path.abspath(properties.dataFilePath),
            "frame_start": int(properties.frameStart),
            "frame_end": int(properties.frameEnd),
            "frame_step": int(properties.frameStep),
            "single_ray": bool(properties.singleRay),
            "animation_enabled": bool(properties.enableAnimation),
        },
        "scanner_config": scanner_config,
        "noise_config": noise_config,
        "experiment_config": {
            "noise_seed": getattr(properties, "noiseSeed", None),
            "gnss_drift_m": getattr(properties, "gnssDriftMeters", None),
            "gnss_drift_heading_deg": getattr(
                properties, "gnssDriftHeadingDegrees", None
            ),
            "sea_state_amplitude_m": getattr(
                properties, "seaStateAmplitudeMeters", None
            ),
            "sea_state_period_frames": getattr(
                properties, "seaStatePeriodFrames", None
            ),
            "sea_state_phase_deg": getattr(properties, "seaStatePhaseDegrees", None),
        },
        "ground_truth_source": results["ground_truth_source"],
        "hit_counts": results["hit_counts"],
        "pre_fusion_combined_metrics": results["metrics"],
        "per_sensor_pre_fusion_metrics": results["per_sensor_pre_fusion_metrics"],
        "regional_pre_fusion_metrics": results.get("regional_metrics"),
        "medium_filtering": results.get("medium_filtering"),
        "degradation_modeling": results.get("degradation_modeling"),
        "post_processing_fusion": results.get("post_processing_fusion"),
        "medium_separated_reconstruction": results.get(
            "medium_separated_reconstruction"
        ),
        "medium_height_grid_reconstruction": results.get(
            "medium_height_grid_reconstruction"
        ),
        "alignment_correction": results.get("alignment_correction"),
        "blind_alignment_correction": results.get("blind_alignment_correction"),
        "corrected_grid_reconstruction": results.get("corrected_grid_reconstruction"),
        "blind_corrected_grid_reconstruction": results.get(
            "blind_corrected_grid_reconstruction"
        ),
        "depth_distribution_comparison": results.get("depth_distribution_comparison"),
        "performance": results.get("performance"),
        "acceptance": results["acceptance"],
        "status": "pass" if results["acceptance"]["passes_rmse_threshold"] else "fail",
        "status_description": (
            "Pre-fusion combined accuracy accumulated over animation frames and evaluated against the ground-truth surface."
            if properties.enableAnimation
            else "Pre-fusion combined accuracy evaluated against ground-truth surface."
        ),
        "error_reduction": results.get("error_reduction"),
        "per_frame_degraded_rmse": results.get("per_frame_degraded_rmse"),
        "warnings": warnings or [],
    }

    return report


def write_validation_report(file_path, file_name, report):
    absolute_path = bpy.path.abspath(file_path)
    os.makedirs(absolute_path, exist_ok=True)
    output_path = os.path.join(absolute_path, "%s_report.json" % file_name)

    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(report, output_file, indent=2, sort_keys=True)

    return output_path
