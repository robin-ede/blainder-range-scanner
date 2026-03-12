#!/usr/bin/env python3

import argparse
import csv
import json
import os
import subprocess
import tempfile


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run repeatable fixed-scene validation trials in fresh Blender processes."
    )
    parser.add_argument("--blender", default="blender", help="Blender executable path.")
    parser.add_argument(
        "--scene", required=True, help="Path to the fixed .blend scene."
    )
    parser.add_argument("--config", required=True, help="Path to a batch JSON config.")
    parser.add_argument(
        "--summary-dir",
        default=None,
        help="Directory for batch summary artifacts. Defaults to the scene output directory.",
    )
    return parser.parse_args()


def load_batch_config(config_path):
    with open(config_path, "r", encoding="utf-8") as input_file:
        config = json.load(input_file)
    # Resolve referenceDepthFile relative to the config file's directory so
    # configs can use relative paths (e.g. "../../docs/validation/foo.npy") and
    # remain portable across machines.
    config_dir = os.path.dirname(os.path.abspath(config_path))
    ref = config.get("common_defaults", {}).get("referenceDepthFile")
    if ref and not os.path.isabs(ref):
        config["common_defaults"]["referenceDepthFile"] = os.path.abspath(
            os.path.join(config_dir, ref)
        )
    return config


def build_trial(batch_config, trial):
    merged = {
        "trial_name": trial["trial_name"],
        "primary": dict(batch_config.get("primary_defaults", {})),
        "secondary": dict(batch_config.get("secondary_defaults", {})),
        "common": dict(batch_config.get("common_defaults", {})),
        "primary_pose": dict(batch_config.get("primary_pose_defaults", {})),
        "secondary_pose": dict(batch_config.get("secondary_pose_defaults", {})),
        "water_profile": trial.get(
            "water_profile", batch_config.get("water_profile", None)
        ),
    }

    merged["primary"].update(trial.get("primary", {}))
    merged["secondary"].update(trial.get("secondary", {}))
    merged["common"].update(trial.get("common", {}))
    merged["primary_pose"].update(trial.get("primary_pose", {}))
    merged["secondary_pose"].update(trial.get("secondary_pose", {}))

    if "dataFileName" not in merged["common"]:
        merged["common"]["dataFileName"] = merged["trial_name"]

    return merged


def remove_invalid_characters(file_name):
    invalid_characters = '<>:"/\\|?*'
    return "".join(char for char in file_name if char not in invalid_characters)


def resolve_output_dir(scene_path, output_path):
    if output_path.startswith("//"):
        scene_dir = os.path.dirname(os.path.abspath(scene_path))
        return os.path.abspath(os.path.join(scene_dir, output_path[2:]))
    return os.path.abspath(output_path)


def get_expected_report_path(scene_path, trial):
    output_dir = resolve_output_dir(scene_path, trial["common"]["dataFilePath"])
    cleaned_file_name = remove_invalid_characters(
        trial["common"]["dataFileName"] + "_multi_sensor"
    )
    return os.path.join(output_dir, "%s_report.json" % cleaned_file_name)


def load_report(report_path):
    with open(report_path, "r", encoding="utf-8") as input_file:
        return json.load(input_file)


def summarize_trial(trial, report_path, report):
    metrics = report.get("pre_fusion_combined_metrics", {})
    post_fusion = report.get("post_processing_fusion", {})
    post_fusion_metrics = post_fusion.get("metrics", {})
    reconstruction = report.get("medium_separated_reconstruction", {})
    reconstruction_metrics = reconstruction.get("metrics", {})
    grid_reconstruction = report.get("medium_height_grid_reconstruction", {})
    grid_reconstruction_metrics = grid_reconstruction.get("metrics", {})
    alignment_correction = report.get("alignment_correction", {})
    alignment_metrics = alignment_correction.get("metrics", {})
    blind_alignment_correction = report.get("blind_alignment_correction", {})
    blind_alignment_metrics = blind_alignment_correction.get("metrics", {})
    blind_post_fusion = report.get("blind_post_processing_fusion", {})
    blind_post_fusion_metrics = blind_post_fusion.get("metrics", {})
    corrected_grid = report.get("corrected_grid_reconstruction", {})
    corrected_grid_metrics = corrected_grid.get("metrics", {})
    blind_corrected_grid = report.get("blind_corrected_grid_reconstruction", {})
    blind_corrected_grid_metrics = blind_corrected_grid.get("metrics", {})
    error_reduction = report.get("error_reduction", {})
    to4 = report.get("technical_objectives", {}).get("TO4", {})
    depth_dist = report.get("depth_distribution_comparison", {})
    performance = report.get("performance", {})
    regional_metrics = report.get("regional_pre_fusion_metrics", {})
    above_water_metrics = regional_metrics.get("above_water", {})
    below_water_metrics = regional_metrics.get("below_water", {})
    medium_filtering = report.get("medium_filtering", {})
    primary_filtering = medium_filtering.get("per_sensor", {}).get("primary", {})
    secondary_filtering = medium_filtering.get("per_sensor", {}).get("secondary", {})
    degradation_modeling = report.get("degradation_modeling", {})
    degradation_config = degradation_modeling.get("config", {})
    acceptance = report.get("acceptance", {})
    hit_counts = report.get("hit_counts", {})
    by_sensor = hit_counts.get("by_sensor", {})
    lidar_count = by_sensor.get("lidar", 0)
    sonar_count = by_sensor.get("sonar", 0)
    total_sensor_hits = lidar_count + sonar_count
    sensor_balance = None
    if total_sensor_hits > 0:
        sensor_balance = 1.0 - abs(lidar_count - sonar_count) / float(total_sensor_hits)

    return {
        "trial_name": trial["trial_name"],
        "report_path": report_path,
        "status": report.get("status"),
        "passes_rmse_threshold": acceptance.get("passes_rmse_threshold"),
        "rmse_threshold_m": acceptance.get("rmse_threshold_m"),
        "rmse": metrics.get("rmse"),
        "mae": metrics.get("mae"),
        "median": metrics.get("median"),
        "p95": metrics.get("p95"),
        "max": metrics.get("max"),
        "point_count": metrics.get("point_count"),
        "above_water_point_count": above_water_metrics.get("point_count"),
        "above_water_rmse": above_water_metrics.get("rmse"),
        "below_water_point_count": below_water_metrics.get("point_count"),
        "below_water_rmse": below_water_metrics.get("rmse"),
        "post_fusion_point_count": post_fusion_metrics.get("point_count"),
        "post_fusion_rmse": post_fusion_metrics.get("rmse"),
        "post_fusion_p95": post_fusion_metrics.get("p95"),
        "post_fusion_compression_ratio": post_fusion.get("point_counts", {}).get(
            "compression_ratio"
        ),
        "reconstructed_point_count": reconstruction_metrics.get("point_count"),
        "reconstructed_rmse": reconstruction_metrics.get("rmse"),
        "reconstructed_p95": reconstruction_metrics.get("p95"),
        "reconstructed_compression_ratio": reconstruction.get("point_counts", {}).get(
            "compression_ratio"
        ),
        "grid_reconstructed_point_count": grid_reconstruction_metrics.get(
            "point_count"
        ),
        "grid_reconstructed_rmse": grid_reconstruction_metrics.get("rmse"),
        "grid_reconstructed_p95": grid_reconstruction_metrics.get("p95"),
        "grid_reconstructed_compression_ratio": grid_reconstruction.get(
            "point_counts", {}
        ).get("compression_ratio"),
        "corrected_rmse": alignment_metrics.get("rmse"),
        "corrected_p95": alignment_metrics.get("p95"),
        "corrected_grid_point_count": corrected_grid_metrics.get("point_count"),
        "corrected_grid_rmse": corrected_grid_metrics.get("rmse"),
        "corrected_grid_p95": corrected_grid_metrics.get("p95"),
        "corrected_grid_compression_ratio": corrected_grid.get("point_counts", {}).get(
            "compression_ratio"
        ),
        "blind_corrected_rmse": blind_alignment_metrics.get("rmse"),
        "blind_corrected_p95": blind_alignment_metrics.get("p95"),
        "blind_post_fusion_point_count": blind_post_fusion_metrics.get("point_count"),
        "blind_post_fusion_rmse": blind_post_fusion_metrics.get("rmse"),
        "blind_post_fusion_p95": blind_post_fusion_metrics.get("p95"),
        "blind_post_fusion_compression_ratio": blind_post_fusion.get(
            "point_counts", {}
        ).get("compression_ratio"),
        "blind_corrected_grid_point_count": blind_corrected_grid_metrics.get(
            "point_count"
        ),
        "blind_corrected_grid_rmse": blind_corrected_grid_metrics.get("rmse"),
        "blind_corrected_grid_p95": blind_corrected_grid_metrics.get("p95"),
        "blind_corrected_grid_compression_ratio": blind_corrected_grid.get(
            "point_counts", {}
        ).get("compression_ratio"),
        "degraded_rmse": error_reduction.get("degraded_rmse"),
        "degraded_surface_rmse": error_reduction.get("degraded_surface_rmse"),
        "positional_degradation_magnitude_m": error_reduction.get(
            "positional_degradation_magnitude_m"
        ),
        "meets_positional_baseline_2m": error_reduction.get(
            "meets_positional_baseline_2m"
        ),
        "error_reduction_pct": error_reduction.get("error_reduction_pct"),
        "meets_80pct_reduction": error_reduction.get("meets_80pct_reduction"),
        "meets_degraded_baseline_2m": error_reduction.get("meets_degraded_baseline_2m"),
        "peak_frame_degraded_rmse": error_reduction.get("peak_frame_degraded_rmse"),
        "peak_frame_error_reduction_pct": error_reduction.get(
            "peak_frame_error_reduction_pct"
        ),
        "peak_frame_meets_80pct_reduction": error_reduction.get(
            "peak_frame_meets_80pct_reduction"
        ),
        "best_post_fusion_stage": error_reduction.get("best_post_fusion_stage"),
        "to4_official_stage": to4.get("official_stage_key"),
        "to4_post_fusion_rmse": to4.get("post_fusion_rmse"),
        "to4_error_reduction_pct": to4.get("error_reduction_pct"),
        "to4_positional_error_reduction_pct": to4.get("positional_error_reduction_pct"),
        "to4_passes_post_fusion_rmse_target": to4.get("passes_post_fusion_rmse_target"),
        "to4_passes_all": to4.get("passes_all"),
        "ks_statistic": depth_dist.get("ks_statistic"),
        "ks_p_value": depth_dist.get("ks_p_value"),
        "passes_ks_threshold": depth_dist.get("passes_ks_threshold"),
        "max_bin_deviation": depth_dist.get("max_bin_deviation"),
        "passes_bin_deviation_threshold": depth_dist.get(
            "passes_bin_deviation_threshold"
        ),
        "passes_distribution_all": depth_dist.get("passes_all"),
        "wall_time_seconds": performance.get("wall_time_seconds"),
        "wall_time_minutes": performance.get("wall_time_minutes"),
        "peak_memory_gb": performance.get("peak_memory_gb"),
        "passes_time_threshold": performance.get("passes_time_threshold"),
        "passes_memory_threshold": performance.get("passes_memory_threshold"),
        "primary_rejected_hits": primary_filtering.get("rejected_count"),
        "secondary_rejected_hits": secondary_filtering.get("rejected_count"),
        "gnss_drift_m": degradation_config.get("gnss_drift_m"),
        "sea_state_amplitude_m": degradation_config.get("sea_state_amplitude_m"),
        "warning_count": len(report.get("warnings", [])),
        "lidar_hit_count": lidar_count,
        "sonar_hit_count": sonar_count,
        "sensor_balance": sensor_balance,
        "primary_location": report.get("scanner_config", {})
        .get("primary", {})
        .get("location"),
        "secondary_location": report.get("scanner_config", {})
        .get("secondary", {})
        .get("location"),
    }


def write_summary(summary_dir, scene_path, summaries):
    os.makedirs(summary_dir, exist_ok=True)

    batch_name = os.path.splitext(os.path.basename(scene_path))[0]
    json_path = os.path.join(summary_dir, "%s_validation_summary.json" % batch_name)
    csv_path = os.path.join(summary_dir, "%s_validation_summary.csv" % batch_name)

    payload = {
        "scene_path": os.path.abspath(scene_path),
        "trial_count": len(summaries),
        "trials": summaries,
    }

    with open(json_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)

    field_names = [
        "trial_name",
        "status",
        "passes_rmse_threshold",
        "rmse_threshold_m",
        "rmse",
        "mae",
        "median",
        "p95",
        "max",
        "point_count",
        "above_water_point_count",
        "above_water_rmse",
        "below_water_point_count",
        "below_water_rmse",
        "post_fusion_point_count",
        "post_fusion_rmse",
        "post_fusion_p95",
        "post_fusion_compression_ratio",
        "reconstructed_point_count",
        "reconstructed_rmse",
        "reconstructed_p95",
        "reconstructed_compression_ratio",
        "grid_reconstructed_point_count",
        "grid_reconstructed_rmse",
        "grid_reconstructed_p95",
        "grid_reconstructed_compression_ratio",
        "corrected_rmse",
        "corrected_p95",
        "corrected_grid_point_count",
        "corrected_grid_rmse",
        "corrected_grid_p95",
        "corrected_grid_compression_ratio",
        "blind_corrected_rmse",
        "blind_corrected_p95",
        "blind_post_fusion_point_count",
        "blind_post_fusion_rmse",
        "blind_post_fusion_p95",
        "blind_post_fusion_compression_ratio",
        "blind_corrected_grid_point_count",
        "blind_corrected_grid_rmse",
        "blind_corrected_grid_p95",
        "blind_corrected_grid_compression_ratio",
        "degraded_rmse",
        "error_reduction_pct",
        "meets_80pct_reduction",
        "degraded_surface_rmse",
        "best_post_fusion_stage",
        "to4_official_stage",
        "to4_post_fusion_rmse",
        "to4_error_reduction_pct",
        "to4_positional_error_reduction_pct",
        "to4_passes_post_fusion_rmse_target",
        "to4_passes_all",
        "positional_degradation_magnitude_m",
        "meets_positional_baseline_2m",
        "meets_degraded_baseline_2m",
        "peak_frame_degraded_rmse",
        "peak_frame_error_reduction_pct",
        "peak_frame_meets_80pct_reduction",
        "ks_statistic",
        "ks_p_value",
        "passes_ks_threshold",
        "max_bin_deviation",
        "passes_bin_deviation_threshold",
        "passes_distribution_all",
        "wall_time_seconds",
        "wall_time_minutes",
        "peak_memory_gb",
        "passes_time_threshold",
        "passes_memory_threshold",
        "primary_rejected_hits",
        "secondary_rejected_hits",
        "gnss_drift_m",
        "sea_state_amplitude_m",
        "warning_count",
        "lidar_hit_count",
        "sonar_hit_count",
        "sensor_balance",
        "primary_location",
        "secondary_location",
        "report_path",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(summaries)

    return {"json": json_path, "csv": csv_path}


def run_trial(blender_path, scene_path, trial, script_path):
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as temp_file:
        json.dump(trial, temp_file, indent=2)
        temp_path = temp_file.name

    try:
        command = [
            blender_path,
            scene_path,
            "--background",
            "--python",
            script_path,
            "--",
            "--config",
            temp_path,
        ]
        subprocess.run(command, check=True)
    finally:
        os.unlink(temp_path)


def main():
    args = parse_args()
    batch_config = load_batch_config(args.config)
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "run_fixed_scene_validation_trial.py",
    )

    summaries = []

    for trial in batch_config["trials"]:
        prepared_trial = build_trial(batch_config, trial)
        print("Running trial '%s'..." % prepared_trial["trial_name"])
        run_trial(args.blender, args.scene, prepared_trial, script_path)

        report_path = get_expected_report_path(args.scene, prepared_trial)
        report = load_report(report_path)
        summaries.append(summarize_trial(prepared_trial, report_path, report))

    summary_dir = args.summary_dir
    if summary_dir is None:
        summary_dir = resolve_output_dir(
            args.scene,
            batch_config.get("common_defaults", {}).get("dataFilePath", "//output"),
        )

    summary_paths = write_summary(summary_dir, args.scene, summaries)
    print("Batch summary JSON: %s" % summary_paths["json"])
    print("Batch summary CSV: %s" % summary_paths["csv"])


if __name__ == "__main__":
    main()
