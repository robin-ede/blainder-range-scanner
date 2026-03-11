#!/usr/bin/env python3

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create deterministic animated path scenes, run accumulated validation, and aggregate results."
    )
    parser.add_argument("--blender", default="blender", help="Blender executable path.")
    parser.add_argument("--base-scene", required=True, help="Base .blend scene path.")
    parser.add_argument(
        "--config", required=True, help="Animated path validation config."
    )
    return parser.parse_args()


def load_json(path):
    with open(path, "r", encoding="utf-8") as input_file:
        return json.load(input_file)


def write_temp_json(payload):
    temp_file = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    try:
        json.dump(payload, temp_file, indent=2)
        temp_file.close()
        return temp_file.name
    except Exception:
        temp_file.close()
        os.unlink(temp_file.name)
        raise


def resolve_relative(base_scene, path_value):
    if path_value.startswith("//"):
        return os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(base_scene)), path_value[2:])
        )
    return os.path.abspath(path_value)


def create_animated_scene(blender_path, base_scene, path_definition, output_scene):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "create_animated_validation_scene.py",
    )
    path_config = write_temp_json(path_definition)
    command = [
        blender_path,
        base_scene,
        "--background",
        "--python",
        script_path,
        "--",
        "--output-scene",
        output_scene,
        "--path-config",
        path_config,
    ]
    try:
        subprocess.run(command, check=True)
    finally:
        os.unlink(path_config)


def path_score(summary_item, selection):
    weights = selection.get("weights", {})
    score = 0.0
    score += weights.get("point_count", 0.0) * (summary_item.get("point_count") or 0.0)
    score += weights.get("below_water_point_count", 0.0) * (
        summary_item.get("below_water_point_count") or 0.0
    )
    score += weights.get("sonar_hit_count", 0.0) * (
        summary_item.get("sonar_hit_count") or 0.0
    )
    score += weights.get("sensor_balance", 0.0) * (
        summary_item.get("sensor_balance") or 0.0
    )

    if summary_item.get("rmse") is not None:
        score -= weights.get("rmse", 0.0) * summary_item["rmse"]
    if summary_item.get("p95") is not None:
        score -= weights.get("p95", 0.0) * summary_item["p95"]
    if summary_item.get("max") is not None:
        score -= weights.get("max", 0.0) * summary_item["max"]
    if summary_item.get("post_fusion_rmse") is not None:
        score -= weights.get("post_fusion_rmse", 0.0) * summary_item["post_fusion_rmse"]
    if summary_item.get("post_fusion_p95") is not None:
        score -= weights.get("post_fusion_p95", 0.0) * summary_item["post_fusion_p95"]
    if summary_item.get("reconstructed_rmse") is not None:
        score -= (
            weights.get("reconstructed_rmse", 0.0) * summary_item["reconstructed_rmse"]
        )
    if summary_item.get("reconstructed_p95") is not None:
        score -= (
            weights.get("reconstructed_p95", 0.0) * summary_item["reconstructed_p95"]
        )
    if summary_item.get("grid_reconstructed_rmse") is not None:
        score -= (
            weights.get("grid_reconstructed_rmse", 0.0)
            * summary_item["grid_reconstructed_rmse"]
        )
    if summary_item.get("grid_reconstructed_p95") is not None:
        score -= (
            weights.get("grid_reconstructed_p95", 0.0)
            * summary_item["grid_reconstructed_p95"]
        )
    if summary_item.get("corrected_rmse") is not None:
        score -= weights.get("corrected_rmse", 0.0) * summary_item["corrected_rmse"]
    if summary_item.get("corrected_p95") is not None:
        score -= weights.get("corrected_p95", 0.0) * summary_item["corrected_p95"]
    if summary_item.get("corrected_grid_rmse") is not None:
        score -= (
            weights.get("corrected_grid_rmse", 0.0)
            * summary_item["corrected_grid_rmse"]
        )
    if summary_item.get("corrected_grid_p95") is not None:
        score -= (
            weights.get("corrected_grid_p95", 0.0) * summary_item["corrected_grid_p95"]
        )
    if summary_item.get("blind_corrected_rmse") is not None:
        score -= (
            weights.get("blind_corrected_rmse", 0.0)
            * summary_item["blind_corrected_rmse"]
        )
    if summary_item.get("blind_post_fusion_rmse") is not None:
        score -= (
            weights.get("blind_post_fusion_rmse", 0.0)
            * summary_item["blind_post_fusion_rmse"]
        )
    if summary_item.get("blind_corrected_grid_rmse") is not None:
        score -= (
            weights.get("blind_corrected_grid_rmse", 0.0)
            * summary_item["blind_corrected_grid_rmse"]
        )
    if summary_item.get("to4_post_fusion_rmse") is not None:
        score -= (
            weights.get("to4_post_fusion_rmse", 0.0)
            * summary_item["to4_post_fusion_rmse"]
        )

    return score


def select_representative_trial(path_name, acceptance, selection):
    preferred_trial = selection.get("preferred_trial_name")
    trials = acceptance["trials"]

    if preferred_trial is not None:
        for trial in trials:
            if trial["trial_name"] == preferred_trial:
                selected = dict(trial)
                selected["path_name"] = path_name
                selected["path_score"] = path_score(selected, selection)
                return selected

    selected = max(trials, key=lambda trial: path_score(trial, selection))
    selected = dict(selected)
    selected["path_name"] = path_name
    selected["path_score"] = path_score(selected, selection)
    return selected


def run_validation_batch(blender_path, scene_path, validation_config):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "run_fixed_scene_validation_batch.py",
    )
    config_path = write_temp_json(validation_config)
    try:
        subprocess.run(
            [
                sys.executable,
                script_path,
                "--blender",
                blender_path,
                "--scene",
                scene_path,
                "--config",
                config_path,
            ],
            check=True,
        )
    finally:
        os.unlink(config_path)


def validation_summary_path(scene_path, output_dir):
    scene_name = os.path.splitext(os.path.basename(scene_path))[0]
    return os.path.join(output_dir, "%s_validation_summary.json" % scene_name)


def evaluate_acceptance(summary, thresholds):
    rows = []
    pass_counts = {"rmse": 0, "p95": 0, "max": 0, "all": 0}

    for trial in summary["trials"]:
        rmse = trial.get("rmse")
        p95 = trial.get("p95")
        max_error = trial.get("max")

        passes_rmse = rmse is not None and rmse <= thresholds["rmse_threshold_m"]
        passes_p95 = p95 is not None and p95 <= thresholds["p95_threshold_m"]
        passes_max = True
        if thresholds.get("max_threshold_m") is not None:
            passes_max = (
                max_error is not None and max_error <= thresholds["max_threshold_m"]
            )
        passes_all = passes_rmse and passes_p95 and passes_max

        if passes_rmse:
            pass_counts["rmse"] += 1
        if passes_p95:
            pass_counts["p95"] += 1
        if passes_max:
            pass_counts["max"] += 1
        if passes_all:
            pass_counts["all"] += 1

        enriched = dict(trial)
        enriched["passes_rmse_gate"] = passes_rmse
        enriched["passes_p95_gate"] = passes_p95
        enriched["passes_max_gate"] = passes_max
        enriched["passes_all_gates"] = passes_all
        rows.append(enriched)

    return {"thresholds": thresholds, "pass_counts": pass_counts, "trials": rows}


def build_validation_config(base_config, path_name, frame_start, frame_end):
    config = json.loads(json.dumps(base_config))
    config["common_defaults"] = dict(config.get("common_defaults", {}))
    output_dir = config["common_defaults"].get("dataFilePath", "//output")
    config["common_defaults"]["dataFilePath"] = os.path.join(output_dir, path_name)
    config["common_defaults"]["frameStart"] = frame_start
    config["common_defaults"]["frameEnd"] = frame_end
    return config


def build_aggregate_rows(path_summaries, selection):
    aggregate_rows = []
    for path_summary in path_summaries:
        for trial in path_summary["acceptance"]["trials"]:
            row = dict(trial)
            row["path_name"] = path_summary["path_name"]
            row["path_score"] = path_score(trial, selection)
            aggregate_rows.append(row)
    return aggregate_rows


def write_aggregate(output_dir, path_summaries, selection):
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "animated_path_validation_summary.json")
    csv_path = os.path.join(output_dir, "animated_path_validation_summary.csv")

    best_path = None
    if path_summaries:
        best_path = max(
            path_summaries, key=lambda item: item["selected_trial"]["path_score"]
        )
        best_path = {
            "path_name": best_path["path_name"],
            "selected_trial": best_path["selected_trial"],
        }

    payload = {
        "path_count": len(path_summaries),
        "best_path": best_path,
        "paths": path_summaries,
    }
    with open(json_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)

    aggregate_rows = build_aggregate_rows(path_summaries, selection)

    field_names = [
        "path_name",
        "trial_name",
        "status",
        "rmse",
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
        "meets_degraded_baseline_2m",
        "best_post_fusion_stage",
        "to4_official_stage",
        "to4_post_fusion_rmse",
        "to4_error_reduction_pct",
        "to4_positional_error_reduction_pct",
        "to4_passes_post_fusion_rmse_target",
        "to4_passes_all",
        "ks_statistic",
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
        "warning_count",
        "lidar_hit_count",
        "sonar_hit_count",
        "sensor_balance",
        "passes_rmse_gate",
        "passes_p95_gate",
        "passes_max_gate",
        "passes_all_gates",
        "path_score",
        "report_path",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(field_names)
        for row in aggregate_rows:
            writer.writerow([row.get(field_name) for field_name in field_names])

    return {"json": json_path, "csv": csv_path}


def main():
    args = parse_args()
    config = load_json(args.config)

    animated_scene_dir = resolve_relative(args.base_scene, config["animated_scene_dir"])
    aggregate_dir = resolve_relative(args.base_scene, config["aggregate_summary_dir"])
    thresholds = config.get(
        "acceptance",
        {"rmse_threshold_m": 0.15, "p95_threshold_m": 0.15, "max_threshold_m": 0.45},
    )
    selection = config.get(
        "selection",
        {
            "preferred_trial_name": "animated_noise_sigma_005",
            "weights": {
                "point_count": 1.0,
                "sonar_hit_count": 4.0,
                "sensor_balance": 10000.0,
                "rmse": 5000.0,
                "p95": 1000.0,
                "max": 100.0,
            },
        },
    )

    os.makedirs(animated_scene_dir, exist_ok=True)

    path_summaries = []

    for path_definition in config["paths"]:
        path_name = path_definition["path_name"]
        animated_scene = os.path.join(animated_scene_dir, "%s.blend" % path_name)
        create_animated_scene(
            args.blender, args.base_scene, path_definition, animated_scene
        )

        validation_config = build_validation_config(
            config["validation_batch"],
            path_name,
            path_definition["frame_start"],
            path_definition["frame_end"],
        )
        run_validation_batch(args.blender, animated_scene, validation_config)

        summary_dir = resolve_relative(
            animated_scene, validation_config["common_defaults"]["dataFilePath"]
        )
        summary_path = validation_summary_path(animated_scene, summary_dir)
        summary = load_json(summary_path)
        acceptance = evaluate_acceptance(summary, thresholds)

        path_summaries.append(
            {
                "path_name": path_name,
                "animated_scene": animated_scene,
                "summary_path": summary_path,
                "acceptance": acceptance,
                "selected_trial": select_representative_trial(
                    path_name, acceptance, selection
                ),
            }
        )

    output_paths = write_aggregate(aggregate_dir, path_summaries, selection)
    print("Animated path aggregate JSON: %s" % output_paths["json"])
    print("Animated path aggregate CSV: %s" % output_paths["csv"])


if __name__ == "__main__":
    main()
