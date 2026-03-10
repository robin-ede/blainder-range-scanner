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
        description="Run a pose sweep, pick the best poses, then run noise sweeps on them."
    )
    parser.add_argument("--blender", default="blender", help="Blender executable path.")
    parser.add_argument(
        "--scene", required=True, help="Path to the fixed .blend scene."
    )
    parser.add_argument(
        "--config", required=True, help="Path to the combined JSON config."
    )
    return parser.parse_args()


def load_json(path):
    with open(path, "r", encoding="utf-8") as input_file:
        return json.load(input_file)


def write_json_temp(payload):
    temp_file = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    try:
        json.dump(payload, temp_file, indent=2)
        temp_file.close()
        return temp_file.name
    except Exception:
        temp_file.close()
        os.unlink(temp_file.name)
        raise


def run_batch(blender_path, scene_path, config_payload):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "run_fixed_scene_validation_batch.py",
    )
    config_path = write_json_temp(config_payload)

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


def summary_path(scene_path, output_dir):
    scene_name = os.path.splitext(os.path.basename(scene_path))[0]
    return os.path.join(output_dir, "%s_validation_summary.json" % scene_name)


def sort_trials(trials, metric, descending):
    return sorted(
        trials,
        key=lambda trial: (
            trial.get(metric) if trial.get(metric) is not None else float("-inf")
        ),
        reverse=descending,
    )


def composite_pose_score(trial, selection):
    score = 0.0
    weights = selection.get("weights", {})

    point_count = trial.get("point_count") or 0.0
    sensor_balance = trial.get("sensor_balance") or 0.0
    rmse = trial.get("rmse")
    p95 = trial.get("p95")

    score += weights.get("point_count", 1.0) * point_count
    score += weights.get("sensor_balance", 0.0) * sensor_balance

    if rmse is not None:
        score -= weights.get("rmse", 0.0) * rmse
    if p95 is not None:
        score -= weights.get("p95", 0.0) * p95

    return score


def rank_pose_trials(trials, selection):
    strategy = selection.get("strategy", "metric")
    descending = bool(selection.get("descending", True))

    if strategy == "composite":
        return sorted(
            trials,
            key=lambda trial: composite_pose_score(trial, selection),
            reverse=True,
        )

    metric = selection.get("metric", "point_count")
    return sort_trials(trials, metric, descending)


def resolve_scene_relative_path(scene_path, path_value):
    if path_value.startswith("//"):
        return os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(scene_path)), path_value[2:])
        )
    return path_value


def evaluate_acceptance(noise_summary, acceptance_config):
    rmse_threshold = acceptance_config.get("rmse_threshold_m")
    p95_threshold = acceptance_config.get("p95_threshold_m")
    max_threshold = acceptance_config.get("max_threshold_m")

    enriched_trials = []
    pass_counts = {
        "rmse": 0,
        "p95": 0,
        "max": 0,
        "all": 0,
    }

    for trial in noise_summary["trials"]:
        rmse = trial.get("rmse")
        p95 = trial.get("p95")
        max_error = trial.get("max")

        passes_rmse = rmse is not None and (
            rmse_threshold is None or rmse <= rmse_threshold
        )
        passes_p95 = p95 is not None and (p95_threshold is None or p95 <= p95_threshold)
        passes_max = max_error is not None and (
            max_threshold is None or max_error <= max_threshold
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

        enriched_trial = dict(trial)
        enriched_trial["passes_rmse_gate"] = passes_rmse
        enriched_trial["passes_p95_gate"] = passes_p95
        enriched_trial["passes_max_gate"] = passes_max
        enriched_trial["passes_all_gates"] = passes_all
        enriched_trials.append(enriched_trial)

    return {
        "thresholds": {
            "rmse_threshold_m": rmse_threshold,
            "p95_threshold_m": p95_threshold,
            "max_threshold_m": max_threshold,
        },
        "trial_count": len(enriched_trials),
        "pass_counts": pass_counts,
        "trials": enriched_trials,
    }


def build_noise_config(base_config, selected_pose_trial):
    noise_config = json.loads(json.dumps(base_config))
    pose_name = selected_pose_trial["trial_name"]
    pose_report = load_json(selected_pose_trial["report_path"])
    primary_report = pose_report.get("scanner_config", {}).get("primary", {})
    secondary_report = pose_report.get("scanner_config", {}).get("secondary", {})

    noise_config["common_defaults"] = dict(noise_config.get("common_defaults", {}))
    output_dir = noise_config["common_defaults"].get("dataFilePath", "//output")
    noise_config["common_defaults"]["dataFilePath"] = os.path.join(
        output_dir, pose_name
    )
    noise_config["primary_pose_defaults"] = dict(
        noise_config.get("primary_pose_defaults", {})
    )
    noise_config["secondary_pose_defaults"] = dict(
        noise_config.get("secondary_pose_defaults", {})
    )
    noise_config["primary_pose_defaults"]["location"] = primary_report.get("location")
    noise_config["primary_pose_defaults"]["rotation_euler_rad"] = primary_report.get(
        "rotation_euler_rad"
    )
    noise_config["secondary_pose_defaults"]["location"] = secondary_report.get(
        "location"
    )
    noise_config["secondary_pose_defaults"]["rotation_euler_rad"] = (
        secondary_report.get("rotation_euler_rad")
    )
    return noise_config


def combined_rows(noise_item, selected_pose_name):
    rows = []
    for trial in noise_item["acceptance"]["trials"]:
        row = dict(trial)
        row["selected_pose"] = selected_pose_name
        rows.append(row)
    return rows


def write_combined_summary(
    scene_path, output_dir, pose_summary, selected_pose_trials, noise_summaries
):
    os.makedirs(output_dir, exist_ok=True)
    scene_name = os.path.splitext(os.path.basename(scene_path))[0]
    json_path = os.path.join(output_dir, "%s_pose_then_noise_summary.json" % scene_name)
    csv_path = os.path.join(output_dir, "%s_pose_then_noise_summary.csv" % scene_name)

    payload = {
        "scene_path": os.path.abspath(scene_path),
        "pose_selection_metric": pose_summary["selection_metric"],
        "selected_poses": selected_pose_trials,
        "pose_sweep_summary": pose_summary["summary_path"],
        "acceptance": pose_summary["acceptance"],
        "noise_sweep_summaries": noise_summaries,
    }

    with open(json_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)

    csv_rows = []
    for item in noise_summaries:
        csv_rows.extend(combined_rows(item, item["selected_pose"]))

    field_names = [
        "selected_pose",
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
        "warning_count",
        "passes_rmse_gate",
        "passes_p95_gate",
        "passes_max_gate",
        "passes_all_gates",
        "primary_location",
        "secondary_location",
        "report_path",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(csv_rows)

    return {"json": json_path, "csv": csv_path}


def main():
    args = parse_args()
    config = load_json(args.config)

    pose_config = config["pose_sweep"]
    run_batch(args.blender, args.scene, pose_config)

    pose_output_dir = pose_config.get("common_defaults", {}).get(
        "dataFilePath", "//output"
    )
    pose_output_dir = resolve_scene_relative_path(args.scene, pose_output_dir)
    pose_summary_file = summary_path(args.scene, pose_output_dir)
    pose_summary_json = load_json(pose_summary_file)

    selection = config.get("selection", {})
    metric = selection.get("metric", "point_count")
    top_k = int(selection.get("top_k", 1))
    ordered_pose_trials = rank_pose_trials(pose_summary_json["trials"], selection)
    selected_pose_trials = ordered_pose_trials[:top_k]

    acceptance_config = config.get(
        "acceptance",
        {
            "rmse_threshold_m": 0.15,
            "p95_threshold_m": 0.15,
            "max_threshold_m": None,
        },
    )

    noise_summaries = []
    for pose_trial in selected_pose_trials:
        noise_config = build_noise_config(config["noise_sweep"], pose_trial)
        run_batch(args.blender, args.scene, noise_config)

        noise_output_dir = noise_config.get("common_defaults", {}).get(
            "dataFilePath", "//output"
        )
        noise_output_dir = resolve_scene_relative_path(args.scene, noise_output_dir)

        noise_summary_file = summary_path(args.scene, noise_output_dir)
        noise_summary_json = load_json(noise_summary_file)
        noise_summaries.append(
            {
                "selected_pose": pose_trial["trial_name"],
                "summary_path": noise_summary_file,
                "summary": noise_summary_json,
                "acceptance": evaluate_acceptance(
                    noise_summary_json, acceptance_config
                ),
            }
        )

    combined_output_dir = config.get("combined_summary_dir")
    if combined_output_dir is None:
        combined_output_dir = pose_output_dir
    else:
        combined_output_dir = resolve_scene_relative_path(
            args.scene, combined_output_dir
        )

    combined_paths = write_combined_summary(
        args.scene,
        combined_output_dir,
        {
            "selection_metric": metric,
            "summary_path": pose_summary_file,
            "acceptance": acceptance_config,
        },
        selected_pose_trials,
        noise_summaries,
    )

    print("Pose sweep summary: %s" % pose_summary_file)
    for item in noise_summaries:
        print("Noise sweep for %s: %s" % (item["selected_pose"], item["summary_path"]))
    print("Combined summary JSON: %s" % combined_paths["json"])
    print("Combined summary CSV: %s" % combined_paths["csv"])


if __name__ == "__main__":
    main()
