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
        description="Generate or reuse multiple seeded scenes and run combined pose/noise validation across them."
    )
    parser.add_argument("--blender", default="blender", help="Blender executable path.")
    parser.add_argument(
        "--config", required=True, help="Path to the multi-seed validation config."
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


def scene_name(seed_value):
    return "dual_sensor_scene_%d" % seed_value


def generate_scene(blender_path, output_dir, seed_value):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "generate_seeded_dual_sensor_scene.py",
    )
    subprocess.run(
        [
            blender_path,
            "--background",
            "--python",
            script_path,
            "--",
            "--seed",
            str(seed_value),
            "--output-dir",
            output_dir,
            "--scene-name",
            scene_name(seed_value),
        ],
        check=True,
    )


def run_combined_validation(blender_path, scene_path, config_payload):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "run_pose_then_noise_validation.py",
    )
    temp_config = write_json_temp(config_payload)

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
                temp_config,
            ],
            check=True,
        )
    finally:
        os.unlink(temp_config)


def build_scene_config(base_config, seed_value):
    config = json.loads(json.dumps(base_config))
    seed_label = "seed_%d" % seed_value

    pose_dir = config["pose_sweep"]["common_defaults"].get("dataFilePath", "//output")
    noise_dir = config["noise_sweep"]["common_defaults"].get("dataFilePath", "//output")
    combined_dir = config.get("combined_summary_dir", "//output")

    config["pose_sweep"]["common_defaults"]["dataFilePath"] = os.path.join(
        pose_dir, seed_label
    )
    config["noise_sweep"]["common_defaults"]["dataFilePath"] = os.path.join(
        noise_dir, seed_label
    )
    config["noise_sweep"]["common_defaults"].setdefault("noiseSeed", seed_value)
    config["combined_summary_dir"] = os.path.join(combined_dir, seed_label)
    return config


def resolve_scene_relative(scene_path, path_value):
    if path_value.startswith("//"):
        return os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(scene_path)), path_value[2:])
        )
    return os.path.abspath(path_value)


def combined_summary_path(scene_path, combined_summary_dir):
    scene_base = os.path.splitext(os.path.basename(scene_path))[0]
    return os.path.join(
        resolve_scene_relative(scene_path, combined_summary_dir),
        "%s_pose_then_noise_summary.json" % scene_base,
    )


def aggregate_seed_result(seed_value, combined_summary):
    selected_pose = combined_summary["selected_poses"][0]
    noise_acceptance = combined_summary["noise_sweep_summaries"][0]["acceptance"]

    return {
        "seed": seed_value,
        "selected_pose": selected_pose["trial_name"],
        "selected_pose_point_count": selected_pose.get("point_count"),
        "selected_pose_rmse": selected_pose.get("rmse"),
        "passes_all_noise_trials": noise_acceptance["pass_counts"]["all"]
        == noise_acceptance["trial_count"],
        "noise_trial_count": noise_acceptance["trial_count"],
        "noise_all_gate_pass_count": noise_acceptance["pass_counts"]["all"],
        "noise_rmse_pass_count": noise_acceptance["pass_counts"]["rmse"],
        "noise_p95_pass_count": noise_acceptance["pass_counts"]["p95"],
        "noise_max_pass_count": noise_acceptance["pass_counts"]["max"],
        "combined_summary_path": combined_summary["combined_summary_path"],
    }


def write_aggregate_summary(output_dir, results):
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "multi_seed_pose_then_noise_summary.json")
    csv_path = os.path.join(output_dir, "multi_seed_pose_then_noise_summary.csv")

    payload = {
        "seed_count": len(results),
        "results": results,
    }
    with open(json_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)

    field_names = [
        "seed",
        "selected_pose",
        "selected_pose_point_count",
        "selected_pose_rmse",
        "passes_all_noise_trials",
        "noise_trial_count",
        "noise_all_gate_pass_count",
        "noise_rmse_pass_count",
        "noise_p95_pass_count",
        "noise_max_pass_count",
        "combined_summary_path",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(results)

    return {"json": json_path, "csv": csv_path}


def main():
    args = parse_args()
    config = load_json(args.config)

    scenes_dir = os.path.abspath(config["scenes_output_dir"])
    seeds = config["seeds"]
    aggregate_results = []

    os.makedirs(scenes_dir, exist_ok=True)

    for seed_value in seeds:
        if config.get("generate_scenes", True):
            print("Generating seed %d scene..." % seed_value)
            generate_scene(args.blender, scenes_dir, seed_value)

        scene_path = os.path.join(scenes_dir, "%s.blend" % scene_name(seed_value))
        scene_config = build_scene_config(config["combined_validation"], seed_value)
        print("Running combined validation for seed %d..." % seed_value)
        run_combined_validation(args.blender, scene_path, scene_config)

        combined_summary = load_json(
            combined_summary_path(scene_path, scene_config["combined_summary_dir"])
        )
        combined_summary["combined_summary_path"] = combined_summary_path(
            scene_path, scene_config["combined_summary_dir"]
        )
        aggregate_results.append(aggregate_seed_result(seed_value, combined_summary))

    aggregate_dir = os.path.abspath(config["aggregate_summary_dir"])
    summary_paths = write_aggregate_summary(aggregate_dir, aggregate_results)
    print("Aggregate summary JSON: %s" % summary_paths["json"])
    print("Aggregate summary CSV: %s" % summary_paths["csv"])


if __name__ == "__main__":
    main()
