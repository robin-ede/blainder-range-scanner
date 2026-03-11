#!/usr/bin/env python3

import argparse
import json
import os
import sys

import bpy

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import range_scanner


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run one fixed-scene multi-sensor validation trial."
    )
    parser.add_argument("--config", required=True, help="Path to a trial JSON file.")
    return parser.parse_args(argv)


def load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as input_file:
        return json.load(input_file)


def ensure_output_dir(trial):
    output_dir = trial.get("common", {}).get("dataFilePath")
    if output_dir:
        os.makedirs(bpy.path.abspath(output_dir), exist_ok=True)


def main(argv):
    args = parse_args(argv)
    trial = load_config(args.config)

    ensure_output_dir(trial)

    result = range_scanner.ui.user_interface.scan_multi_sensor(
        bpy.context,
        primary_config=trial["primary"],
        secondary_config=trial["secondary"],
        common_config=trial.get("common", {}),
        water_profile=trial.get("water_profile"),
        primary_pose=trial.get("primary_pose"),
        secondary_pose=trial.get("secondary_pose"),
    )

    if result.get("result") == {"CANCELLED"}:
        raise RuntimeError(
            "Trial '%s' was cancelled during scanner validation."
            % trial.get("trial_name", "unnamed_trial")
        )

    report_path = result["report_path"]
    print("Trial '%s' complete." % trial.get("trial_name", "unnamed_trial"))
    print("Validation report: %s" % report_path)

    if not os.path.exists(report_path):
        raise RuntimeError(
            "Expected validation report was not written: %s" % report_path
        )

    with open(report_path, "r", encoding="utf-8") as input_file:
        report = json.load(input_file)

    metrics = report.get("pre_fusion_combined_metrics", {})
    fused_metrics = report.get("post_processing_fusion", {}).get("metrics", {})
    reconstructed_metrics = report.get("medium_separated_reconstruction", {}).get(
        "metrics", {}
    )
    grid_metrics = report.get("medium_height_grid_reconstruction", {}).get(
        "metrics", {}
    )
    corrected_metrics = report.get("alignment_correction", {}).get("metrics", {})
    corrected_grid_metrics = report.get("corrected_grid_reconstruction", {}).get(
        "metrics", {}
    )
    blind_corrected_metrics = report.get("blind_alignment_correction", {}).get(
        "metrics", {}
    )
    blind_fused_metrics = report.get("blind_post_processing_fusion", {}).get(
        "metrics", {}
    )
    blind_corrected_grid_metrics = report.get(
        "blind_corrected_grid_reconstruction", {}
    ).get("metrics", {})
    error_reduction = report.get("error_reduction", {})
    to4 = report.get("technical_objectives", {}).get("TO4", {})
    print(
        "Pre=%s, corrected=%s, blind=%s, blind_fused=%s, post=%s, stitched=%s, "
        "grid=%s, corrected_grid=%s, blind_grid=%s, TO4_post=%s, TO4_pass=%s, "
        "err_reduction=%s%%, pass=%s"
        % (
            metrics.get("rmse"),
            corrected_metrics.get("rmse"),
            blind_corrected_metrics.get("rmse"),
            blind_fused_metrics.get("rmse"),
            fused_metrics.get("rmse"),
            reconstructed_metrics.get("rmse"),
            grid_metrics.get("rmse"),
            corrected_grid_metrics.get("rmse"),
            blind_corrected_grid_metrics.get("rmse"),
            to4.get("post_fusion_rmse"),
            to4.get("passes_all"),
            error_reduction.get("error_reduction_pct"),
            report.get("acceptance", {}).get("passes_rmse_threshold"),
        )
    )


if __name__ == "__main__":
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    main(argv)
