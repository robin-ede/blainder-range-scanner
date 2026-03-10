#!/usr/bin/env python3

import argparse
import os
import subprocess


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate multiple seeded dual-sensor scenes in fresh Blender processes."
    )
    parser.add_argument("--blender", default="blender", help="Blender executable path.")
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory where generated scenes are saved.",
    )
    parser.add_argument(
        "--seeds",
        nargs="+",
        type=int,
        required=True,
        help="List of integer seeds to generate.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "generate_seeded_dual_sensor_scene.py",
    )

    os.makedirs(args.output_dir, exist_ok=True)

    for seed_value in args.seeds:
        scene_name = "dual_sensor_scene_%d" % seed_value
        command = [
            args.blender,
            "--background",
            "--python",
            script_path,
            "--",
            "--seed",
            str(seed_value),
            "--output-dir",
            args.output_dir,
            "--scene-name",
            scene_name,
        ]
        print("Generating scene for seed %d..." % seed_value)
        subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
