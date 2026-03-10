import argparse
import json
import os
import sys

import bpy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from path_generation import bounds_from_blender_object, generate_path_from_bounds


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--target-object", default="Seabed")
    parser.add_argument(
        "--mode", choices=["straight", "lawnmower"], default="lawnmower"
    )
    parser.add_argument("--frame-start", type=int, default=1)
    parser.add_argument("--frame-end", type=int)
    parser.add_argument("--margin-m", type=float, default=15.0)
    parser.add_argument("--line-spacing-m", type=float)
    parser.add_argument("--swath-width-m", type=float, default=60.0)
    parser.add_argument("--overlap-ratio", type=float, default=0.3)
    parser.add_argument("--speed-mps", type=float, default=15.0)
    parser.add_argument("--frame-rate", type=float, default=10.0)
    parser.add_argument("--primary-height-m", type=float, default=8.0)
    parser.add_argument("--secondary-clearance-m", type=float, default=1.5)
    parser.add_argument("--secondary-min-depth-m", type=float, default=-1.2)
    args = parser.parse_args(argv)

    obj = bpy.context.scene.objects.get(args.target_object)
    if obj is None:
        raise RuntimeError("Target object '%s' not found" % args.target_object)

    generator = {
        "mode": args.mode,
        "target_object": args.target_object,
        "margin_m": args.margin_m,
        "line_spacing_m": args.line_spacing_m,
        "swath_width_m": args.swath_width_m,
        "overlap_ratio": args.overlap_ratio,
        "speed_mps": args.speed_mps,
        "frame_rate": args.frame_rate,
        "primary_height_m": args.primary_height_m,
        "secondary_clearance_m": args.secondary_clearance_m,
        "secondary_min_depth_m": args.secondary_min_depth_m,
    }
    path_definition = generate_path_from_bounds(
        bounds_from_blender_object(obj), generator, args.frame_start, args.frame_end
    )
    path_definition["path_generator"] = generator

    with open(args.output, "w", encoding="utf-8") as output_file:
        json.dump(path_definition, output_file, indent=2)
    print("Wrote generated path config to %s" % args.output)


if __name__ == "__main__":
    main()
