#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import bpy

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from scan_ready_blend_utils import apply_scan_ready_config, example_scene_config


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Bake scan-ready scanner properties into a .blend file."
    )
    parser.add_argument(
        "--blend",
        required=True,
        help="Path to the .blend file to update in place.",
    )
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    blend_path = Path(args.blend).resolve()

    bpy.ops.wm.open_mainfile(filepath=str(blend_path))
    config = example_scene_config(bpy.context.scene, blend_path)
    properties = apply_scan_ready_config(bpy.context.scene, config)
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))

    print("Baked scan-ready config into %s" % blend_path)
    print(
        "Primary=%s (%s), secondary=%s (%s), frames=%d-%d step=%d"
        % (
            properties.scannerObject.name if properties.scannerObject else None,
            properties.scannerType,
            properties.scannerObject2.name if properties.scannerObject2 else None,
            properties.scannerType2,
            properties.frameStart,
            properties.frameEnd,
            properties.frameStep,
        )
    )


if __name__ == "__main__":
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    main(argv)
