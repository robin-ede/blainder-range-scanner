#!/usr/bin/env python3

import argparse
import json
import os
import sys

import bpy
from mathutils import Vector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from path_generation import bounds_from_blender_object, generate_path_from_bounds


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Apply deterministic dual-sensor animation to an existing scene."
    )
    parser.add_argument(
        "--output-scene", required=True, help="Path for the saved animated .blend file."
    )
    parser.add_argument("--path-config", help="Path to a JSON path definition.")
    parser.add_argument("--frame-start", type=int, default=1)
    parser.add_argument("--frame-end", type=int, default=20)
    parser.add_argument("--primary-camera", default="Camera")
    parser.add_argument("--secondary-camera", default="Camera.001")
    parser.add_argument("--look-at", nargs=3, type=float, default=[3.5, 0.0, -0.8])
    parser.add_argument(
        "--primary-start", nargs=3, type=float, default=[-12.0, 0.0, 6.5]
    )
    parser.add_argument("--primary-end", nargs=3, type=float, default=[-2.5, 0.0, 7.5])
    parser.add_argument(
        "--secondary-start", nargs=3, type=float, default=[-8.0, 0.0, -2.4]
    )
    parser.add_argument(
        "--secondary-end", nargs=3, type=float, default=[-1.5, 0.0, -2.0]
    )
    return parser.parse_args(argv)


def load_path_definition(args):
    if args.path_config:
        with open(args.path_config, "r", encoding="utf-8") as input_file:
            return json.load(input_file)

    return {
        "frame_start": args.frame_start,
        "frame_end": args.frame_end,
        "look_at": args.look_at,
        "primary_camera": args.primary_camera,
        "secondary_camera": args.secondary_camera,
        "primary_start": args.primary_start,
        "primary_end": args.primary_end,
        "secondary_start": args.secondary_start,
        "secondary_end": args.secondary_end,
    }


def resolve_generated_path(path_definition, scene):
    generator = path_definition.get("path_generator")
    if not generator:
        return path_definition

    target_object_name = generator.get("target_object", "Seabed")
    target_object = scene.objects.get(target_object_name)
    if target_object is None:
        raise ValueError(
            "Generated path target object '%s' not found." % target_object_name
        )

    frame_start = int(path_definition.get("frame_start", scene.frame_start))
    frame_end = int(path_definition.get("frame_end", scene.frame_end))
    generated = generate_path_from_bounds(
        bounds_from_blender_object(target_object), generator, frame_start, frame_end
    )
    merged = dict(path_definition)
    merged.update(generated)
    return merged


def look_at(obj, target):
    direction = Vector(target) - obj.location
    obj.rotation_mode = "XYZ"
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def clear_animation(obj):
    if obj.animation_data is not None:
        obj.animation_data_clear()


def insert_pose_keyframe(obj, frame_number, location, target):
    obj.location = Vector(location)
    look_at(obj, target)
    obj.keyframe_insert(data_path="location", frame=frame_number)
    obj.keyframe_insert(data_path="rotation_euler", frame=frame_number)


def _frame_sequence(frame_start, frame_end, count):
    if count == 1:
        return [frame_start]
    step = float(frame_end - frame_start) / float(count - 1)
    return [int(round(frame_start + index * step)) for index in range(count)]


def _waypoints_for_camera(path_definition, prefix):
    key = "%s_waypoints" % prefix
    if key in path_definition:
        return path_definition[key]
    return [path_definition["%s_start" % prefix], path_definition["%s_end" % prefix]]


def animate_camera(obj, frame_start, frame_end, waypoints, target):
    clear_animation(obj)
    frames = _frame_sequence(frame_start, frame_end, len(waypoints))
    for frame_number, waypoint in zip(frames, waypoints):
        insert_pose_keyframe(obj, frame_number, waypoint, target)


def animate_camera_with_frames(obj, waypoints, waypoint_frames, target):
    clear_animation(obj)
    for frame_number, waypoint in zip(waypoint_frames, waypoints):
        insert_pose_keyframe(obj, int(frame_number), waypoint, target)


def main(argv):
    args = parse_args(argv)
    path_definition = load_path_definition(args)

    scene = bpy.context.scene
    path_definition = resolve_generated_path(path_definition, scene)
    scene.frame_start = int(path_definition["frame_start"])
    scene.frame_end = int(path_definition["frame_end"])

    primary_name = path_definition.get("primary_camera", args.primary_camera)
    secondary_name = path_definition.get("secondary_camera", args.secondary_camera)

    primary = scene.objects.get(primary_name)
    secondary = scene.objects.get(secondary_name)
    if primary is None:
        raise ValueError("Primary camera '%s' not found." % primary_name)
    if secondary is None:
        raise ValueError("Secondary camera '%s' not found." % secondary_name)

    target = path_definition.get("look_at", args.look_at)
    primary_waypoints = _waypoints_for_camera(path_definition, "primary")
    secondary_waypoints = _waypoints_for_camera(path_definition, "secondary")
    primary_frames = path_definition.get("primary_waypoint_frames")
    secondary_frames = path_definition.get("secondary_waypoint_frames")

    if primary_frames is not None:
        animate_camera_with_frames(primary, primary_waypoints, primary_frames, target)
    else:
        animate_camera(
            primary,
            scene.frame_start,
            scene.frame_end,
            primary_waypoints,
            target,
        )

    if secondary_frames is not None:
        animate_camera_with_frames(
            secondary, secondary_waypoints, secondary_frames, target
        )
    else:
        animate_camera(
            secondary,
            scene.frame_start,
            scene.frame_end,
            secondary_waypoints,
            target,
        )

    output_scene = os.path.abspath(args.output_scene)
    os.makedirs(os.path.dirname(output_scene), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=output_scene)
    print("Animated validation scene saved to %s" % output_scene)


if __name__ == "__main__":
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    main(argv)
