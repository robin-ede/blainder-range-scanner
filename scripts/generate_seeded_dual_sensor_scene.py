#!/usr/bin/env python3

import argparse
import json
import math
import os
import random
import sys

import bpy
from mathutils import Vector


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Generate a seeded random scene for dual-sensor validation."
    )
    parser.add_argument("--seed", required=True, type=int, help="Scene seed.")
    parser.add_argument(
        "--output-dir", required=True, help="Directory where the .blend scene is saved."
    )
    parser.add_argument(
        "--scene-name",
        default=None,
        help="Optional base filename without extension. Defaults to dual_sensor_scene_<seed>.",
    )
    return parser.parse_args(argv)


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def create_material(name, color):
    material = bpy.data.materials.new(name=name)
    material.diffuse_color = color
    return material


def look_at(obj, target):
    direction = target - obj.location
    rotation = direction.to_track_quat("-Z", "Y")
    obj.rotation_euler = rotation.to_euler()


def add_camera(name, location, target, lens=24.0):
    camera_data = bpy.data.cameras.new(name)
    camera = bpy.data.objects.new(name, camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = location
    camera.data.lens = lens
    look_at(camera, target)
    return camera


def add_light():
    light_data = bpy.data.lights.new(name="Sun", type="SUN")
    light = bpy.data.objects.new(name="Sun", object_data=light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = (0.0, 0.0, 12.0)
    light.rotation_euler = (math.radians(35.0), 0.0, math.radians(20.0))


def add_water_surface(size):
    bpy.ops.mesh.primitive_plane_add(size=size, location=(0.0, 0.0, 0.0))
    water = bpy.context.object
    water.name = "WaterSurface"
    water.display_type = "WIRE"
    return water


def add_seafloor(size, material):
    bpy.ops.mesh.primitive_plane_add(size=size, location=(0.0, 0.0, -4.0))
    seafloor = bpy.context.object
    seafloor.name = "Seafloor"
    seafloor.scale = (1.0, 1.0, 0.35)
    seafloor.data.materials.append(material)
    return seafloor


def add_pier(material):
    bpy.ops.mesh.primitive_cube_add(location=(-2.0, 0.0, 1.2))
    pier = bpy.context.object
    pier.name = "Pier"
    pier.scale = (4.0, 1.5, 0.3)
    pier.data.materials.append(material)
    return pier


def add_random_object(rng, name, region, material, underwater):
    primitive = rng.choice(["cube", "sphere", "cylinder", "cone"])
    location = (
        rng.uniform(region["x"][0], region["x"][1]),
        rng.uniform(region["y"][0], region["y"][1]),
        rng.uniform(region["z"][0], region["z"][1]),
    )

    if primitive == "cube":
        bpy.ops.mesh.primitive_cube_add(location=location)
    elif primitive == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(location=location)
    elif primitive == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(location=location)
    else:
        bpy.ops.mesh.primitive_cone_add(location=location)

    obj = bpy.context.object
    obj.name = name
    obj.scale = (
        rng.uniform(0.4, 1.8),
        rng.uniform(0.4, 1.8),
        rng.uniform(0.4, 1.8 if underwater else 2.2),
    )
    obj.rotation_euler = (
        rng.uniform(0.0, math.pi),
        rng.uniform(0.0, math.pi),
        rng.uniform(0.0, math.pi),
    )
    obj.data.materials.append(material)
    return obj


def generate_scene(seed_value):
    rng = random.Random(seed_value)
    clear_scene()

    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.render.engine = "BLENDER_EEVEE"

    add_light()

    seafloor_material = create_material("SeafloorMaterial", (0.28, 0.19, 0.1, 1.0))
    underwater_material = create_material(
        "UnderwaterTargetMaterial", (0.14, 0.38, 0.52, 1.0)
    )
    above_water_material = create_material(
        "AboveWaterTargetMaterial", (0.62, 0.41, 0.18, 1.0)
    )
    pier_material = create_material("PierMaterial", (0.35, 0.28, 0.19, 1.0))

    add_water_surface(40.0)
    add_seafloor(40.0, seafloor_material)
    add_pier(pier_material)

    underwater_region = {"x": (-2.0, 10.0), "y": (-6.0, 6.0), "z": (-3.4, -0.6)}
    above_water_region = {"x": (-1.0, 10.0), "y": (-6.0, 6.0), "z": (0.6, 4.0)}

    underwater_count = rng.randint(6, 10)
    above_water_count = rng.randint(4, 7)

    generated_names = []
    for index in range(underwater_count):
        obj = add_random_object(
            rng,
            "UnderwaterTarget_%02d" % index,
            underwater_region,
            underwater_material,
            underwater=True,
        )
        generated_names.append(obj.name)

    for index in range(above_water_count):
        obj = add_random_object(
            rng,
            "AboveWaterTarget_%02d" % index,
            above_water_region,
            above_water_material,
            underwater=False,
        )
        generated_names.append(obj.name)

    target = Vector((3.5, 0.0, -0.8))
    add_camera("Camera", Vector((-12.0, 0.0, 6.5)), target, lens=20.0)
    add_camera("Camera.001", Vector((-8.0, 0.0, -2.4)), target, lens=22.0)

    bpy.context.scene.frame_set(1)

    return {
        "seed": seed_value,
        "water_surface_z": 0.0,
        "primary_camera": "Camera",
        "secondary_camera": "Camera.001",
        "generated_target_count": len(generated_names),
        "generated_targets": generated_names,
    }


def save_outputs(scene_name, output_dir, metadata):
    os.makedirs(output_dir, exist_ok=True)
    blend_path = os.path.join(output_dir, "%s.blend" % scene_name)
    metadata_path = os.path.join(output_dir, "%s.metadata.json" % scene_name)

    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    with open(metadata_path, "w", encoding="utf-8") as output_file:
        json.dump(metadata, output_file, indent=2)

    return blend_path, metadata_path


def main(argv):
    args = parse_args(argv)
    scene_name = args.scene_name or ("dual_sensor_scene_%d" % args.seed)
    metadata = generate_scene(args.seed)
    blend_path, metadata_path = save_outputs(scene_name, args.output_dir, metadata)
    print("Generated scene: %s" % blend_path)
    print("Scene metadata: %s" % metadata_path)


if __name__ == "__main__":
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    main(argv)
