import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from scan_ready_blend_utils import apply_scan_ready_config, build_dual_sensor_config


def look_at(obj, target):
    direction = target - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def ensure_material(obj, name, base_color, alpha=1.0):
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (*base_color, 1.0)
        bsdf.inputs["Alpha"].default_value = alpha
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    bpy.ops.wm.open_mainfile(filepath=args.input)

    mesh_objs = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not mesh_objs:
        raise RuntimeError("No mesh found in input scene")
    seabed = mesh_objs[0]
    seabed.name = "Seabed"
    ensure_material(seabed, "SeabedMaterial", (0.35, 0.32, 0.24))

    bpy.context.view_layer.objects.active = seabed
    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="BOUNDS")
    xs = [(seabed.matrix_world @ v.co).x for v in seabed.data.vertices]
    ys = [(seabed.matrix_world @ v.co).y for v in seabed.data.vertices]
    zs = [(seabed.matrix_world @ v.co).z for v in seabed.data.vertices]
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0
    seabed_top = max(zs)
    seabed_bottom = min(zs)
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    footprint_span = max(span_x, span_y)

    sensor_standoff = min(max(footprint_span * 0.8, 8.0), 80.0)
    primary_height = min(max(footprint_span * 0.2, 6.0), 30.0)
    look_target = Vector((center_x, center_y, (seabed_top + seabed_bottom) / 2.0))

    # Water surface plane at z=0.
    bpy.ops.mesh.primitive_plane_add(
        size=max(max(xs) - min(xs), max(ys) - min(ys)) * 1.2,
        location=(center_x, center_y, 0.0),
    )
    water = bpy.context.active_object
    water.name = "WaterSurface"
    ensure_material(water, "WaterMaterial", (0.15, 0.35, 0.55), alpha=0.35)

    # Primary above-water camera.
    bpy.ops.object.camera_add(
        location=(center_x - sensor_standoff, center_y, primary_height)
    )
    cam_primary = bpy.context.active_object
    cam_primary.name = "Camera"
    look_at(cam_primary, look_target)

    # Secondary below-water camera placeholder for sonar scanner.
    water_surface_z = float(water.location.z)
    lower_bound = max(seabed_top + 0.25, seabed_bottom + 0.75)
    upper_bound = water_surface_z - 0.25
    if lower_bound > upper_bound:
        raise RuntimeError(
            "Seabed reaches too close to the water surface for a safe sonar camera placement. "
            "Raise the water surface or use a deeper crop."
        )
    secondary_z = min(-1.0, upper_bound)
    secondary_z = max(secondary_z, lower_bound)
    bpy.ops.object.camera_add(
        location=(center_x - sensor_standoff, center_y, secondary_z)
    )
    cam_secondary = bpy.context.active_object
    cam_secondary.name = "Camera.001"
    look_at(cam_secondary, look_target)

    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = 1
    bpy.context.scene.camera = cam_primary

    apply_scan_ready_config(
        bpy.context.scene,
        build_dual_sensor_config(
            primary_object_name=cam_primary.name,
            secondary_object_name=cam_secondary.name,
            frame_end=1,
            frame_rate=10.0,
            surface_height=float(water.location.z),
            enable_animation=False,
            add_noise=True,
        ),
    )

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    print(f"Saved scan-ready scene to {out_path}")


if __name__ == "__main__":
    main()
