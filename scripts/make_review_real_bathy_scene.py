import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def look_at(obj, target):
    direction = target - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    bpy.ops.wm.open_mainfile(filepath=args.input)
    mesh_objs = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not mesh_objs:
        raise RuntimeError("No mesh found")
    seabed = mesh_objs[0]
    seabed.name = "Seabed"

    for obj in list(bpy.context.scene.objects):
        if obj.type == "CAMERA":
            bpy.data.objects.remove(obj, do_unlink=True)

    bpy.context.view_layer.objects.active = seabed
    bpy.ops.object.shade_smooth()

    dims = seabed.dimensions
    target = Vector((0.0, 0.0, float(seabed.location.z - dims.z * 0.2)))

    bpy.ops.object.camera_add(
        location=(
            dims.x * 0.75,
            -dims.y * 1.5 - dims.x * 0.25,
            dims.z * 8.0 + dims.y * 0.2,
        )
    )
    cam = bpy.context.active_object
    cam.name = "OverviewCamera"
    look_at(cam, target)
    bpy.context.scene.camera = cam

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    print(f"Saved review scene to {out_path}")


if __name__ == "__main__":
    main()
