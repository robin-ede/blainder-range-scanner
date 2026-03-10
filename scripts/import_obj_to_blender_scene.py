import argparse
import sys
from pathlib import Path

import bpy


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def import_obj(obj_path: str):
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=obj_path)
    else:
        bpy.ops.import_scene.obj(filepath=obj_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--name", default="SeabedCrop")
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    clear_scene()
    import_obj(args.obj)

    imported = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not imported:
        raise RuntimeError("No mesh objects imported")

    mesh_obj = imported[0]
    mesh_obj.name = args.name
    mesh_obj.data.name = f"{args.name}Mesh"
    bpy.context.view_layer.objects.active = mesh_obj
    mesh_obj.select_set(True)

    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="BOUNDS")

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    print(f"Saved Blender scene to {out_path}")


if __name__ == "__main__":
    main()
