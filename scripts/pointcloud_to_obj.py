#!/usr/bin/env python3
"""Convert a real point cloud into a gridded OBJ mesh for Blender.

Intended for validation: take real sounder points, interpolate them onto a grid,
and create a mesh that can be scanned synthetically.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import griddata


def write_obj(
    obj_path: Path, x_coords: np.ndarray, y_coords: np.ndarray, z_grid: np.ndarray
) -> None:
    nx = len(x_coords)
    ny = len(y_coords)
    center_x = float((x_coords.min() + x_coords.max()) / 2.0)
    center_y = float((y_coords.min() + y_coords.max()) / 2.0)

    with obj_path.open("w", encoding="utf-8") as fh:
        fh.write("# Generated from real point cloud\n")
        fh.write("o real_pointcloud_surface\n")

        for j, y in enumerate(y_coords):
            for i, x in enumerate(x_coords):
                z = z_grid[j, i]
                if np.isnan(z):
                    z = 0.0
                local_x = float(x - center_x)
                # Keep OBJ in Y-up convention, but place depths below the waterline.
                local_y = float(-z)
                local_z = float(-(y - center_y))
                fh.write(f"v {local_x:.6f} {local_y:.6f} {local_z:.6f}\n")

        def vid(ix: int, iy: int) -> int:
            return iy * nx + ix + 1

        for j in range(ny - 1):
            for i in range(nx - 1):
                v1 = vid(i, j)
                v2 = vid(i + 1, j)
                v3 = vid(i + 1, j + 1)
                v4 = vid(i, j + 1)
                fh.write(f"f {v1} {v2} {v3}\n")
                fh.write(f"f {v1} {v3} {v4}\n")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", required=True, help="NPY file with east,north,depth,... columns"
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cell-size", type=float, default=2.0)
    parser.add_argument("--prefix", default="real_pointcloud_mesh")
    parser.add_argument("--max-points", type=int)
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    points = np.load(args.input)
    if args.max_points and len(points) > args.max_points:
        idx = np.linspace(0, len(points) - 1, args.max_points).astype(int)
        points = points[idx]

    east = points[:, 0]
    north = points[:, 1]
    depth = points[:, 2]

    min_e, max_e = float(east.min()), float(east.max())
    min_n, max_n = float(north.min()), float(north.max())

    x_coords = np.arange(min_e, max_e + args.cell_size, args.cell_size)
    y_coords = np.arange(min_n, max_n + args.cell_size, args.cell_size)
    grid_x, grid_y = np.meshgrid(x_coords, y_coords)

    linear = griddata((east, north), depth, (grid_x, grid_y), method="linear")
    nearest = griddata((east, north), depth, (grid_x, grid_y), method="nearest")
    z_grid = np.where(np.isnan(linear), nearest, linear)

    obj_path = output_dir / f"{args.prefix}.obj"
    grid_path = output_dir / f"{args.prefix}_grid.npy"
    meta_path = output_dir / f"{args.prefix}_metadata.json"
    write_obj(obj_path, x_coords, y_coords, z_grid)
    np.save(grid_path, z_grid)

    meta = {
        "input": str(Path(args.input).resolve()),
        "point_count": int(len(points)),
        "cell_size": args.cell_size,
        "east_min": min_e,
        "east_max": max_e,
        "north_min": min_n,
        "north_max": max_n,
        "depth_min": float(np.nanmin(z_grid)),
        "depth_max": float(np.nanmax(z_grid)),
        "grid_shape": [int(z_grid.shape[0]), int(z_grid.shape[1])],
        "obj": str(obj_path.resolve()),
        "grid": str(grid_path.resolve()),
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(main(sys.argv[1:]))
