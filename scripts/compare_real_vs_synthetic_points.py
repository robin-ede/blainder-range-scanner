#!/usr/bin/env python3
"""Compare real sounder points against synthetic points on the same geometry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import laspy
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import ks_2samp


def load_synthetic_las(path: Path) -> np.ndarray:
    las = laspy.read(path)
    return np.column_stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)])


def load_real_points(path: Path, mesh_meta_path: Path) -> np.ndarray:
    pts = np.load(path)
    meta = json.loads(mesh_meta_path.read_text())
    center_e = (meta["east_min"] + meta["east_max"]) / 2.0
    center_n = (meta["north_min"] + meta["north_max"]) / 2.0
    east = pts[:, 0] - center_e
    north = -(pts[:, 1] - center_n)
    depth = -pts[:, 2]
    return np.column_stack([east, north, depth])


def nn_spacing_xy(points: np.ndarray) -> np.ndarray:
    tree = cKDTree(points[:, :2])
    dists, _ = tree.query(points[:, :2], k=2)
    return dists[:, 1]


def one_way_distance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    tree = cKDTree(b)
    dists, _ = tree.query(a, k=1)
    return dists


def stats(values: np.ndarray) -> dict:
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
    }


def bbox(points: np.ndarray) -> dict:
    return {
        "x_min": float(points[:, 0].min()),
        "x_max": float(points[:, 0].max()),
        "y_min": float(points[:, 1].min()),
        "y_max": float(points[:, 1].max()),
        "z_min": float(points[:, 2].min()),
        "z_max": float(points[:, 2].max()),
    }


def area_from_bbox(b: dict) -> float:
    return max(0.0, b["x_max"] - b["x_min"]) * max(0.0, b["y_max"] - b["y_min"])


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-points", required=True)
    parser.add_argument("--mesh-meta", required=True)
    parser.add_argument("--synthetic-las", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    real = load_real_points(Path(args.real_points), Path(args.mesh_meta))
    synthetic = load_synthetic_las(Path(args.synthetic_las))

    real_bbox = bbox(real)
    syn_bbox = bbox(synthetic)
    real_area = area_from_bbox(real_bbox)
    syn_area = area_from_bbox(syn_bbox)

    real_spacing = nn_spacing_xy(real)
    syn_spacing = nn_spacing_xy(synthetic)
    depth_ks = ks_2samp(real[:, 2], synthetic[:, 2])
    spacing_ks = ks_2samp(real_spacing, syn_spacing)

    real_to_syn = one_way_distance(real, synthetic)
    syn_to_real = one_way_distance(synthetic, real)

    out = {
        "real_point_count": int(len(real)),
        "synthetic_point_count": int(len(synthetic)),
        "real_bbox": real_bbox,
        "synthetic_bbox": syn_bbox,
        "real_density_pts_per_m2": float(len(real) / real_area)
        if real_area > 0
        else None,
        "synthetic_density_pts_per_m2": float(len(synthetic) / syn_area)
        if syn_area > 0
        else None,
        "real_depth_stats": stats(real[:, 2]),
        "synthetic_depth_stats": stats(synthetic[:, 2]),
        "real_spacing_stats": stats(real_spacing),
        "synthetic_spacing_stats": stats(syn_spacing),
        "depth_distribution_ks": {
            "statistic": float(depth_ks.statistic),
            "pvalue": float(depth_ks.pvalue),
        },
        "spacing_distribution_ks": {
            "statistic": float(spacing_ks.statistic),
            "pvalue": float(spacing_ks.pvalue),
        },
        "real_to_synthetic_distance": stats(real_to_syn),
        "synthetic_to_real_distance": stats(syn_to_real),
        "symmetric_chamfer_mean": float(
            (np.mean(real_to_syn) + np.mean(syn_to_real)) / 2.0
        ),
        "symmetric_chamfer_rmse": float(
            np.sqrt((np.mean(real_to_syn**2) + np.mean(syn_to_real**2)) / 2.0)
        ),
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(main(sys.argv[1:]))
