#!/usr/bin/env python3
"""Convert processed bathymetry grids to Blender-ready mesh assets.

Supports:
- NetCDF grids with x/y/z variables
- GeoTIFF rasters
- BAG rasters (via rasterio/GDAL if supported in the environment)

Outputs:
- OBJ mesh
- depth grid .npy
- metadata JSON
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rasterio
import xarray as xr


def meters_per_degree_lat() -> float:
    return 111_320.0


def meters_per_degree_lon(latitude_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(latitude_deg))


def choose_crop_bounds(center_lon: float, center_lat: float, size_m: float):
    half_lat_deg = (size_m / 2.0) / meters_per_degree_lat()
    half_lon_deg = (size_m / 2.0) / meters_per_degree_lon(center_lat)
    return (
        center_lon - half_lon_deg,
        center_lon + half_lon_deg,
        center_lat - half_lat_deg,
        center_lat + half_lat_deg,
    )


def crop_netcdf(
    dataset_path: Path, min_lon: float, max_lon: float, min_lat: float, max_lat: float
):
    ds = xr.open_dataset(dataset_path, engine="h5netcdf")
    cropped = ds.sel(x=slice(min_lon, max_lon), y=slice(min_lat, max_lat))
    z = cropped["z"].values
    x = cropped["x"].values
    y = cropped["y"].values
    return x, y, z


def crop_raster(
    dataset_path: Path, min_lon: float, max_lon: float, min_lat: float, max_lat: float
):
    with rasterio.open(dataset_path) as src:
        window = rasterio.windows.from_bounds(
            min_lon, min_lat, max_lon, max_lat, transform=src.transform
        )
        data = src.read(1, window=window, boundless=True)
        transform = src.window_transform(window)

        rows, cols = data.shape
        xs = np.array([transform * (c + 0.5, 0.5) for c in range(cols)], dtype=float)[
            :, 0
        ]
        ys = np.array([transform * (0.5, r + 0.5) for r in range(rows)], dtype=float)[
            :, 1
        ]

        if ys[0] > ys[-1]:
            ys = ys[::-1]
            data = np.flipud(data)

        nodata = src.nodata
        if nodata is not None:
            data = np.where(data == nodata, np.nan, data)

        return xs, ys, data


def load_and_crop(
    dataset_path: Path, min_lon: float, max_lon: float, min_lat: float, max_lat: float
):
    suffix = dataset_path.suffix.lower()
    if suffix == ".nc":
        return crop_netcdf(dataset_path, min_lon, max_lon, min_lat, max_lat)
    if suffix in {".tif", ".tiff", ".bag"}:
        return crop_raster(dataset_path, min_lon, max_lon, min_lat, max_lat)
    raise ValueError(f"Unsupported processed bathymetry input '{dataset_path.suffix}'")


def write_obj(
    obj_path: Path, x: np.ndarray, y: np.ndarray, z: np.ndarray, z_scale: float
) -> None:
    center_lon = float((x.min() + x.max()) / 2.0)
    center_lat = float((y.min() + y.max()) / 2.0)
    x_scale = meters_per_degree_lon(center_lat)
    y_scale = meters_per_degree_lat()
    nx = len(x)
    ny = len(y)

    with obj_path.open("w", encoding="utf-8") as fh:
        fh.write("# Generated from processed bathymetry crop\n")
        fh.write("o seabed_crop\n")
        for j, lat in enumerate(y):
            for i, lon in enumerate(x):
                depth = z[j, i]
                if np.isnan(depth):
                    depth = 0.0
                local_x = (float(lon) - center_lon) * x_scale
                local_y = (float(lat) - center_lat) * y_scale
                local_z = float(depth) * z_scale
                fh.write(f"v {local_x:.6f} {local_z:.6f} {-local_y:.6f}\n")

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--center-lon", type=float, required=True)
    parser.add_argument("--center-lat", type=float, required=True)
    parser.add_argument("--size-m", type=float, default=800.0)
    parser.add_argument("--z-scale", type=float, default=1.0)
    parser.add_argument("--prefix", default="seabed_crop")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    min_lon, max_lon, min_lat, max_lat = choose_crop_bounds(
        args.center_lon, args.center_lat, args.size_m
    )
    x, y, z = load_and_crop(Path(args.input), min_lon, max_lon, min_lat, max_lat)
    if z.size == 0:
        raise SystemExit("Crop returned no data; adjust center or size.")

    obj_path = output_dir / f"{args.prefix}.obj"
    npy_path = output_dir / f"{args.prefix}_depths.npy"
    json_path = output_dir / f"{args.prefix}_metadata.json"
    write_obj(obj_path, x, y, z, args.z_scale)
    np.save(npy_path, z)

    meta = {
        "input": str(Path(args.input).resolve()),
        "center_lon": args.center_lon,
        "center_lat": args.center_lat,
        "size_m": args.size_m,
        "z_scale": args.z_scale,
        "bounds": {
            "min_lon": min_lon,
            "max_lon": max_lon,
            "min_lat": min_lat,
            "max_lat": max_lat,
        },
        "grid_shape": [int(z.shape[0]), int(z.shape[1])],
        "depth_min_m": float(np.nanmin(z)),
        "depth_max_m": float(np.nanmax(z)),
        "depth_mean_m": float(np.nanmean(z)),
        "obj": str(obj_path.resolve()),
        "depth_npy": str(npy_path.resolve()),
    }
    json_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
