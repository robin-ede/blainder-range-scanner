#!/usr/bin/env python3
"""Extract normalized point clouds from Kongsberg .all files using pyall.

This is intentionally a narrow validation utility, not a general ingestion system.
It expects the real GitHub `pyall` repo to be cloned at `external/pyall-github`.

Outputs:
- CSV with east,north,depth_m,quality,beam_id,ping_index
- NPY array with the same columns
- JSON metadata summary
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parent.parent
PYALL_ROOT = REPO_ROOT / "external" / "pyall-github"
if str(PYALL_ROOT) not in sys.path:
    sys.path.insert(0, str(PYALL_ROOT))

import pyall  # type: ignore
import geodetic  # type: ignore


def extract_points(
    input_path: Path, max_pings: int | None = None
) -> tuple[np.ndarray, dict]:
    epsg = pyall.getsuitableepsg(str(input_path))
    geo = geodetic.geodesy(epsg)
    reader = pyall.allreader(str(input_path))

    last_lat = None
    last_lon = None
    last_heading = None
    beamcounter = 0
    ping_index = 0
    rows = []
    datagram_counts = {}

    try:
        while reader.moredata():
            dtype, datagram = reader.readdatagram()
            datagram_counts[dtype] = datagram_counts.get(dtype, 0) + 1

            if dtype == "P":
                datagram.read()
                last_lat = getattr(datagram, "latitude", None)
                last_lon = getattr(datagram, "longitude", None)
                last_heading = getattr(datagram, "heading", None)
                continue

            if dtype != "X":
                continue

            datagram.read()
            if last_lat is None or last_lon is None:
                continue

            # pyall's point-cloud helper expects nav on the X datagram instance
            datagram.latitude = last_lat
            datagram.longitude = last_lon
            if getattr(datagram, "heading", None) is None and last_heading is not None:
                datagram.heading = last_heading

            east, north, depth, quality, beam_ids, beamcounter = (
                pyall.computebathypointcloud(datagram, geo, beamcounter)
            )

            for e, n, d, q, bid in zip(east, north, depth, quality, beam_ids):
                rows.append(
                    [float(e), float(n), float(d), float(q), int(bid), ping_index]
                )

            ping_index += 1
            if max_pings is not None and ping_index >= max_pings:
                break
    finally:
        reader.close()

    array = np.asarray(rows, dtype=float)
    meta = {
        "input": str(input_path.resolve()),
        "epsg": int(epsg),
        "ping_count": int(ping_index),
        "point_count": int(len(rows)),
        "datagram_counts": datagram_counts,
        "depth_min_m": float(array[:, 2].min()) if len(array) else None,
        "depth_max_m": float(array[:, 2].max()) if len(array) else None,
        "east_min": float(array[:, 0].min()) if len(array) else None,
        "east_max": float(array[:, 0].max()) if len(array) else None,
        "north_min": float(array[:, 1].min()) if len(array) else None,
        "north_max": float(array[:, 1].max()) if len(array) else None,
    }
    return array, meta


def write_outputs(points: np.ndarray, meta: dict, output_prefix: Path) -> None:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output_prefix.with_suffix(".csv")
    npy_path = output_prefix.with_suffix(".npy")
    json_path = output_prefix.with_suffix(".json")

    np.save(npy_path, points)
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            ["east", "north", "depth_m", "quality", "beam_id", "ping_index"]
        )
        writer.writerows(points.tolist())

    json_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"Wrote {csv_path}")
    print(f"Wrote {npy_path}")
    print(f"Wrote {json_path}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--max-pings", type=int, default=None)
    args = parser.parse_args(argv)

    points, meta = extract_points(Path(args.input), max_pings=args.max_pings)
    print(json.dumps(meta, indent=2))
    write_outputs(points, meta, Path(args.output_prefix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
