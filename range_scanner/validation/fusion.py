import math

import numpy as np


def _point_tuple(hit, use_noisy_measurements=False):
    point = hit.location
    if use_noisy_measurements and hit.noiseLocation is not None:
        point = hit.noiseLocation
    return (float(point.x), float(point.y), float(point.z))


def _point_medium(point, water_surface_height):
    if water_surface_height is None:
        return "unknown"
    if point[2] >= water_surface_height:
        return "above_water"
    return "below_water"


def _sensor_medium_weight(sensor_id, medium):
    weights = {
        ("lidar", "above_water"): 1.0,
        ("lidar", "below_water"): 0.1,
        ("sonar", "above_water"): 0.1,
        ("sonar", "below_water"): 1.0,
        ("unknown", "above_water"): 0.5,
        ("unknown", "below_water"): 0.5,
        ("unknown", "unknown"): 0.5,
    }
    return weights.get((sensor_id, medium), 0.5)


def _distance_weight(hit):
    measurement_distance = hit.noiseDistance
    if measurement_distance is None:
        measurement_distance = hit.distance
    if measurement_distance is None:
        return 1.0
    return 1.0 / max(float(measurement_distance), 1e-3)


def _reflection_weight(hit):
    return 0.7 if getattr(hit, "wasReflected", False) else 1.0


def _robust_weighted_point(bucket):
    points = np.asarray(bucket["points"], dtype=float)
    weights = np.asarray(bucket["weights"], dtype=float)

    if len(points) == 1:
        return tuple(float(value) for value in points[0])

    median_point = np.median(points, axis=0)
    residuals = np.linalg.norm(points - median_point, axis=1)
    residual_limit = np.percentile(residuals, 80)
    keep_mask = residuals <= residual_limit

    if not np.any(keep_mask):
        keep_mask = np.ones(len(points), dtype=bool)

    kept_points = points[keep_mask]
    kept_weights = weights[keep_mask]

    if float(np.sum(kept_weights)) <= 0.0:
        kept_weights = np.ones(len(kept_points), dtype=float)

    weighted_center = np.average(kept_points, axis=0, weights=kept_weights)
    center_distances = np.linalg.norm(kept_points - weighted_center, axis=1)
    representative_index = int(np.argmin(center_distances))
    representative_point = kept_points[representative_index]
    return tuple(float(value) for value in representative_point)


def _summarize_fused_buckets(voxels, input_count, water_surface_height):
    fused_points = []
    per_sensor_contribution = {}
    multi_sensor_voxels = 0
    multi_frame_voxels = 0
    point_counts_by_medium = {"above_water": 0, "below_water": 0}
    fused_point_counts_by_medium = {"above_water": 0, "below_water": 0}

    for bucket in voxels.values():
        fused_point = _robust_weighted_point(bucket)
        fused_points.append(fused_point)

        fused_medium = _point_medium(fused_point, water_surface_height)
        if fused_medium in fused_point_counts_by_medium:
            fused_point_counts_by_medium[fused_medium] += 1

        if len(bucket["sensor_counts"]) > 1:
            multi_sensor_voxels += 1
        if len(bucket["frames"]) > 1:
            multi_frame_voxels += 1

        for sensor_id, count in bucket["sensor_counts"].items():
            per_sensor_contribution[sensor_id] = (
                per_sensor_contribution.get(sensor_id, 0) + count
            )

        for point in bucket["points"]:
            medium = _point_medium(point, water_surface_height)
            if medium in point_counts_by_medium:
                point_counts_by_medium[medium] += 1

    return {
        "input_point_count": input_count,
        "fused_point_count": len(fused_points),
        "compression_ratio": (
            float(len(fused_points)) / float(input_count) if input_count > 0 else None
        ),
        "multi_sensor_voxel_count": multi_sensor_voxels,
        "multi_frame_voxel_count": multi_frame_voxels,
        "per_sensor_input_counts": per_sensor_contribution,
        "point_counts_by_medium": point_counts_by_medium,
        "fused_point_counts_by_medium": fused_point_counts_by_medium,
        "fused_points": fused_points,
    }


def _fuse_hits_internal(
    hits,
    voxel_size=0.05,
    use_noisy_measurements=False,
    water_surface_height=None,
    aligned_points=None,
    fusion_mode="weighted_trimmed_voxel",
):
    if voxel_size <= 0.0:
        raise ValueError("voxel_size must be positive")

    if aligned_points is not None and len(aligned_points) != len(hits):
        raise ValueError("aligned_points must match hits length")

    voxels = {}

    for index, hit in enumerate(hits):
        if aligned_points is None:
            point = _point_tuple(hit, use_noisy_measurements=use_noisy_measurements)
        else:
            aligned_point = aligned_points[index]
            point = (
                float(aligned_point[0]),
                float(aligned_point[1]),
                float(aligned_point[2]),
            )
        voxel_key = tuple(int(math.floor(value / voxel_size)) for value in point)
        bucket = voxels.setdefault(
            voxel_key,
            {
                "points": [],
                "weights": [],
                "sensor_counts": {},
                "frames": set(),
            },
        )

        sensor_id = hit.sensor_id or "unknown"
        medium = _point_medium(point, water_surface_height)
        base_weight = _sensor_medium_weight(sensor_id, medium)
        point_weight = base_weight * _distance_weight(hit) * _reflection_weight(hit)

        bucket["points"].append(point)
        bucket["weights"].append(point_weight)
        bucket["sensor_counts"][sensor_id] = (
            bucket["sensor_counts"].get(sensor_id, 0) + 1
        )
        if hit.frame is not None:
            bucket["frames"].add(int(hit.frame))

    return {
        "fusion_mode": fusion_mode,
        "voxel_size_m": float(voxel_size),
        **_summarize_fused_buckets(voxels, len(hits), water_surface_height),
    }


def voxel_fuse_hits(
    hits,
    voxel_size=0.05,
    use_noisy_measurements=False,
    water_surface_height=None,
):
    return _fuse_hits_internal(
        hits,
        voxel_size=voxel_size,
        use_noisy_measurements=use_noisy_measurements,
        water_surface_height=water_surface_height,
    )


def voxel_fuse_aligned_hits(
    hits,
    aligned_points,
    voxel_size=0.05,
    water_surface_height=None,
):
    return _fuse_hits_internal(
        hits,
        voxel_size=voxel_size,
        water_surface_height=water_surface_height,
        aligned_points=aligned_points,
        fusion_mode="aligned_weighted_trimmed_voxel",
    )


def voxel_fuse_points(
    points,
    voxel_size=0.05,
    water_surface_height=None,
):
    if voxel_size <= 0.0:
        raise ValueError("voxel_size must be positive")

    voxels = {}

    for point in points:
        point_tuple = (float(point[0]), float(point[1]), float(point[2]))
        voxel_key = tuple(int(math.floor(value / voxel_size)) for value in point_tuple)
        bucket = voxels.setdefault(
            voxel_key,
            {
                "points": [],
                "weights": [],
                "sensor_counts": {},
                "frames": set(),
            },
        )
        bucket["points"].append(point_tuple)
        bucket["weights"].append(1.0)
        bucket["sensor_counts"]["aligned_points"] = (
            bucket["sensor_counts"].get("aligned_points", 0) + 1
        )

    return {
        "fusion_mode": "aligned_trimmed_voxel",
        "voxel_size_m": float(voxel_size),
        **_summarize_fused_buckets(voxels, len(points), water_surface_height),
    }


def reconstruct_medium_separated_map(
    hits,
    above_water_voxel_size=0.04,
    below_water_voxel_size=0.06,
    use_noisy_measurements=False,
    water_surface_height=None,
):
    above_water_hits = []
    below_water_hits = []

    for hit in hits:
        point = _point_tuple(hit, use_noisy_measurements=use_noisy_measurements)
        medium = _point_medium(point, water_surface_height)
        if medium == "above_water":
            above_water_hits.append(hit)
        elif medium == "below_water":
            below_water_hits.append(hit)

    above_results = _fuse_hits_internal(
        above_water_hits,
        voxel_size=above_water_voxel_size,
        use_noisy_measurements=use_noisy_measurements,
        water_surface_height=water_surface_height,
    )
    below_results = _fuse_hits_internal(
        below_water_hits,
        voxel_size=below_water_voxel_size,
        use_noisy_measurements=use_noisy_measurements,
        water_surface_height=water_surface_height,
    )

    stitched_points = list(above_results["fused_points"]) + list(
        below_results["fused_points"]
    )
    input_count = (
        above_results["input_point_count"] + below_results["input_point_count"]
    )
    fused_count = (
        above_results["fused_point_count"] + below_results["fused_point_count"]
    )

    return {
        "reconstruction_mode": "medium_separated_stitched_map",
        "input_point_count": input_count,
        "reconstructed_point_count": fused_count,
        "compression_ratio": (
            float(fused_count) / float(input_count) if input_count > 0 else None
        ),
        "stitched_points": stitched_points,
        "regions": {
            "above_water": above_results,
            "below_water": below_results,
        },
    }


def _grid_reconstruct_points(points, grid_size):
    if grid_size <= 0.0:
        raise ValueError("grid_size must be positive")

    cells = {}
    for point in points:
        cell_key = (
            int(math.floor(point[0] / grid_size)),
            int(math.floor(point[1] / grid_size)),
        )
        bucket = cells.setdefault(cell_key, [])
        bucket.append(point)

    reconstructed = []
    for bucket in cells.values():
        arr = np.asarray(bucket, dtype=float)
        representative = np.median(arr, axis=0)
        reconstructed.append(tuple(float(value) for value in representative))

    return reconstructed


def reconstruct_medium_height_grid_map(
    hits,
    above_water_grid_size=0.08,
    below_water_grid_size=0.10,
    use_noisy_measurements=False,
    water_surface_height=None,
):
    above_points = []
    below_points = []

    for hit in hits:
        point = _point_tuple(hit, use_noisy_measurements=use_noisy_measurements)
        medium = _point_medium(point, water_surface_height)
        if medium == "above_water":
            above_points.append(point)
        elif medium == "below_water":
            below_points.append(point)

    reconstructed_above = _grid_reconstruct_points(above_points, above_water_grid_size)
    reconstructed_below = _grid_reconstruct_points(below_points, below_water_grid_size)
    stitched_points = reconstructed_above + reconstructed_below

    input_count = len(above_points) + len(below_points)
    reconstructed_count = len(stitched_points)

    return {
        "reconstruction_mode": "medium_height_grid_map",
        "input_point_count": input_count,
        "reconstructed_point_count": reconstructed_count,
        "compression_ratio": (
            float(reconstructed_count) / float(input_count) if input_count > 0 else None
        ),
        "stitched_points": stitched_points,
        "regions": {
            "above_water": {
                "input_point_count": len(above_points),
                "reconstructed_point_count": len(reconstructed_above),
            },
            "below_water": {
                "input_point_count": len(below_points),
                "reconstructed_point_count": len(reconstructed_below),
            },
        },
    }


def reconstruct_medium_height_grid_from_points(
    points,
    above_water_grid_size=0.08,
    below_water_grid_size=0.10,
    water_surface_height=None,
):
    above_points = []
    below_points = []

    for point in points:
        medium = _point_medium(point, water_surface_height)
        if medium == "above_water":
            above_points.append(point)
        elif medium == "below_water":
            below_points.append(point)

    reconstructed_above = _grid_reconstruct_points(above_points, above_water_grid_size)
    reconstructed_below = _grid_reconstruct_points(below_points, below_water_grid_size)
    stitched_points = reconstructed_above + reconstructed_below

    input_count = len(above_points) + len(below_points)
    reconstructed_count = len(stitched_points)

    return {
        "reconstruction_mode": "medium_height_grid_map",
        "input_point_count": input_count,
        "reconstructed_point_count": reconstructed_count,
        "compression_ratio": (
            float(reconstructed_count) / float(input_count) if input_count > 0 else None
        ),
        "stitched_points": stitched_points,
        "regions": {
            "above_water": {
                "input_point_count": len(above_points),
                "reconstructed_point_count": len(reconstructed_above),
            },
            "below_water": {
                "input_point_count": len(below_points),
                "reconstructed_point_count": len(reconstructed_below),
            },
        },
    }
