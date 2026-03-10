"""Reusable geometry-based path generation helpers for validation scenes."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class Bounds3D:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    @property
    def center_x(self) -> float:
        return (self.min_x + self.max_x) / 2.0

    @property
    def center_y(self) -> float:
        return (self.min_y + self.max_y) / 2.0

    @property
    def center_z(self) -> float:
        return (self.min_z + self.max_z) / 2.0

    @property
    def size_x(self) -> float:
        return self.max_x - self.min_x

    @property
    def size_y(self) -> float:
        return self.max_y - self.min_y

    @property
    def size_z(self) -> float:
        return self.max_z - self.min_z


def bounds_from_blender_object(obj) -> Bounds3D:
    corners = [obj.matrix_world @ obj.data.vertices[0].co]
    corners = [obj.matrix_world @ v.co for v in obj.data.vertices]
    xs = [point.x for point in corners]
    ys = [point.y for point in corners]
    zs = [point.z for point in corners]
    return Bounds3D(
        min_x=min(xs),
        max_x=max(xs),
        min_y=min(ys),
        max_y=max(ys),
        min_z=min(zs),
        max_z=max(zs),
    )


def _line_positions(start: float, end: float, spacing: float):
    positions = []
    value = start
    while value <= end + 1e-6:
        positions.append(value)
        value += spacing
    if positions and positions[-1] < end:
        positions.append(end)
    return positions


def _resolve_line_spacing(config: dict) -> float:
    if config.get("line_spacing_m") is not None:
        return float(config["line_spacing_m"])

    swath_width = float(config.get("swath_width_m", 60.0))
    overlap_ratio = float(config.get("overlap_ratio", 0.3))
    overlap_ratio = max(0.0, min(0.95, overlap_ratio))
    return max(swath_width * (1.0 - overlap_ratio), 1.0)


def _polyline_length(waypoints):
    length = 0.0
    for index in range(1, len(waypoints)):
        x0, y0, z0 = waypoints[index - 1]
        x1, y1, z1 = waypoints[index]
        length += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
    return length


def _resolve_frame_end(primary_waypoints, config: dict, frame_start: int, frame_end):
    if frame_end is not None:
        return int(frame_end)

    speed_mps = float(config.get("speed_mps", 15.0))
    frame_rate = float(config.get("frame_rate", 10.0))
    duration_s = _polyline_length(primary_waypoints) / max(speed_mps, 0.1)
    frame_count = max(int(math.ceil(duration_s * frame_rate)), 2)
    return int(frame_start + frame_count - 1)


def _waypoint_frames(waypoints, frame_start: int, frame_end: int):
    if len(waypoints) <= 1:
        return [frame_start]

    total = _polyline_length(waypoints)
    if total <= 0.0:
        return [frame_start for _ in waypoints]

    cumulative = [0.0]
    current = 0.0
    for index in range(1, len(waypoints)):
        x0, y0, z0 = waypoints[index - 1]
        x1, y1, z1 = waypoints[index]
        current += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
        cumulative.append(current)

    frame_span = frame_end - frame_start
    return [
        frame_start + int(round(frame_span * (distance / total)))
        for distance in cumulative
    ]


def _platform_heights(bounds: Bounds3D, config: dict):
    water_surface_z = float(config.get("water_surface_z", 0.0))
    primary_height = float(config.get("primary_height_m", 8.0))
    secondary_clearance = float(config.get("secondary_clearance_m", 1.5))
    secondary_min_depth = float(config.get("secondary_min_depth_m", -1.2))

    primary_z = water_surface_z + primary_height
    secondary_z = min(secondary_min_depth, bounds.max_z + secondary_clearance)
    return primary_z, secondary_z, water_surface_z


def generate_straight_path(bounds: Bounds3D, config: dict, frame_start: int, frame_end):
    margin = float(config.get("margin_m", 15.0))
    primary_z, secondary_z, _ = _platform_heights(bounds, config)
    sweep_axis = config.get("sweep_axis", "auto")
    if sweep_axis == "auto":
        sweep_axis = "x" if bounds.size_x >= bounds.size_y else "y"

    if sweep_axis == "x":
        x0 = bounds.min_x + margin
        x1 = bounds.max_x - margin
        y = bounds.center_y
        primary_waypoints = [[x0, y, primary_z], [x1, y, primary_z]]
        secondary_waypoints = [[x0, y, secondary_z], [x1, y, secondary_z]]
    else:
        y0 = bounds.min_y + margin
        y1 = bounds.max_y - margin
        x = bounds.center_x
        primary_waypoints = [[x, y0, primary_z], [x, y1, primary_z]]
        secondary_waypoints = [[x, y0, secondary_z], [x, y1, secondary_z]]

    resolved_frame_end = _resolve_frame_end(
        primary_waypoints, config, frame_start, frame_end
    )
    waypoint_frames = _waypoint_frames(
        primary_waypoints, frame_start, resolved_frame_end
    )

    return {
        "frame_start": frame_start,
        "frame_end": resolved_frame_end,
        "look_at": [bounds.center_x, bounds.center_y, bounds.center_z],
        "primary_waypoints": primary_waypoints,
        "secondary_waypoints": secondary_waypoints,
        "primary_waypoint_frames": waypoint_frames,
        "secondary_waypoint_frames": waypoint_frames,
    }


def generate_lawnmower_path(
    bounds: Bounds3D, config: dict, frame_start: int, frame_end
):
    margin = float(config.get("margin_m", 15.0))
    line_spacing = _resolve_line_spacing(config)
    primary_z, secondary_z, _ = _platform_heights(bounds, config)

    long_axis = "x" if bounds.size_x >= bounds.size_y else "y"
    if config.get("sweep_axis") in ("x", "y"):
        long_axis = config["sweep_axis"]

    primary_waypoints = []
    secondary_waypoints = []

    if long_axis == "x":
        x_start = bounds.min_x + margin
        x_end = bounds.max_x - margin
        line_positions = _line_positions(
            bounds.min_y + margin, bounds.max_y - margin, line_spacing
        )
        forward = True
        for y in line_positions:
            xs = [x_start, x_end] if forward else [x_end, x_start]
            for x in xs:
                primary_waypoints.append([x, y, primary_z])
                secondary_waypoints.append([x, y, secondary_z])
            forward = not forward
    else:
        y_start = bounds.min_y + margin
        y_end = bounds.max_y - margin
        line_positions = _line_positions(
            bounds.min_x + margin, bounds.max_x - margin, line_spacing
        )
        forward = True
        for x in line_positions:
            ys = [y_start, y_end] if forward else [y_end, y_start]
            for y in ys:
                primary_waypoints.append([x, y, primary_z])
                secondary_waypoints.append([x, y, secondary_z])
            forward = not forward

    resolved_frame_end = _resolve_frame_end(
        primary_waypoints, config, frame_start, frame_end
    )
    waypoint_frames = _waypoint_frames(
        primary_waypoints, frame_start, resolved_frame_end
    )

    return {
        "frame_start": frame_start,
        "frame_end": resolved_frame_end,
        "look_at": [bounds.center_x, bounds.center_y, bounds.center_z],
        "primary_waypoints": primary_waypoints,
        "secondary_waypoints": secondary_waypoints,
        "primary_waypoint_frames": waypoint_frames,
        "secondary_waypoint_frames": waypoint_frames,
        "coverage": {
            "line_spacing_m": line_spacing,
            "swath_width_m": float(config.get("swath_width_m", 60.0)),
            "overlap_ratio": float(config.get("overlap_ratio", 0.3)),
        },
    }


def generate_path_from_bounds(
    bounds: Bounds3D, generator: dict, frame_start: int, frame_end
):
    mode = generator.get("mode", "straight")
    if mode == "straight":
        return generate_straight_path(bounds, generator, frame_start, frame_end)
    if mode == "lawnmower":
        return generate_lawnmower_path(bounds, generator, frame_start, frame_end)
    raise ValueError("Unsupported generated path mode '%s'" % mode)
