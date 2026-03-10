import math

from mathutils import Vector


def get_degradation_config(properties):
    return {
        "gnss_drift_m": float(getattr(properties, "gnssDriftMeters", 0.0) or 0.0),
        "gnss_drift_heading_deg": float(
            getattr(properties, "gnssDriftHeadingDegrees", 0.0) or 0.0
        ),
        "sea_state_amplitude_m": float(
            getattr(properties, "seaStateAmplitudeMeters", 0.0) or 0.0
        ),
        "sea_state_period_frames": int(
            getattr(properties, "seaStatePeriodFrames", 20) or 20
        ),
        "sea_state_phase_deg": float(
            getattr(properties, "seaStatePhaseDegrees", 0.0) or 0.0
        ),
    }


def degradation_enabled(config):
    return config["gnss_drift_m"] > 0.0 or config["sea_state_amplitude_m"] > 0.0


def _frame_progress(frame, frame_start, frame_end):
    if frame is None or frame_end <= frame_start:
        return 0.0
    return max(
        0.0, min(1.0, (float(frame) - frame_start) / float(frame_end - frame_start))
    )


def _gnss_offset(config, progress):
    magnitude = config["gnss_drift_m"] * progress
    heading_rad = math.radians(config["gnss_drift_heading_deg"])
    return Vector(
        (
            magnitude * math.cos(heading_rad),
            magnitude * math.sin(heading_rad),
            0.0,
        )
    )


def _sea_state_offset(config, frame, frame_start):
    amplitude = config["sea_state_amplitude_m"]
    period = max(1, config["sea_state_period_frames"])
    phase = math.radians(config["sea_state_phase_deg"])
    frame_term = (
        2.0
        * math.pi
        * (float(frame or frame_start) - float(frame_start))
        / float(period)
    )
    return Vector((0.0, 0.0, amplitude * math.sin(frame_term + phase)))


def apply_maritime_degradation(hits, properties):
    config = get_degradation_config(properties)
    if not degradation_enabled(config):
        return {
            "enabled": False,
            "config": config,
            "applied_hit_count": 0,
        }

    frame_start = int(getattr(properties, "frameStart", 1) or 1)
    frame_end = int(getattr(properties, "frameEnd", frame_start) or frame_start)

    for hit in hits:
        base_location = (
            hit.noiseLocation if hit.noiseLocation is not None else hit.location
        )
        progress = _frame_progress(hit.frame, frame_start, frame_end)
        offset = _gnss_offset(config, progress) + _sea_state_offset(
            config, hit.frame, frame_start
        )
        hit.noiseLocation = Vector(base_location) + offset

        base_distance = (
            hit.noiseDistance if hit.noiseDistance is not None else hit.distance
        )
        hit.noiseDistance = float(base_distance + offset.length)

    return {
        "enabled": True,
        "config": config,
        "applied_hit_count": len(hits),
    }
