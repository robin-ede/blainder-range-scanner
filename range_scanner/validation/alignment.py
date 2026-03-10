import numpy as np


def _hit_point(hit, use_noisy_measurements=False):
    point = hit.location
    if use_noisy_measurements and hit.noiseLocation is not None:
        point = hit.noiseLocation
    return np.asarray((float(point.x), float(point.y), float(point.z)), dtype=float)


def _reference_point(hit):
    point = hit.location
    return np.asarray((float(point.x), float(point.y), float(point.z)), dtype=float)


def _estimate_frame_translation_from_reference(hits, use_noisy_measurements=False):
    residuals = []
    for hit in hits:
        measured = _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
        reference = _reference_point(hit)
        residuals.append(reference - measured)

    if not residuals:
        return np.zeros(3, dtype=float)

    residuals = np.asarray(residuals, dtype=float)
    return np.median(residuals, axis=0)


def _sample_points(points, max_points):
    if len(points) <= max_points:
        return points
    step = max(1, len(points) // max_points)
    return points[::step]


def _estimate_translation_to_point_cloud(
    source_points,
    target_points,
    max_iterations=6,
    max_points=400,
):
    if len(source_points) == 0 or len(target_points) == 0:
        return np.zeros(3, dtype=float)

    sampled_source = np.asarray(_sample_points(source_points, max_points), dtype=float)
    sampled_target = np.asarray(_sample_points(target_points, max_points), dtype=float)

    translation = np.zeros(3, dtype=float)
    for _ in range(max_iterations):
        shifted_source = sampled_source + translation
        distances = np.linalg.norm(
            shifted_source[:, None, :] - sampled_target[None, :, :], axis=2
        )
        nearest_indices = np.argmin(distances, axis=1)
        correspondences = sampled_target[nearest_indices]
        residuals = correspondences - shifted_source
        update = np.median(residuals, axis=0)
        translation += update

        if float(np.linalg.norm(update)) < 1e-4:
            break

    return translation


def align_hits_to_reference(hits, use_noisy_measurements=False):
    frame_hits = {}
    ordered_hits = []

    for hit in hits:
        frame = int(hit.frame) if hit.frame is not None else 0
        frame_hits.setdefault(frame, []).append(hit)
        ordered_hits.append((frame, hit))

    frame_translations = {}
    for frame, frame_group in frame_hits.items():
        frame_translations[frame] = _estimate_frame_translation_from_reference(
            frame_group,
            use_noisy_measurements=use_noisy_measurements,
        )

    corrected_points = []
    translation_norms = []
    for frame, hit in ordered_hits:
        translation = frame_translations.get(frame, np.zeros(3, dtype=float))
        corrected_points.append(
            tuple(
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                + translation
            )
        )
        translation_norms.append(float(np.linalg.norm(translation)))

    return {
        "alignment_mode": "framewise_reference_translation_baseline",
        "reference_mode": "clean_synthetic_reference_points",
        "corrected_points": corrected_points,
        "frame_translations": {
            str(frame): [float(value) for value in translation]
            for frame, translation in frame_translations.items()
        },
        "translation_summary": {
            "frame_count": len(frame_translations),
            "mean_translation_m": float(np.mean(translation_norms))
            if translation_norms
            else 0.0,
            "max_translation_m": float(np.max(translation_norms))
            if translation_norms
            else 0.0,
        },
    }


def align_hits_blind(
    hits, use_noisy_measurements=False, max_plausible_translation_m=5.0
):
    """Frame-to-frame blind ICP alignment.

    Uses only the immediately preceding frame's corrected points as the ICP target
    (not the full growing history) to prevent error accumulation and divergence.
    A translation plausibility cap is applied: if the estimated per-frame delta
    exceeds ``max_plausible_translation_m``, the translation is clamped to zero
    (no correction for that frame) to avoid catastrophic drift on failure cases.

    Parameters
    ----------
    hits : list of HitInfo
        All scan hits with .frame, .location, .noiseLocation attributes.
    use_noisy_measurements : bool
        If True, align the noisy (degraded) point variants.
    max_plausible_translation_m : float
        Maximum plausible per-frame translation delta in metres. Frames where the
        ICP estimate exceeds this are treated as un-correctable (delta set to zero).
    """
    frame_hits = {}
    ordered_hits = []

    for hit in hits:
        frame = int(hit.frame) if hit.frame is not None else 0
        frame_hits.setdefault(frame, []).append(hit)
        ordered_hits.append((frame, hit))

    sorted_frames = sorted(frame_hits.keys())
    frame_translations = {}
    accumulated_translations = {}
    # Only keep the previous frame's corrected points as the rolling reference
    previous_frame_corrected = None

    for frame in sorted_frames:
        frame_points = np.asarray(
            [
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                for hit in frame_hits[frame]
            ],
            dtype=float,
        )

        if previous_frame_corrected is None:
            # First frame: anchor — no correction
            frame_translations[frame] = np.zeros(3, dtype=float)
            accumulated_translations[frame] = np.zeros(3, dtype=float)
            previous_frame_corrected = frame_points
            continue

        # Estimate per-frame delta translation vs previous corrected frame
        delta = _estimate_translation_to_point_cloud(
            frame_points,
            previous_frame_corrected,
        )

        # Plausibility guard: discard deltas that are physically implausible
        if float(np.linalg.norm(delta)) > max_plausible_translation_m:
            delta = np.zeros(3, dtype=float)

        frame_translations[frame] = delta
        prev_frame = sorted_frames[sorted_frames.index(frame) - 1]
        accumulated_translations[frame] = accumulated_translations[prev_frame] + delta

        # Next frame's reference is this frame's corrected points only
        previous_frame_corrected = frame_points + accumulated_translations[frame]

    corrected_points = []
    translation_norms = []
    for frame, hit in ordered_hits:
        translation = accumulated_translations.get(frame, np.zeros(3, dtype=float))
        corrected_points.append(
            tuple(
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                + translation
            )
        )
        translation_norms.append(float(np.linalg.norm(translation)))

    return {
        "alignment_mode": "framewise_blind_translation_registration",
        "reference_mode": "previous_frame_point_cloud",
        "corrected_points": corrected_points,
        "frame_translations": {
            str(frame): [float(v) for v in translation]
            for frame, translation in accumulated_translations.items()
        },
        "translation_summary": {
            "frame_count": len(accumulated_translations),
            "mean_translation_m": float(np.mean(translation_norms))
            if translation_norms
            else 0.0,
            "max_translation_m": float(np.max(translation_norms))
            if translation_norms
            else 0.0,
        },
    }
