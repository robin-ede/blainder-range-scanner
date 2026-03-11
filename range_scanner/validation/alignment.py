import numpy as np


DEFAULT_BLIND_ALIGNMENT_METHOD = "open3d_icp"
DEFAULT_OPEN3D_ICP_CONFIG = {
    # Target selection ---------------------------------------------------
    # Start with the immediately previous corrected frame.  When the
    # accumulated target has fewer points than *min_target_points_for_window*,
    # expand the window up to *max_target_window_frames*.  Setting
    # max_target_window_frames to a large value (e.g. 999) means "use all
    # previously corrected frames" which provides the most stable target
    # for sparse sensor streams.
    "target_window_frames": 1,
    "max_target_window_frames": 999,
    "min_target_points_for_window": 2000,
    # Point-count limits -------------------------------------------------
    "min_points_per_frame": 30,
    "max_source_points": 3000,
    "max_target_points": 10000,
    # Multi-resolution ICP cascade (coarse -> fine) ----------------------
    "voxel_sizes_m": (0.30, 0.12, 0.05),
    "max_correspondence_distances_m": (1.50, 0.50, 0.18),
    "max_iterations": (50, 30, 20),
    # Gating thresholds --------------------------------------------------
    "max_delta_translation_m": 2.00,
    "max_total_translation_m": 10.0,
    "min_fitness": 0.10,
    "max_inlier_rmse_m": 0.40,
    # Coarse-vs-refined divergence gate: if ICP result diverges from the
    # coarse seed by more than this, reject the ICP result.
    "max_coarse_icp_divergence_m": 1.00,
}


# ---------------------------------------------------------------------------
# Point helpers
# ---------------------------------------------------------------------------


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
    """Estimate an *absolute* translation that maps source_points into the
    coordinate frame of target_points.  The returned vector is the shift to
    *add* to each source point so that it lands near the target cloud.

    This is a simple iterative closest-point median alignment — no rotation.
    """
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


# ---------------------------------------------------------------------------
# Frame grouping helpers
# ---------------------------------------------------------------------------


def _group_hits_by_frame(hits):
    frame_hits = {}
    ordered_hits = []

    for hit in hits:
        frame = int(hit.frame) if hit.frame is not None else 0
        frame_hits.setdefault(frame, []).append(hit)
        ordered_hits.append((frame, hit))

    return frame_hits, ordered_hits, sorted(frame_hits.keys())


def _frame_points(frame_hits, use_noisy_measurements=False):
    points = []
    for hit in frame_hits:
        point = _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
        if np.all(np.isfinite(point)):
            points.append(point)
    if not points:
        return np.empty((0, 3), dtype=float)
    return np.asarray(points, dtype=float)


def _translation_summary(frame_translations, point_count):
    translation_norms = [
        float(np.linalg.norm(translation))
        for frame, translation in frame_translations.items()
        for _ in range(point_count.get(frame, 0))
    ]
    return {
        "frame_count": len(frame_translations),
        "mean_translation_m": float(np.mean(translation_norms))
        if translation_norms
        else 0.0,
        "max_translation_m": float(np.max(translation_norms))
        if translation_norms
        else 0.0,
    }


# ---------------------------------------------------------------------------
# Legacy blind alignment (fixed accumulation semantics)
# ---------------------------------------------------------------------------


def _legacy_blind_alignment(
    hits,
    use_noisy_measurements=False,
    max_plausible_translation_m=5.0,
):
    """Frame-by-frame blind translation alignment using median nearest-
    neighbour matching.

    Translation semantics
    ---------------------
    ``_estimate_translation_to_point_cloud(raw_frame, corrected_target)``
    returns an *absolute* translation: the shift to add to the raw source
    so it lands in the target frame.  Because the target is the *already-
    corrected* previous frame (i.e. already in the map frame), the result
    is directly the absolute correction for this frame — *not* a delta to
    accumulate on top of the previous correction.

    The old code treated it as a delta (``prev + delta``), which caused
    run-away drift when the true drift was large.
    """
    frame_hits, ordered_hits, sorted_frames = _group_hits_by_frame(hits)

    absolute_translations = {}
    corrected_frame_points_map = {}

    for frame_index, frame in enumerate(sorted_frames):
        frame_pts = _frame_points(
            frame_hits[frame], use_noisy_measurements=use_noisy_measurements
        )

        if frame_index == 0:
            absolute_translations[frame] = np.zeros(3, dtype=float)
            corrected_frame_points_map[frame] = frame_pts
            continue

        previous_frame = sorted_frames[frame_index - 1]
        target = corrected_frame_points_map[previous_frame]

        # _estimate_translation_to_point_cloud returns the absolute shift
        # that maps raw frame_pts into the target frame.
        absolute_t = _estimate_translation_to_point_cloud(frame_pts, target)

        if float(np.linalg.norm(absolute_t)) > max_plausible_translation_m:
            # Fall back to carrying forward the previous translation
            absolute_t = absolute_translations[previous_frame].copy()

        absolute_translations[frame] = absolute_t
        corrected_frame_points_map[frame] = frame_pts + absolute_t

    corrected_points = []
    frame_point_count = {}
    per_frame_deltas = {}
    for frame_index, frame in enumerate(sorted_frames):
        if frame_index == 0:
            per_frame_deltas[frame] = np.zeros(3, dtype=float)
        else:
            prev = sorted_frames[frame_index - 1]
            per_frame_deltas[frame] = (
                absolute_translations[frame] - absolute_translations[prev]
            )

    for frame, hit in ordered_hits:
        translation = absolute_translations.get(frame, np.zeros(3, dtype=float))
        corrected_points.append(
            tuple(
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                + translation
            )
        )
        frame_point_count[frame] = frame_point_count.get(frame, 0) + 1

    return {
        "alignment_mode": "framewise_blind_translation_registration",
        "reference_mode": "previous_frame_point_cloud",
        "corrected_points": corrected_points,
        "frame_translations": {
            str(frame): [float(v) for v in translation]
            for frame, translation in absolute_translations.items()
        },
        "translation_summary": _translation_summary(
            absolute_translations, frame_point_count
        ),
        "registration_summary": {
            "backend": "legacy_translation",
            "accepted_icp_frames": 0,
            "fallback_translation_frames": max(0, len(sorted_frames) - 1),
            "identity_fallback_frames": 0,
            "insufficient_points_frames": 0,
            "mean_fitness": None,
            "mean_inlier_rmse": None,
        },
        "per_frame_status": {
            str(frame): {
                "status": "accepted_translation_baseline",
                "fitness": None,
                "inlier_rmse": None,
                "source_point_count": len(frame_hits.get(frame, [])),
                "target_point_count": len(frame_hits.get(frame, [])),
                "delta_translation_m": float(
                    np.linalg.norm(per_frame_deltas.get(frame, np.zeros(3)))
                ),
                "translation_m": float(
                    np.linalg.norm(absolute_translations.get(frame, np.zeros(3)))
                ),
                "fallback_reason": None,
            }
            for frame in sorted_frames
        },
        "warnings": [],
    }


# ---------------------------------------------------------------------------
# Open3D helpers
# ---------------------------------------------------------------------------


def _try_import_open3d():
    try:
        import open3d as o3d

        return o3d, None
    except Exception as exc:
        return None, str(exc)


def _build_open3d_cloud(o3d, points):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    return cloud


def _subsample_points(points, max_points):
    if max_points is None or max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, num=max_points, dtype=int)
    return points[indices]


def _build_target_points(
    corrected_frame_points,
    sorted_frames,
    current_index,
    window_size,
    min_target_points=0,
    max_window_size=None,
):
    """Build a target cloud from recent corrected frames.

    Starts with ``window_size`` preceding frames.  If the resulting cloud
    has fewer than ``min_target_points`` and ``max_window_size`` allows,
    expands the window until enough points are gathered or the window limit
    is reached.
    """
    effective_window = window_size
    if max_window_size is None:
        max_window_size = window_size

    while effective_window <= max_window_size:
        start_index = max(0, current_index - effective_window)
        target_frames = sorted_frames[start_index:current_index]
        target_arrays = [
            corrected_frame_points[frame]
            for frame in target_frames
            if frame in corrected_frame_points
            and len(corrected_frame_points[frame]) > 0
        ]
        if not target_arrays:
            total = 0
        else:
            total = sum(len(a) for a in target_arrays)

        if total >= min_target_points or effective_window >= max_window_size:
            break
        effective_window += 1

    start_index = max(0, current_index - effective_window)
    target_frames = sorted_frames[start_index:current_index]
    target_arrays = [
        corrected_frame_points[frame]
        for frame in target_frames
        if frame in corrected_frame_points and len(corrected_frame_points[frame]) > 0
    ]
    if not target_arrays:
        return np.empty((0, 3), dtype=float), target_frames
    return np.concatenate(target_arrays, axis=0), target_frames


def _coerce_translation_transform(transform):
    translation_only = np.eye(4, dtype=float)
    translation_only[:3, 3] = np.asarray(transform[:3, 3], dtype=float)
    return translation_only


def _estimate_translation_open3d(
    o3d,
    source_points,
    target_points,
    initial_translation,
    config,
):
    source_points = _subsample_points(source_points, config["max_source_points"])
    target_points = _subsample_points(target_points, config["max_target_points"])

    if len(source_points) < config["min_points_per_frame"]:
        return {
            "status": "insufficient_source_points",
            "translation": np.asarray(initial_translation, dtype=float),
            "fitness": None,
            "inlier_rmse": None,
            "source_point_count": int(len(source_points)),
            "target_point_count": int(len(target_points)),
            "transformation": np.eye(4, dtype=float),
        }

    if len(target_points) < config["min_points_per_frame"]:
        return {
            "status": "insufficient_target_points",
            "translation": np.asarray(initial_translation, dtype=float),
            "fitness": None,
            "inlier_rmse": None,
            "source_point_count": int(len(source_points)),
            "target_point_count": int(len(target_points)),
            "transformation": np.eye(4, dtype=float),
        }

    current_transform = np.eye(4, dtype=float)
    current_transform[:3, 3] = np.asarray(initial_translation, dtype=float)
    result = None

    for voxel_size, distance_threshold, max_iterations in zip(
        config["voxel_sizes_m"],
        config["max_correspondence_distances_m"],
        config["max_iterations"],
    ):
        source_cloud = _build_open3d_cloud(o3d, source_points)
        target_cloud = _build_open3d_cloud(o3d, target_points)

        if voxel_size and voxel_size > 0.0:
            source_cloud = source_cloud.voxel_down_sample(float(voxel_size))
            target_cloud = target_cloud.voxel_down_sample(float(voxel_size))

        if len(source_cloud.points) == 0 or len(target_cloud.points) == 0:
            break

        result = o3d.pipelines.registration.registration_icp(
            source_cloud,
            target_cloud,
            float(distance_threshold),
            current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(max_iterations)
            ),
        )
        current_transform = _coerce_translation_transform(result.transformation)

    if result is None:
        return {
            "status": "icp_not_run",
            "translation": np.asarray(initial_translation, dtype=float),
            "fitness": None,
            "inlier_rmse": None,
            "source_point_count": int(len(source_points)),
            "target_point_count": int(len(target_points)),
            "transformation": np.eye(4, dtype=float),
        }

    return {
        "status": "ok",
        "translation": np.asarray(current_transform[:3, 3], dtype=float),
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "source_point_count": int(len(source_points)),
        "target_point_count": int(len(target_points)),
        "transformation": current_transform,
    }


# ---------------------------------------------------------------------------
# Per-frame status builder
# ---------------------------------------------------------------------------


def _frame_status(
    status,
    source_count,
    target_count,
    translation,
    delta,
    fitness,
    inlier_rmse,
    fallback_reason=None,
    coarse_translation=None,
    icp_translation=None,
):
    entry = {
        "status": status,
        "fitness": fitness,
        "inlier_rmse": inlier_rmse,
        "source_point_count": int(source_count),
        "target_point_count": int(target_count),
        "translation_m": float(np.linalg.norm(translation)),
        "delta_translation_m": float(np.linalg.norm(delta)),
        "fallback_reason": fallback_reason,
    }
    if coarse_translation is not None:
        entry["coarse_translation_m"] = float(np.linalg.norm(coarse_translation))
    if icp_translation is not None:
        entry["icp_translation_m"] = float(np.linalg.norm(icp_translation))
    return entry


# ---------------------------------------------------------------------------
# Reference alignment (unchanged public API)
# ---------------------------------------------------------------------------


def align_hits_to_reference(hits, use_noisy_measurements=False):
    frame_hits, ordered_hits, sorted_frames = _group_hits_by_frame(hits)

    frame_translations = {}
    for frame, frame_group in frame_hits.items():
        frame_translations[frame] = _estimate_frame_translation_from_reference(
            frame_group,
            use_noisy_measurements=use_noisy_measurements,
        )

    corrected_points = []
    frame_point_count = {}
    for frame, hit in ordered_hits:
        translation = frame_translations.get(frame, np.zeros(3, dtype=float))
        corrected_points.append(
            tuple(
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                + translation
            )
        )
        frame_point_count[frame] = frame_point_count.get(frame, 0) + 1

    return {
        "alignment_mode": "framewise_reference_translation_baseline",
        "reference_mode": "clean_synthetic_reference_points",
        "corrected_points": corrected_points,
        "frame_translations": {
            str(frame): [float(value) for value in translation]
            for frame, translation in frame_translations.items()
        },
        "translation_summary": _translation_summary(
            frame_translations, frame_point_count
        ),
    }


# ---------------------------------------------------------------------------
# Two-stage blind Open3D ICP alignment (P1+P2+P3+P5)
# ---------------------------------------------------------------------------


def align_hits_blind_open3d_icp(
    hits,
    use_noisy_measurements=False,
    max_plausible_translation_m=5.0,
    icp_config=None,
):
    """Two-stage blind alignment: coarse median translation seed + Open3D
    ICP refinement.

    All translations are stored as **absolute** corrections: the vector to
    add to a raw (degraded) frame so it lands in the map frame anchored at
    the first frame.

    Stage 1 — Coarse seed
        ``_estimate_translation_to_point_cloud(raw_frame, target_cloud)``
        returns an absolute shift from raw space to target space.  Because
        the target cloud is already in the map frame (it consists of
        previously corrected frames), this shift is the absolute correction.

    Stage 2 — ICP refinement
        Open3D ICP is seeded from the coarse translation.  If it converges
        well, its result replaces the coarse seed.  If not, we keep the
        coarse seed directly (or fall back to carrying the previous frame's
        translation if the coarse seed is itself implausible).

    Gate: coarse-vs-refined divergence
        If the ICP result is further than ``max_coarse_icp_divergence_m``
        from the coarse seed, we prefer the coarse seed.  This prevents
        ICP from jumping to a local minimum far from the expected pose.
    """
    config = dict(DEFAULT_OPEN3D_ICP_CONFIG)
    if icp_config:
        config.update(icp_config)

    o3d, import_error = _try_import_open3d()
    if o3d is None:
        legacy = _legacy_blind_alignment(
            hits,
            use_noisy_measurements=use_noisy_measurements,
            max_plausible_translation_m=max_plausible_translation_m,
        )
        legacy["alignment_mode"] = "framewise_blind_translation_registration_fallback"
        legacy["warnings"] = [
            "Open3D blind alignment unavailable; falling back to legacy translation alignment: %s"
            % import_error
        ]
        return legacy

    frame_hits, ordered_hits, sorted_frames = _group_hits_by_frame(hits)
    absolute_translations = {}
    corrected_frame_points = {}
    per_frame_status = {}
    frame_transforms = {}
    frame_point_count = {}
    fitness_values = []
    rmse_values = []
    fallback_translation_frames = 0
    identity_fallback_frames = 0
    insufficient_points_frames = 0
    accepted_icp_frames = 0
    coarse_accepted_frames = 0
    last_target_frames = []

    target_window = int(config["target_window_frames"])
    max_target_window = int(config.get("max_target_window_frames", target_window))
    min_target_pts = int(config.get("min_target_points_for_window", 0))
    max_coarse_icp_div = float(config.get("max_coarse_icp_divergence_m", 0.60))

    for frame_index, frame in enumerate(sorted_frames):
        frame_pts = _frame_points(
            frame_hits[frame], use_noisy_measurements=use_noisy_measurements
        )
        frame_point_count[frame] = len(frame_pts)

        # ----- Anchor frame -----
        if frame_index == 0:
            absolute_translations[frame] = np.zeros(3, dtype=float)
            corrected_frame_points[frame] = frame_pts
            frame_transforms[frame] = np.eye(4, dtype=float)
            per_frame_status[str(frame)] = _frame_status(
                "anchor_frame",
                len(frame_pts),
                len(frame_pts),
                absolute_translations[frame],
                np.zeros(3, dtype=float),
                1.0,
                0.0,
            )
            continue

        previous_frame = sorted_frames[frame_index - 1]
        previous_translation = absolute_translations[previous_frame]

        # ----- Build target cloud (P3: adaptive window) -----
        target_points, target_frames = _build_target_points(
            corrected_frame_points,
            sorted_frames,
            frame_index,
            target_window,
            min_target_points=min_target_pts,
            max_window_size=max_target_window,
        )
        last_target_frames = target_frames

        # ----- Stage 1: Coarse translation seed -----
        # The coarse estimator returns an absolute translation from raw space
        # into the target (map) space.
        coarse_translation = _estimate_translation_to_point_cloud(
            frame_pts, target_points
        )

        # Sanity check the coarse estimate
        coarse_plausible = (
            float(np.linalg.norm(coarse_translation)) <= max_plausible_translation_m
        )
        if not coarse_plausible:
            # Coarse estimate is wild — use previous frame's translation as seed
            coarse_translation = previous_translation.copy()

        # ----- Stage 2: ICP refinement seeded from coarse -----
        icp_result = _estimate_translation_open3d(
            o3d,
            frame_pts,
            target_points,
            coarse_translation,
            config,
        )

        icp_translation = np.asarray(icp_result["translation"], dtype=float)
        delta_from_prev = icp_translation - previous_translation
        coarse_icp_divergence = float(
            np.linalg.norm(icp_translation - coarse_translation)
        )
        fallback_reason = None

        # ----- Gating: decide whether to accept ICP, coarse, or fallback -----

        # Insufficient points — record and move to fallback
        if icp_result["status"] in (
            "insufficient_source_points",
            "insufficient_target_points",
        ):
            insufficient_points_frames += 1
            fallback_reason = icp_result["status"]

        # Low fitness
        if (
            fallback_reason is None
            and icp_result["fitness"] is not None
            and icp_result["fitness"] < float(config["min_fitness"])
        ):
            fallback_reason = "low_fitness"

        # High inlier RMSE
        if (
            fallback_reason is None
            and icp_result["inlier_rmse"] is not None
            and icp_result["inlier_rmse"] > float(config["max_inlier_rmse_m"])
        ):
            fallback_reason = "high_inlier_rmse"

        # Delta from previous too large
        if fallback_reason is None and float(np.linalg.norm(delta_from_prev)) > float(
            config["max_delta_translation_m"]
        ):
            fallback_reason = "delta_translation_exceeded"

        # Coarse-vs-ICP divergence: ICP wandered too far from the coarse seed
        if fallback_reason is None and coarse_icp_divergence > max_coarse_icp_div:
            fallback_reason = "coarse_icp_divergence"

        # Total translation too large
        if fallback_reason is None and float(np.linalg.norm(icp_translation)) > min(
            float(config["max_total_translation_m"]),
            float(max_plausible_translation_m),
        ):
            fallback_reason = "total_translation_exceeded"

        # -------- Accept ICP result --------
        if fallback_reason is None:
            absolute_translations[frame] = icp_translation
            corrected_frame_points[frame] = frame_pts + icp_translation
            transform = np.eye(4, dtype=float)
            transform[:3, 3] = icp_translation
            frame_transforms[frame] = transform
            accepted_icp_frames += 1
            if icp_result["fitness"] is not None:
                fitness_values.append(float(icp_result["fitness"]))
            if icp_result["inlier_rmse"] is not None:
                rmse_values.append(float(icp_result["inlier_rmse"]))
            per_frame_status[str(frame)] = _frame_status(
                "accepted_icp",
                icp_result["source_point_count"],
                icp_result["target_point_count"],
                icp_translation,
                delta_from_prev,
                icp_result["fitness"],
                icp_result["inlier_rmse"],
                coarse_translation=coarse_translation,
                icp_translation=icp_translation,
            )
            continue

        # -------- Fallback 1: accept coarse translation directly --------
        # The coarse estimate is absolute (maps raw frame into map frame).
        # Check if it is plausible on its own.
        coarse_delta_from_prev = coarse_translation - previous_translation
        coarse_total_ok = float(np.linalg.norm(coarse_translation)) <= min(
            float(config["max_total_translation_m"]),
            float(max_plausible_translation_m),
        )
        coarse_delta_ok = float(np.linalg.norm(coarse_delta_from_prev)) <= float(
            config["max_delta_translation_m"]
        )

        if coarse_plausible and coarse_total_ok and coarse_delta_ok:
            absolute_translations[frame] = coarse_translation
            corrected_frame_points[frame] = frame_pts + coarse_translation
            transform = np.eye(4, dtype=float)
            transform[:3, 3] = coarse_translation
            frame_transforms[frame] = transform
            coarse_accepted_frames += 1
            fallback_translation_frames += 1
            per_frame_status[str(frame)] = _frame_status(
                "accepted_coarse_translation",
                icp_result["source_point_count"],
                icp_result["target_point_count"],
                coarse_translation,
                coarse_delta_from_prev,
                icp_result["fitness"],
                icp_result["inlier_rmse"],
                fallback_reason=fallback_reason,
                coarse_translation=coarse_translation,
                icp_translation=icp_translation,
            )
            continue

        # -------- Fallback 2: carry forward previous translation --------
        absolute_translations[frame] = previous_translation.copy()
        corrected_frame_points[frame] = frame_pts + previous_translation
        transform = np.eye(4, dtype=float)
        transform[:3, 3] = previous_translation
        frame_transforms[frame] = transform
        identity_fallback_frames += 1
        per_frame_status[str(frame)] = _frame_status(
            "identity_fallback",
            icp_result["source_point_count"],
            icp_result["target_point_count"],
            previous_translation,
            np.zeros(3, dtype=float),
            icp_result["fitness"],
            icp_result["inlier_rmse"],
            fallback_reason=fallback_reason,
            coarse_translation=coarse_translation,
            icp_translation=icp_translation,
        )

    # ----- Apply absolute translations to all hits -----
    corrected_points = []
    for frame, hit in ordered_hits:
        translation = absolute_translations.get(frame, np.zeros(3, dtype=float))
        corrected_points.append(
            tuple(
                _hit_point(hit, use_noisy_measurements=use_noisy_measurements)
                + translation
            )
        )

    return {
        "alignment_mode": "framewise_blind_open3d_translation_icp",
        "reference_mode": "rolling_recent_corrected_frames",
        "corrected_points": corrected_points,
        "frame_translations": {
            str(frame): [float(v) for v in translation]
            for frame, translation in absolute_translations.items()
        },
        "frame_transforms": {
            str(frame): [[float(value) for value in row] for row in transform]
            for frame, transform in frame_transforms.items()
        },
        "translation_summary": _translation_summary(
            absolute_translations, frame_point_count
        ),
        "registration_summary": {
            "backend": "open3d_icp",
            "accepted_icp_frames": accepted_icp_frames,
            "coarse_accepted_frames": coarse_accepted_frames,
            "fallback_translation_frames": fallback_translation_frames,
            "identity_fallback_frames": identity_fallback_frames,
            "insufficient_points_frames": insufficient_points_frames,
            "mean_fitness": float(np.mean(fitness_values)) if fitness_values else None,
            "mean_inlier_rmse": float(np.mean(rmse_values)) if rmse_values else None,
            "target_window_frames": int(config["target_window_frames"]),
            "max_target_window_frames": int(
                config.get("max_target_window_frames", target_window)
            ),
            "max_source_points": int(config["max_source_points"]),
            "max_target_points": int(config["max_target_points"]),
            "target_frames_last": last_target_frames,
        },
        "per_frame_status": per_frame_status,
        "warnings": [],
    }


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def align_hits_blind(
    hits,
    use_noisy_measurements=False,
    max_plausible_translation_m=5.0,
    method=DEFAULT_BLIND_ALIGNMENT_METHOD,
    icp_config=None,
):
    if method == "legacy_translation":
        return _legacy_blind_alignment(
            hits,
            use_noisy_measurements=use_noisy_measurements,
            max_plausible_translation_m=max_plausible_translation_m,
        )

    if method == "open3d_icp":
        return align_hits_blind_open3d_icp(
            hits,
            use_noisy_measurements=use_noisy_measurements,
            max_plausible_translation_m=max_plausible_translation_m,
            icp_config=icp_config,
        )

    raise ValueError("Unsupported blind alignment method: %s" % method)
