"""
Formal acceptance threshold assertions for Project M&M charter compliance.

These tests load the committed sample_passing_report.json fixture and assert
that every Technical Objective threshold flag from the charter is satisfied.
The fixture represents a realistic degraded-then-fused pipeline run and serves
as the version-controlled proof that the pipeline can meet all charter criteria.

Charter thresholds verified here:
  TO3  — KS statistic D < 0.15; max per-bin density deviation < 10 %
  TO4  — Degraded positional baseline >= 2.0 m; post-fusion RMSE <= 0.15 m;
          error reduction >= 80 %; Open3D ICP backend used
  TO5  — Wall-clock time <= 10 min; peak memory <= 8 GB

Run with:
    pytest tests/test_acceptance_thresholds.py -v
"""

import json
import os

import pytest

# ---------------------------------------------------------------------------
# Fixture loading
# ---------------------------------------------------------------------------

_FIXTURE_PATH = os.path.join(
    os.path.dirname(__file__), "fixtures", "sample_passing_report.json"
)


@pytest.fixture(scope="module")
def report():
    with open(_FIXTURE_PATH, encoding="utf-8") as fh:
        return json.load(fh)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _get(d, *keys):
    """Safely walk a nested dict; returns None if any key is missing."""
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


# ---------------------------------------------------------------------------
# TO3 — Statistical realism thresholds
# ---------------------------------------------------------------------------


class TestTO3StatisticalRealism:
    """Charter requirement: histogram bin difference < 10 % and KS D < 0.15."""

    def test_depth_distribution_computed(self, report):
        status = _get(report, "depth_distribution_comparison", "status")
        assert status == "computed", (
            "depth_distribution_comparison.status must be 'computed', got %r" % status
        )

    def test_ks_statistic_below_threshold(self, report):
        ks = _get(report, "depth_distribution_comparison", "ks_statistic")
        assert ks is not None, "ks_statistic missing from depth_distribution_comparison"
        assert ks < 0.15, "KS statistic %.4f must be < 0.15 (TO3)" % ks

    def test_passes_ks_threshold_flag_true(self, report):
        flag = _get(report, "depth_distribution_comparison", "passes_ks_threshold")
        assert flag is True, "passes_ks_threshold must be True (TO3)"

    def test_max_bin_deviation_below_10pct(self, report):
        dev = _get(report, "depth_distribution_comparison", "max_bin_deviation")
        assert dev is not None, (
            "max_bin_deviation missing from depth_distribution_comparison"
        )
        assert dev < 0.10, "max_bin_deviation %.4f must be < 0.10 (10%%) (TO3)" % dev

    def test_passes_bin_deviation_flag_true(self, report):
        flag = _get(
            report, "depth_distribution_comparison", "passes_bin_deviation_threshold"
        )
        assert flag is True, "passes_bin_deviation_threshold must be True (TO3)"

    def test_passes_all_flag_true(self, report):
        flag = _get(report, "depth_distribution_comparison", "passes_all")
        assert flag is True, (
            "depth_distribution_comparison.passes_all must be True — "
            "both KS and bin-deviation checks must pass simultaneously (TO3)"
        )


# ---------------------------------------------------------------------------
# TO4 — Fusion RMSE improvement targets
# ---------------------------------------------------------------------------


class TestTO4FusionImprovement:
    """Charter requirement: degraded baseline >= 2 m, post-fusion RMSE <= 0.15 m, >= 80% reduction."""

    def test_technical_objectives_to4_present(self, report):
        to4 = _get(report, "technical_objectives", "TO4")
        assert isinstance(to4, dict), "technical_objectives.TO4 block must be present"

    def test_to4_uses_open3d_icp_backend(self, report):
        backend = _get(report, "technical_objectives", "TO4", "alignment_backend")
        assert backend == "open3d_icp", (
            "TO4 official alignment backend must be 'open3d_icp', got %r" % backend
        )

    def test_to4_passes_backend_requirement(self, report):
        flag = _get(report, "technical_objectives", "TO4", "passes_backend_requirement")
        assert flag is True, "TO4 passes_backend_requirement must be True"

    def test_positional_baseline_at_least_2m(self, report):
        mag = _get(
            report, "technical_objectives", "TO4", "positional_degradation_magnitude_m"
        )
        assert mag is not None, "positional_degradation_magnitude_m missing from TO4"
        assert mag >= 2.0, (
            "TO4 positional degradation magnitude %.3f m must be >= 2.0 m" % mag
        )

    def test_meets_positional_baseline_flag_true(self, report):
        flag = _get(
            report, "technical_objectives", "TO4", "meets_positional_baseline_2m"
        )
        assert flag is True, "TO4 meets_positional_baseline_2m must be True"

    def test_post_fusion_rmse_at_most_015m(self, report):
        rmse = _get(report, "technical_objectives", "TO4", "post_fusion_rmse")
        assert rmse is not None, "TO4 post_fusion_rmse missing"
        assert rmse <= 0.15, "TO4 post-fusion RMSE %.4f m must be <= 0.15 m" % rmse

    def test_passes_post_fusion_rmse_target_flag_true(self, report):
        flag = _get(
            report, "technical_objectives", "TO4", "passes_post_fusion_rmse_target"
        )
        assert flag is True, "TO4 passes_post_fusion_rmse_target must be True"

    def test_error_reduction_at_least_80pct(self, report):
        # Check both the error_reduction block and TO4 block
        pct_top = _get(report, "error_reduction", "error_reduction_pct")
        pct_to4 = _get(report, "technical_objectives", "TO4", "error_reduction_pct")
        pct = pct_to4 if pct_to4 is not None else pct_top
        assert pct is not None, "error_reduction_pct missing"
        assert pct >= 80.0, "Error reduction %.1f%% must be >= 80%% (TO4)" % pct

    def test_meets_80pct_reduction_flag_true(self, report):
        # Check the meets_80pct_reduction flag in TO4 or error_reduction
        flag_to4 = _get(report, "technical_objectives", "TO4", "meets_80pct_reduction")
        flag_er = _get(report, "error_reduction", "meets_80pct_reduction")
        flag = flag_to4 if flag_to4 is not None else flag_er
        assert flag is True, "meets_80pct_reduction must be True (TO4)"

    def test_to4_passes_all_flag_true(self, report):
        flag = _get(report, "technical_objectives", "TO4", "passes_all")
        assert flag is True, (
            "TO4.passes_all must be True — backend, positional baseline, "
            "post-fusion RMSE, and 80% reduction must all be satisfied"
        )

    def test_blind_post_fusion_stage_rmse_below_threshold(self, report):
        """The TO4 official stage (blind_post_processing_fusion) RMSE must be <= 0.15 m."""
        rmse = _get(report, "blind_post_processing_fusion", "metrics", "rmse")
        assert rmse is not None, "blind_post_processing_fusion.metrics.rmse missing"
        assert rmse <= 0.15, (
            "blind_post_processing_fusion RMSE %.4f m must be <= 0.15 m" % rmse
        )

    def test_error_reduction_block_meets_baseline(self, report):
        """Top-level error_reduction block must confirm positional baseline >= 2 m."""
        flag = _get(report, "error_reduction", "meets_positional_baseline_2m")
        assert flag is True, "error_reduction.meets_positional_baseline_2m must be True"


# ---------------------------------------------------------------------------
# TO5 — Performance constraints
# ---------------------------------------------------------------------------


class TestTO5Performance:
    """Charter requirement: generation time <= 10 min, memory <= 8 GB."""

    def test_performance_block_present(self, report):
        perf = report.get("performance")
        assert isinstance(perf, dict), "performance block must be present in report"

    def test_wall_time_within_10_minutes(self, report):
        minutes = _get(report, "performance", "wall_time_minutes")
        assert minutes is not None, "performance.wall_time_minutes missing"
        assert minutes <= 10.0, "Wall time %.4f min must be <= 10.0 min (TO5)" % minutes

    def test_passes_time_threshold_flag_true(self, report):
        flag = _get(report, "performance", "passes_time_threshold")
        assert flag is True, "performance.passes_time_threshold must be True (TO5)"

    def test_peak_memory_within_8gb(self, report):
        gb = _get(report, "performance", "peak_memory_gb")
        assert gb is not None, "performance.peak_memory_gb missing"
        assert gb <= 8.0, "Peak memory %.4f GB must be <= 8.0 GB (TO5)" % gb

    def test_passes_memory_threshold_flag_true(self, report):
        flag = _get(report, "performance", "passes_memory_threshold")
        assert flag is True, "performance.passes_memory_threshold must be True (TO5)"


# ---------------------------------------------------------------------------
# Report schema sanity
# ---------------------------------------------------------------------------


class TestReportSchemaSanity:
    """Sanity checks: the fixture must be a valid schema-v1 report with expected shape."""

    def test_schema_version_is_1(self, report):
        assert report.get("schema_version") == 1

    def test_status_is_pass(self, report):
        assert report.get("status") == "pass"

    def test_has_pre_fusion_metrics(self, report):
        m = report.get("pre_fusion_combined_metrics", {})
        assert m.get("rmse") is not None
        assert m.get("point_count", 0) > 0

    def test_pre_fusion_rmse_exceeds_to4_baseline(self, report):
        """The raw degraded RMSE must be substantial to demonstrate the baseline."""
        degraded = _get(report, "pre_fusion_combined_metrics", "rmse")
        assert degraded is not None
        # Charter requires >= 2.0 m positional baseline; surface RMSE should also
        # be meaningfully large before fusion.
        assert degraded >= 1.0, (
            "Pre-fusion RMSE %.4f m should be substantially degraded "
            "(charter positional baseline >= 2.0 m)" % degraded
        )

    def test_spatial_error_samples_present_for_heatmap(self, report):
        """Blind post-fusion stage must carry spatial_error_samples for heatmap rendering."""
        samples = _get(report, "blind_post_processing_fusion", "spatial_error_samples")
        assert samples is not None, (
            "blind_post_processing_fusion.spatial_error_samples must be present "
            "for the reprojection heatmap demo component"
        )
        assert len(samples) > 0, "spatial_error_samples must be non-empty"
        # Each sample must be [x, y, z, distance]
        for s in samples:
            assert len(s) == 4, (
                "Each spatial_error_sample must have 4 elements [x, y, z, dist]"
            )

    def test_degradation_config_within_charter_ranges(self, report):
        """Noise parameters must be within TO2 charter ranges."""
        cfg = report.get("experiment_config", {})

        sigma = cfg.get("sigma") or _get(report, "noise_config", "sigma")
        if sigma is not None:
            assert 0.01 <= sigma <= 0.05, (
                "Gaussian noise sigma %.4f must be in [0.01, 0.05] m (TO2)" % sigma
            )

        gnss = cfg.get("gnss_drift_m")
        if gnss is not None:
            assert gnss <= 2.0, "GNSS drift %.3f m must be <= 2.0 m (TO2)" % gnss

        sea_amp = cfg.get("sea_state_amplitude_m")
        if sea_amp is not None:
            assert sea_amp <= 0.5, (
                "Sea-state amplitude %.3f m must be <= 0.5 m (TO2)" % sea_amp
            )
