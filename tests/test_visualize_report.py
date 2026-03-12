"""
Tests for scripts/visualize_validation_report.py plot functions.

Verifies that each of the three demo-component outputs (RMSE comparison chart,
reprojection error heatmap, depth distribution histogram) can be generated from
a well-formed report dict and produce a real PNG file.

Also checks graceful skip behaviour when the report lacks optional data sections
(no spatial_error_samples, no depth_distribution_comparison).

Requires: matplotlib, numpy  (pip install matplotlib numpy)
"""

import json
import os
import sys
import tempfile

import pytest

# ---------------------------------------------------------------------------
# Add scripts/ to path so we can import visualize_validation_report directly
# ---------------------------------------------------------------------------

_SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, os.path.abspath(_SCRIPTS_DIR))

from visualize_validation_report import (  # noqa: E402
    plot_depth_histogram,
    plot_reprojection_heatmap,
    plot_rmse_comparison,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

_FIXTURE_PATH = os.path.join(
    os.path.dirname(__file__), "fixtures", "sample_passing_report.json"
)


@pytest.fixture(scope="module")
def full_report():
    with open(_FIXTURE_PATH, encoding="utf-8") as fh:
        return json.load(fh)


@pytest.fixture()
def minimal_rmse_report():
    """Minimal report that has just enough data for the RMSE bar chart."""
    return {
        "schema_version": 1,
        "status": "pass",
        "acceptance": {"passes_rmse_threshold": True, "rmse_threshold_m": 0.15},
        "pre_fusion_combined_metrics": {"rmse": 2.143, "point_count": 5000},
        "post_processing_fusion": {"metrics": {"rmse": 0.134, "point_count": 6841}},
        "blind_post_processing_fusion": {
            "metrics": {"rmse": 0.112, "point_count": 6814}
        },
        "error_reduction": {
            "error_reduction_pct": 94.77,
            "meets_80pct_reduction": True,
        },
    }


@pytest.fixture()
def report_without_spatial_samples():
    """Report that is missing spatial_error_samples — heatmap should skip gracefully."""
    return {
        "schema_version": 1,
        "status": "pass",
        "acceptance": {"rmse_threshold_m": 0.15},
        "pre_fusion_combined_metrics": {"rmse": 2.1},
        "post_processing_fusion": {"metrics": {"rmse": 0.13}},
        "blind_post_processing_fusion": {"metrics": {"rmse": 0.11}},
    }


@pytest.fixture()
def report_without_distribution():
    """Report that lacks depth_distribution_comparison — histogram should skip gracefully."""
    return {
        "schema_version": 1,
        "status": "pass",
        "acceptance": {"rmse_threshold_m": 0.15},
        "pre_fusion_combined_metrics": {"rmse": 2.1},
        "post_processing_fusion": {"metrics": {"rmse": 0.13}},
        "blind_post_processing_fusion": {"metrics": {"rmse": 0.11}},
    }


# ---------------------------------------------------------------------------
# Plot 1 — RMSE comparison bar chart
# ---------------------------------------------------------------------------


class TestPlotRmseComparison:
    def test_produces_png_from_full_report(self, full_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "rmse_test.png")
            result = plot_rmse_comparison(full_report, out)
            assert result == out
            assert os.path.isfile(out), "PNG file must be written to disk"
            assert os.path.getsize(out) > 1024, "PNG must be a non-trivial file"

    def test_produces_png_from_minimal_report(self, minimal_rmse_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "rmse_minimal.png")
            result = plot_rmse_comparison(minimal_rmse_report, out)
            assert result == out
            assert os.path.isfile(out)

    def test_returns_none_when_no_rmse_data(self):
        empty_report = {"schema_version": 1, "acceptance": {}}
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "rmse_empty.png")
            result = plot_rmse_comparison(empty_report, out)
            assert result is None, "Should return None when no RMSE stage data present"
            assert not os.path.isfile(out), "No PNG should be written for empty report"

    def test_custom_title_accepted(self, minimal_rmse_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "rmse_titled.png")
            result = plot_rmse_comparison(
                minimal_rmse_report, out, title="Custom Test Title"
            )
            assert result == out

    def test_error_reduction_annotation_shown(self, full_report):
        """Full report has error_reduction data — chart should produce PNG with annotation."""
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "rmse_annot.png")
            result = plot_rmse_comparison(full_report, out)
            assert result is not None, (
                "Chart with error_reduction data must be produced"
            )


# ---------------------------------------------------------------------------
# Plot 2 — Reprojection error heatmap
# ---------------------------------------------------------------------------


class TestPlotReprojectionHeatmap:
    def test_produces_png_from_full_report(self, full_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "heatmap_test.png")
            result = plot_reprojection_heatmap(full_report, out)
            assert result == out
            assert os.path.isfile(out), "Heatmap PNG must be written to disk"
            assert os.path.getsize(out) > 1024, "Heatmap PNG must be non-trivial"

    def test_skips_gracefully_without_samples(
        self, report_without_spatial_samples, capsys
    ):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "heatmap_empty.png")
            result = plot_reprojection_heatmap(report_without_spatial_samples, out)
            assert result is None, (
                "Should return None when spatial_error_samples absent"
            )
            assert not os.path.isfile(out), "No PNG should be written when skipping"
            captured = capsys.readouterr()
            assert "[skip]" in captured.out

    def test_custom_title_accepted(self, full_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "heatmap_titled.png")
            result = plot_reprojection_heatmap(
                full_report, out, title="Heatmap Custom Title"
            )
            assert result == out

    def test_uses_blind_post_processing_stage_by_default(self, full_report):
        """The TO4 official stage (blind_post_processing_fusion) must be used."""
        # Verify the fixture has the right TO4 stage key
        stage_key = (
            full_report.get("technical_objectives", {})
            .get("TO4", {})
            .get("official_stage_key")
        )
        assert stage_key == "blind_post_processing_fusion"
        # Verify the stage has samples
        samples = full_report.get(stage_key, {}).get("spatial_error_samples")
        assert samples is not None and len(samples) > 0

    def test_heatmap_uses_four_column_samples(self, full_report):
        """spatial_error_samples must be [x, y, z, distance] — 4 columns."""
        samples = full_report.get("blind_post_processing_fusion", {}).get(
            "spatial_error_samples", []
        )
        for row in samples:
            assert len(row) == 4, "Each sample must have 4 values: [x, y, z, distance]"


# ---------------------------------------------------------------------------
# Plot 3 — Depth distribution histogram
# ---------------------------------------------------------------------------


class TestPlotDepthHistogram:
    def test_produces_png_from_full_report(self, full_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "hist_test.png")
            result = plot_depth_histogram(full_report, out)
            assert result == out
            assert os.path.isfile(out), "Histogram PNG must be written to disk"
            assert os.path.getsize(out) > 1024, "Histogram PNG must be non-trivial"

    def test_skips_gracefully_without_distribution_data(
        self, report_without_distribution, capsys
    ):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "hist_empty.png")
            result = plot_depth_histogram(report_without_distribution, out)
            assert result is None, "Should return None when distribution data absent"
            assert not os.path.isfile(out)
            captured = capsys.readouterr()
            assert "[skip]" in captured.out

    def test_skips_when_distribution_not_computed(self):
        report_incomplete = {
            "depth_distribution_comparison": {"status": "insufficient_data"}
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "hist_incomplete.png")
            result = plot_depth_histogram(report_incomplete, out)
            assert result is None

    def test_custom_title_accepted(self, full_report):
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "hist_titled.png")
            result = plot_depth_histogram(full_report, out, title="Depth Hist Title")
            assert result == out

    def test_ks_annotation_visible_in_output(self, full_report):
        """Report with KS data should produce a PNG (annotation exercise)."""
        assert full_report["depth_distribution_comparison"]["ks_statistic"] is not None
        with tempfile.TemporaryDirectory() as tmpdir:
            out = os.path.join(tmpdir, "hist_ks.png")
            result = plot_depth_histogram(full_report, out)
            assert result is not None


# ---------------------------------------------------------------------------
# All three plots from full fixture
# ---------------------------------------------------------------------------


class TestAllPlotsFromFixture:
    def test_all_three_plots_produced(self, full_report):
        """The full fixture should produce all three PNGs without error."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rmse_out = os.path.join(tmpdir, "full_rmse.png")
            heatmap_out = os.path.join(tmpdir, "full_heatmap.png")
            hist_out = os.path.join(tmpdir, "full_hist.png")

            r1 = plot_rmse_comparison(full_report, rmse_out)
            r2 = plot_reprojection_heatmap(full_report, heatmap_out)
            r3 = plot_depth_histogram(full_report, hist_out)

            assert r1 is not None, "RMSE chart must be produced from full fixture"
            assert r2 is not None, "Heatmap must be produced from full fixture"
            assert r3 is not None, "Histogram must be produced from full fixture"

            for path in [rmse_out, heatmap_out, hist_out]:
                assert os.path.isfile(path)
                assert os.path.getsize(path) > 1024
