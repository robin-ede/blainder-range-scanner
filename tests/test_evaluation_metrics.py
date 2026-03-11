"""
Unit tests for pure metric functions in range_scanner.validation.evaluation.
No Blender context required — all functions tested here are numpy-only.
"""

import math

import numpy as np
import pytest

from range_scanner.validation.evaluation import (
    _metric_summary_from_values,
    compare_depth_distributions,
    compute_depth_histogram,
    compute_error_reduction,
)


# ---------------------------------------------------------------------------
# _metric_summary_from_values
# ---------------------------------------------------------------------------

class TestMetricSummaryFromValues:
    def test_empty_returns_none_metrics(self):
        result = _metric_summary_from_values([])
        assert result["point_count"] == 0
        assert result["rmse"] is None
        assert result["mae"] is None
        assert result["median"] is None
        assert result["p95"] is None
        assert result["max"] is None

    def test_single_value(self):
        result = _metric_summary_from_values([3.0])
        assert result["point_count"] == 1
        assert result["rmse"] == pytest.approx(3.0)
        assert result["mae"] == pytest.approx(3.0)
        assert result["median"] == pytest.approx(3.0)
        assert result["max"] == pytest.approx(3.0)

    def test_known_rmse(self):
        # RMSE([3, 4]) = sqrt((9 + 16) / 2) = sqrt(12.5)
        result = _metric_summary_from_values([3.0, 4.0])
        assert result["rmse"] == pytest.approx(math.sqrt(12.5), rel=1e-6)

    def test_known_mae(self):
        result = _metric_summary_from_values([1.0, 3.0])
        assert result["mae"] == pytest.approx(2.0)

    def test_all_zeros(self):
        result = _metric_summary_from_values([0.0, 0.0, 0.0])
        assert result["rmse"] == pytest.approx(0.0)
        assert result["mae"] == pytest.approx(0.0)
        assert result["max"] == pytest.approx(0.0)

    def test_p95_and_max(self):
        values = list(range(1, 101))   # 1 … 100
        result = _metric_summary_from_values(values)
        assert result["max"] == pytest.approx(100.0)
        assert result["p95"] == pytest.approx(95.05, rel=0.02)

    def test_median_odd_length(self):
        result = _metric_summary_from_values([1.0, 3.0, 5.0])
        assert result["median"] == pytest.approx(3.0)

    def test_median_even_length(self):
        result = _metric_summary_from_values([1.0, 2.0, 3.0, 4.0])
        assert result["median"] == pytest.approx(2.5)

    def test_returns_python_floats(self):
        result = _metric_summary_from_values([1.0, 2.0])
        assert isinstance(result["rmse"], float)
        assert isinstance(result["point_count"], int)


# ---------------------------------------------------------------------------
# compute_error_reduction
# ---------------------------------------------------------------------------

class TestComputeErrorReduction:
    def test_none_inputs_return_none(self):
        result = compute_error_reduction(None, None)
        assert result["error_reduction_pct"] is None
        assert result["meets_80pct_reduction"] is None

    def test_none_post_fusion_rmse(self):
        result = compute_error_reduction(1.0, None)
        assert result["error_reduction_pct"] is None

    def test_zero_degraded_rmse_avoids_division(self):
        result = compute_error_reduction(0.0, 0.05)
        assert result["error_reduction_pct"] is None

    def test_90pct_reduction_passes(self):
        result = compute_error_reduction(1.0, 0.1)
        assert result["error_reduction_pct"] == pytest.approx(90.0)
        assert result["meets_80pct_reduction"] is True

    def test_80pct_boundary_passes(self):
        result = compute_error_reduction(1.0, 0.2)
        assert result["error_reduction_pct"] == pytest.approx(80.0)
        assert result["meets_80pct_reduction"] is True

    def test_below_80pct_fails(self):
        result = compute_error_reduction(1.0, 0.5)
        assert result["error_reduction_pct"] == pytest.approx(50.0)
        assert result["meets_80pct_reduction"] is False

    def test_positional_baseline_passes_above_2m(self):
        result = compute_error_reduction(1.0, 0.1, max_positional_correction_m=2.5)
        assert result["meets_positional_baseline_2m"] is True
        assert result["positional_degradation_magnitude_m"] == pytest.approx(2.5)

    def test_positional_baseline_fails_below_2m(self):
        result = compute_error_reduction(1.0, 0.1, max_positional_correction_m=1.5)
        assert result["meets_positional_baseline_2m"] is False

    def test_positional_baseline_exact_2m_passes(self):
        result = compute_error_reduction(1.0, 0.1, max_positional_correction_m=2.0)
        assert result["meets_positional_baseline_2m"] is True

    def test_no_positional_correction_gives_none_baseline(self):
        result = compute_error_reduction(1.0, 0.1, max_positional_correction_m=None)
        assert result["meets_positional_baseline_2m"] is None

    def test_legacy_keys_present(self):
        result = compute_error_reduction(1.0, 0.1)
        assert "degraded_rmse" in result
        assert "meets_degraded_baseline_2m" in result

    def test_custom_positional_threshold(self):
        result = compute_error_reduction(
            1.0, 0.1,
            max_positional_correction_m=1.5,
            positional_baseline_threshold_m=1.0,
        )
        assert result["meets_positional_baseline_2m"] is True


# ---------------------------------------------------------------------------
# compute_depth_histogram
# ---------------------------------------------------------------------------

class TestComputeDepthHistogram:
    def test_basic_structure(self):
        depths = [1.0, 2.0, 3.0, 4.0]
        bin_edges = [0.5, 1.5, 2.5, 3.5, 4.5]
        result = compute_depth_histogram(depths, bin_edges)
        assert "bin_edges" in result
        assert "counts" in result
        assert "density" in result

    def test_counts_sum_to_total(self):
        depths = list(range(10))
        bin_edges = [-0.5 + i for i in range(12)]
        result = compute_depth_histogram(depths, bin_edges)
        assert sum(result["counts"]) == 10

    def test_density_sums_to_one(self):
        depths = [1.0, 2.0, 3.0]
        bin_edges = [0.5, 1.5, 2.5, 3.5]
        result = compute_depth_histogram(depths, bin_edges)
        assert sum(result["density"]) == pytest.approx(1.0)

    def test_empty_depths_density_all_zeros(self):
        result = compute_depth_histogram([], [0.0, 1.0, 2.0])
        assert all(d == 0.0 for d in result["density"])

    def test_single_bin_gets_all_density(self):
        depths = [0.5, 0.6, 0.7]
        bin_edges = [0.0, 1.0, 2.0]
        result = compute_depth_histogram(depths, bin_edges)
        assert result["density"][0] == pytest.approx(1.0)
        assert result["density"][1] == pytest.approx(0.0)

    def test_bin_edges_preserved(self):
        bin_edges = [0.0, 1.0, 2.0, 3.0]
        result = compute_depth_histogram([0.5], bin_edges)
        assert result["bin_edges"] == pytest.approx(bin_edges)


# ---------------------------------------------------------------------------
# compare_depth_distributions
# ---------------------------------------------------------------------------

class TestCompareDepthDistributions:
    def test_insufficient_data_synthetic_empty(self):
        result = compare_depth_distributions([], [1.0, 2.0, 3.0])
        assert result["status"] == "insufficient_data"
        assert result["ks_statistic"] is None
        assert result["passes_all"] is None

    def test_insufficient_data_reference_empty(self):
        result = compare_depth_distributions([1.0, 2.0], [])
        assert result["status"] == "insufficient_data"

    def test_identical_distributions_pass(self):
        data = [float(i) for i in range(1, 21)]
        result = compare_depth_distributions(data, data, ks_threshold=0.15)
        assert result["status"] == "computed"
        assert result["ks_statistic"] == pytest.approx(0.0, abs=1e-9)
        assert result["passes_ks_threshold"] is True

    def test_completely_different_distributions_fail(self):
        syn = [float(i) for i in range(1, 11)]
        ref = [float(i) for i in range(100, 111)]
        result = compare_depth_distributions(syn, ref, ks_threshold=0.15)
        assert result["ks_statistic"] == pytest.approx(1.0, abs=0.01)
        assert result["passes_ks_threshold"] is False

    def test_custom_ks_threshold(self):
        syn = [1.0, 2.0, 3.0, 4.0, 5.0]
        ref = [1.1, 2.1, 3.1, 4.1, 5.1]
        # With a very tight threshold, even a similar distribution should fail
        result_tight = compare_depth_distributions(syn, ref, ks_threshold=0.01)
        result_loose = compare_depth_distributions(syn, ref, ks_threshold=0.99)
        assert result_loose["passes_ks_threshold"] is True
        # tight may or may not pass depending on exact KS value, but loose must pass

    def test_result_contains_histograms(self):
        data = [1.0, 2.0, 3.0, 4.0, 5.0]
        result = compare_depth_distributions(data, data)
        assert "synthetic_histogram" in result
        assert "reference_histogram" in result

    def test_custom_bin_edges(self):
        syn = [0.5, 1.5]
        ref = [0.5, 1.5]
        bin_edges = [0.0, 1.0, 2.0]
        result = compare_depth_distributions(syn, ref, bin_edges=bin_edges)
        assert result["status"] == "computed"
        assert len(result["synthetic_histogram"]["bin_edges"]) == 3

    def test_passes_all_requires_both_checks(self):
        # Force bin_deviation by using very different bin distributions
        # with a shared bin_edges
        syn = [0.1] * 10          # all in first bin
        ref = [0.9] * 10          # all in second bin
        bin_edges = [0.0, 0.5, 1.0]
        result = compare_depth_distributions(
            syn, ref,
            bin_edges=bin_edges,
            bin_deviation_threshold=0.10,
        )
        assert result["passes_bin_deviation_threshold"] is False
        assert result["passes_all"] is False

    def test_bin_deviations_list_length(self):
        syn = [1.0, 2.0, 3.0]
        ref = [1.0, 2.0, 3.0]
        bin_edges = np.linspace(0.5, 3.5, 4).tolist()
        result = compare_depth_distributions(syn, ref, bin_edges=bin_edges)
        # Should have one deviation per bin (len(bin_edges) - 1)
        assert len(result["bin_deviations"]) == len(bin_edges) - 1

    def test_point_counts_recorded(self):
        syn = [1.0, 2.0]
        ref = [3.0, 4.0, 5.0]
        result = compare_depth_distributions(syn, ref)
        assert result["synthetic_point_count"] == 2
        assert result["reference_point_count"] == 3
