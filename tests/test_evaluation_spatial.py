"""
Unit tests for evaluate_points_against_targets and
evaluate_hits_against_targets.

The BVH-tree machinery is patched so these tests run without Blender.
"""

import pytest
from unittest.mock import MagicMock, patch

from tests.conftest import make_hit
from range_scanner.validation.evaluation import (
    evaluate_points_against_targets,
    evaluate_hits_against_targets,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_context():
    ctx = MagicMock()
    ctx.evaluated_depsgraph_get.return_value = MagicMock()
    return ctx


def _mock_targets(n=1):
    return [MagicMock() for _ in range(n)]


# ---------------------------------------------------------------------------
# evaluate_points_against_targets
# ---------------------------------------------------------------------------

class TestEvaluatePointsAgainstTargets:
    """Tests with _build_bvh_trees and _nearest_distance patched."""

    def _run(self, points, distance_value=0.05, **kwargs):
        ctx = _mock_context()
        targets = _mock_targets()
        with patch("range_scanner.validation.evaluation._build_bvh_trees",
                   return_value=[MagicMock()]):
            with patch("range_scanner.validation.evaluation._nearest_distance",
                       return_value=distance_value):
                return evaluate_points_against_targets(
                    ctx, points, targets, **kwargs
                )

    def test_uniform_distances_rmse_equals_distance(self):
        points = [(1.0, 2.0, 3.0)] * 5
        result = self._run(points, distance_value=0.05)
        assert result["metrics"]["rmse"] == pytest.approx(0.05)
        assert result["metrics"]["point_count"] == 5

    def test_passes_threshold(self):
        result = self._run([(0.0, 0.0, 0.0)], distance_value=0.10,
                           rmse_threshold=0.15)
        assert result["acceptance"]["passes_rmse_threshold"] is True

    def test_fails_threshold(self):
        result = self._run([(0.0, 0.0, 0.0)], distance_value=0.20,
                           rmse_threshold=0.15)
        assert result["acceptance"]["passes_rmse_threshold"] is False

    def test_threshold_stored_in_result(self):
        result = self._run([(0.0, 0.0, 0.0)], rmse_threshold=0.25)
        assert result["acceptance"]["rmse_threshold_m"] == pytest.approx(0.25)

    def test_empty_points_returns_none_rmse(self):
        ctx = _mock_context()
        targets = _mock_targets()
        with patch("range_scanner.validation.evaluation._build_bvh_trees",
                   return_value=[]):
            result = evaluate_points_against_targets(ctx, [], targets)
        assert result["metrics"]["rmse"] is None
        assert result["metrics"]["point_count"] == 0

    def test_regional_metrics_populated_above_water(self):
        # All points above water surface (z=0)
        points = [(0.0, 0.0, 1.0), (1.0, 0.0, 2.0)]
        result = self._run(points, distance_value=0.05,
                           water_surface_height=0.0)
        above = result["regional_metrics"]["above_water"]
        assert above["point_count"] == 2
        assert above["rmse"] == pytest.approx(0.05)

    def test_regional_metrics_populated_below_water(self):
        points = [(0.0, 0.0, -1.0), (1.0, 0.0, -2.0)]
        result = self._run(points, distance_value=0.08,
                           water_surface_height=0.0)
        below = result["regional_metrics"]["below_water"]
        assert below["point_count"] == 2

    def test_no_spatial_samples_by_default(self):
        result = self._run([(0.0, 0.0, 0.0)] * 3)
        assert "spatial_error_samples" not in result

    def test_spatial_samples_when_requested(self):
        points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        result = self._run(points, store_spatial_samples=True)
        assert "spatial_error_samples" in result
        samples = result["spatial_error_samples"]
        assert len(samples) == 2
        # Each sample: [x, y, z, distance]
        assert samples[0][0] == pytest.approx(1.0)
        assert samples[0][1] == pytest.approx(2.0)
        assert samples[0][2] == pytest.approx(3.0)
        assert samples[0][3] == pytest.approx(0.05)

    def test_spatial_samples_downsampled_to_max(self):
        points = [(float(i), 0.0, 0.0) for i in range(100)]
        result = self._run(points, store_spatial_samples=True,
                           max_spatial_samples=10)
        samples = result["spatial_error_samples"]
        assert len(samples) <= 10

    def test_nearest_distance_none_skips_point(self):
        """Points for which _nearest_distance returns None are excluded."""
        ctx = _mock_context()
        targets = _mock_targets()
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        # Return None for first call, 0.05 for second
        side_effects = [None, 0.05]
        with patch("range_scanner.validation.evaluation._build_bvh_trees",
                   return_value=[MagicMock()]):
            with patch("range_scanner.validation.evaluation._nearest_distance",
                       side_effect=side_effects):
                result = evaluate_points_against_targets(ctx, points, targets)
        assert result["metrics"]["point_count"] == 1
        assert result["metrics"]["rmse"] == pytest.approx(0.05)


# ---------------------------------------------------------------------------
# evaluate_hits_against_targets
# ---------------------------------------------------------------------------

class TestEvaluateHitsAgainstTargets:
    def _run(self, hits, distance_value=0.05, **kwargs):
        ctx = _mock_context()
        targets = _mock_targets()
        with patch("range_scanner.validation.evaluation._build_bvh_trees",
                   return_value=[MagicMock()]):
            with patch("range_scanner.validation.evaluation._nearest_distance",
                       return_value=distance_value):
                return evaluate_hits_against_targets(
                    ctx, hits, targets, **kwargs
                )

    def test_basic_metrics(self):
        hits = [make_hit(0.0, 0.0, 0.0) for _ in range(4)]
        result = self._run(hits, distance_value=0.10)
        assert result["metrics"]["rmse"] == pytest.approx(0.10)

    def test_total_hit_count_recorded(self):
        hits = [make_hit(float(i), 0.0, 0.0) for i in range(7)]
        result = self._run(hits)
        assert result["hit_counts"]["total"] == 7

    def test_per_sensor_counts(self):
        lidar_hits = [make_hit(0.0, 0.0, 0.0, sensor_id="lidar") for _ in range(3)]
        sonar_hits = [make_hit(1.0, 0.0, 0.0, sensor_id="sonar") for _ in range(2)]
        result = self._run(lidar_hits + sonar_hits)
        assert result["hit_counts"]["by_sensor"]["lidar"] == 3
        assert result["hit_counts"]["by_sensor"]["sonar"] == 2

    def test_per_sensor_metrics_split(self):
        hits = (
            [make_hit(0.0, 0.0, 0.0, sensor_id="lidar")] * 3
            + [make_hit(1.0, 0.0, 0.0, sensor_id="sonar")] * 2
        )
        result = self._run(hits)
        assert "lidar" in result["per_sensor_pre_fusion_metrics"]
        assert "sonar" in result["per_sensor_pre_fusion_metrics"]

    def test_uses_noise_location_when_requested(self):
        """With use_noisy_measurements=True, noiseLocation should be used."""
        # noise location is shifted; we verify the hit's noisy point is passed
        hit = make_hit(0.0, 0.0, 0.0,
                       noise_x=5.0, noise_y=5.0, noise_z=5.0)
        ctx = _mock_context()
        targets = _mock_targets()
        captured_points = []
        def capturing_distance(point, trees):
            captured_points.append(point)
            return 0.05
        with patch("range_scanner.validation.evaluation._build_bvh_trees",
                   return_value=[MagicMock()]):
            with patch("range_scanner.validation.evaluation._nearest_distance",
                       side_effect=capturing_distance):
                evaluate_hits_against_targets(
                    ctx, [hit], targets,
                    use_noisy_measurements=True,
                )
        assert len(captured_points) == 1
        assert captured_points[0] == (5.0, 5.0, 5.0)

    def test_evaluation_mode_field(self):
        result = self._run([make_hit(0.0, 0.0, 0.0)])
        assert result["evaluation_mode"] == "pre_fusion_combined_accuracy"

    def test_empty_hits_zero_metrics(self):
        result = self._run([])
        assert result["hit_counts"]["total"] == 0
        assert result["metrics"]["rmse"] is None
