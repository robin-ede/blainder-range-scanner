"""
Unit tests for range_scanner.validation.fusion.

All functions tested here are pure numpy — no Blender context needed.
"""

import pytest
import numpy as np

from tests.conftest import make_hit
from range_scanner.validation.fusion import (
    _robust_weighted_point,
    _grid_reconstruct_points,
    reconstruct_medium_height_grid_from_points,
    voxel_fuse_hits,
)


# ---------------------------------------------------------------------------
# _robust_weighted_point
# ---------------------------------------------------------------------------

class TestRobustWeightedPoint:
    def test_single_point_returns_that_point(self):
        bucket = {"points": [(1.0, 2.0, 3.0)], "weights": [1.0]}
        result = _robust_weighted_point(bucket)
        assert result == pytest.approx((1.0, 2.0, 3.0))

    def test_two_identical_points(self):
        bucket = {"points": [(5.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
                  "weights": [1.0, 1.0]}
        result = _robust_weighted_point(bucket)
        assert result == pytest.approx((5.0, 0.0, 0.0))

    def test_outlier_trimmed_away(self):
        # Eight tight cluster points + one far outlier
        cluster = [(0.0, 0.0, 0.0)] * 8
        outlier = [(500.0, 0.0, 0.0)]
        bucket = {"points": cluster + outlier, "weights": [1.0] * 9}
        result = _robust_weighted_point(bucket)
        # Representative should be near the cluster, not the outlier
        assert result[0] == pytest.approx(0.0, abs=0.5)

    def test_result_is_one_of_the_input_points(self):
        # The function picks the input point closest to the weighted centre
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        bucket = {"points": points, "weights": [1.0, 1.0, 1.0]}
        result = _robust_weighted_point(bucket)
        # Should return the middle point (centre is 1.0)
        assert result == pytest.approx((1.0, 0.0, 0.0))

    def test_zero_weights_fallback_to_uniform(self):
        bucket = {"points": [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)],
                  "weights": [0.0, 0.0]}
        # Should not raise; falls back to equal weighting
        result = _robust_weighted_point(bucket)
        assert result is not None

    def test_returns_tuple_of_floats(self):
        bucket = {"points": [(1, 2, 3)], "weights": [1.0]}
        result = _robust_weighted_point(bucket)
        assert isinstance(result, tuple)
        assert all(isinstance(v, float) for v in result)


# ---------------------------------------------------------------------------
# _grid_reconstruct_points
# ---------------------------------------------------------------------------

class TestGridReconstructPoints:
    def test_empty_input(self):
        result = _grid_reconstruct_points([], 1.0)
        assert result == []

    def test_invalid_grid_size_raises(self):
        with pytest.raises(ValueError):
            _grid_reconstruct_points([(0.0, 0.0, 0.0)], 0.0)

    def test_negative_grid_size_raises(self):
        with pytest.raises(ValueError):
            _grid_reconstruct_points([(0.0, 0.0, 0.0)], -1.0)

    def test_single_point_single_cell(self):
        result = _grid_reconstruct_points([(0.5, 0.5, 1.0)], 1.0)
        assert len(result) == 1
        assert result[0] == pytest.approx((0.5, 0.5, 1.0))

    def test_points_in_same_cell_collapsed(self):
        # Both in cell (0, 0)
        points = [(0.1, 0.2, 1.0), (0.3, 0.4, 2.0)]
        result = _grid_reconstruct_points(points, 1.0)
        assert len(result) == 1

    def test_points_in_different_cells_kept(self):
        points = [(0.1, 0.1, 1.0), (5.0, 5.0, 2.0)]
        result = _grid_reconstruct_points(points, 1.0)
        assert len(result) == 2

    def test_representative_is_median(self):
        # Three points in same cell; z-values 1, 2, 3 → median z = 2
        points = [(0.1, 0.1, 1.0), (0.2, 0.2, 2.0), (0.3, 0.3, 3.0)]
        result = _grid_reconstruct_points(points, 1.0)
        assert len(result) == 1
        assert result[0][2] == pytest.approx(2.0)

    def test_output_tuples_of_floats(self):
        result = _grid_reconstruct_points([(0.0, 0.0, 0.0)], 1.0)
        assert isinstance(result[0], tuple)
        assert all(isinstance(v, float) for v in result[0])


# ---------------------------------------------------------------------------
# reconstruct_medium_height_grid_from_points
# ---------------------------------------------------------------------------

class TestReconstructMediumHeightGridFromPoints:
    def test_above_and_below_water_separated(self):
        above = [(1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]
        below = [(1.0, 1.0, -1.0), (2.0, 2.0, -2.0)]
        result = reconstruct_medium_height_grid_from_points(
            above + below, water_surface_height=0.0
        )
        assert result["regions"]["above_water"]["input_point_count"] == 2
        assert result["regions"]["below_water"]["input_point_count"] == 2

    def test_no_water_height_skips_all_points(self):
        points = [(float(i), 0.0, float(i)) for i in range(5)]
        result = reconstruct_medium_height_grid_from_points(
            points, water_surface_height=None
        )
        assert result["input_point_count"] == 0
        assert result["reconstructed_point_count"] == 0

    def test_compression_ratio_below_one_for_dense_input(self):
        # Many points in same XY cells should be compressed
        above = [(0.0 + i * 0.01, 0.0 + j * 0.01, 1.0)
                 for i in range(5) for j in range(5)]
        result = reconstruct_medium_height_grid_from_points(
            above,
            above_water_grid_size=1.0,
            water_surface_height=0.0,
        )
        assert result["compression_ratio"] < 1.0

    def test_reconstruction_mode_label(self):
        result = reconstruct_medium_height_grid_from_points(
            [(0.0, 0.0, 1.0)], water_surface_height=0.0
        )
        assert result["reconstruction_mode"] == "medium_height_grid_map"

    def test_empty_input(self):
        result = reconstruct_medium_height_grid_from_points(
            [], water_surface_height=0.0
        )
        assert result["input_point_count"] == 0
        assert result["compression_ratio"] is None


# ---------------------------------------------------------------------------
# voxel_fuse_hits
# ---------------------------------------------------------------------------

class TestVoxelFuseHits:
    def test_single_hit_preserved(self):
        hits = [make_hit(1.0, 2.0, 3.0)]
        result = voxel_fuse_hits(hits)
        assert result["fused_point_count"] == 1
        assert result["input_point_count"] == 1

    def test_hits_in_same_voxel_fused(self):
        # Two hits within 0.05 m of each other → same voxel at default size 0.05
        hits = [
            make_hit(0.01, 0.0, 0.0),
            make_hit(0.02, 0.0, 0.0),
        ]
        result = voxel_fuse_hits(hits, voxel_size=0.05)
        assert result["fused_point_count"] == 1

    def test_hits_in_different_voxels_kept_separate(self):
        hits = [
            make_hit(0.0, 0.0, 0.0),
            make_hit(10.0, 0.0, 0.0),
        ]
        result = voxel_fuse_hits(hits, voxel_size=0.05)
        assert result["fused_point_count"] == 2

    def test_multi_sensor_voxel_counted(self):
        hits = [
            make_hit(0.01, 0.0, 0.0, sensor_id="lidar"),
            make_hit(0.02, 0.0, 0.0, sensor_id="sonar"),
        ]
        result = voxel_fuse_hits(hits, voxel_size=0.05)
        assert result["multi_sensor_voxel_count"] == 1

    def test_multi_frame_voxel_counted(self):
        hits = [
            make_hit(0.01, 0.0, 0.0, frame=1),
            make_hit(0.02, 0.0, 0.0, frame=2),
        ]
        result = voxel_fuse_hits(hits, voxel_size=0.05)
        assert result["multi_frame_voxel_count"] == 1

    def test_empty_hits(self):
        result = voxel_fuse_hits([])
        assert result["fused_point_count"] == 0
        assert result["compression_ratio"] is None

    def test_invalid_voxel_size_raises(self):
        with pytest.raises(ValueError):
            voxel_fuse_hits([make_hit(0.0, 0.0, 0.0)], voxel_size=0.0)

    def test_uses_noisy_measurements(self):
        # Clean location at (0,0,0), noisy at (10,0,0) → different voxels
        hits = [
            make_hit(0.0, 0.0, 0.0, noise_x=10.0, noise_y=0.0, noise_z=0.0),
            make_hit(0.01, 0.0, 0.0, noise_x=10.01, noise_y=0.0, noise_z=0.0),
        ]
        clean_result = voxel_fuse_hits(hits, voxel_size=1.0, use_noisy_measurements=False)
        noisy_result = voxel_fuse_hits(hits, voxel_size=1.0, use_noisy_measurements=True)
        # In clean mode both at ~(0,0,0) → 1 voxel; noisy at ~(10,0,0) → also 1 voxel
        assert clean_result["fused_point_count"] == 1
        assert noisy_result["fused_point_count"] == 1

    def test_compression_ratio_populated(self):
        # 4 hits in 2 voxels → ratio = 0.5
        hits = [
            make_hit(0.01, 0.0, 0.0), make_hit(0.02, 0.0, 0.0),
            make_hit(5.0, 0.0, 0.0),  make_hit(5.01, 0.0, 0.0),
        ]
        result = voxel_fuse_hits(hits, voxel_size=0.05)
        assert result["compression_ratio"] == pytest.approx(0.5)

    def test_fusion_mode_label(self):
        result = voxel_fuse_hits([make_hit(0.0, 0.0, 0.0)])
        assert result["fusion_mode"] == "weighted_trimmed_voxel"

    def test_water_medium_separation_counted(self):
        above_hits = [make_hit(0.0, 0.0, 1.0) for _ in range(3)]
        below_hits = [make_hit(0.0, 0.0, -1.0) for _ in range(2)]
        result = voxel_fuse_hits(
            above_hits + below_hits,
            water_surface_height=0.0,
        )
        assert result["point_counts_by_medium"]["above_water"] == 3
        assert result["point_counts_by_medium"]["below_water"] == 2
