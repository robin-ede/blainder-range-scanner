"""
Unit tests for range_scanner.validation.fusion — reconstruction functions.

Covers two functions not tested in test_fusion.py:
  - reconstruct_medium_separated_map
  - reconstruct_medium_height_grid_map
"""

import pytest

from tests.conftest import make_hit
from range_scanner.validation.fusion import (
    reconstruct_medium_separated_map,
    reconstruct_medium_height_grid_map,
)

# Water surface at z=0: positive z → above water, negative z → below water
_WATER_HEIGHT = 0.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _above_hits(n=4, z=1.0):
    """n hits clearly above the water surface."""
    return [make_hit(float(i), 0.0, z, sensor_id="lidar") for i in range(n)]


def _below_hits(n=4, z=-1.0):
    """n hits clearly below the water surface."""
    return [make_hit(float(i), 0.0, z, sensor_id="sonar") for i in range(n)]


# ---------------------------------------------------------------------------
# reconstruct_medium_separated_map
# ---------------------------------------------------------------------------


class TestReconstructMediumSeparatedMap:
    def test_output_dict_keys(self):
        hits = _above_hits(2) + _below_hits(2)
        result = reconstruct_medium_separated_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        assert result["reconstruction_mode"] == "medium_separated_stitched_map"
        assert "input_point_count" in result
        assert "reconstructed_point_count" in result
        assert "compression_ratio" in result
        assert "stitched_points" in result
        assert "regions" in result
        assert "above_water" in result["regions"]
        assert "below_water" in result["regions"]

    def test_splits_hits_by_medium(self):
        above = _above_hits(3)
        below = _below_hits(2)
        result = reconstruct_medium_separated_map(
            above + below, water_surface_height=_WATER_HEIGHT
        )
        assert result["regions"]["above_water"]["input_point_count"] == 3
        assert result["regions"]["below_water"]["input_point_count"] == 2

    def test_total_input_count_matches(self):
        hits = _above_hits(4) + _below_hits(3)
        result = reconstruct_medium_separated_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        assert result["input_point_count"] == 7

    def test_stitched_points_is_union_of_regions(self):
        hits = _above_hits(4) + _below_hits(4)
        result = reconstruct_medium_separated_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        expected_count = (
            result["regions"]["above_water"]["fused_point_count"]
            + result["regions"]["below_water"]["fused_point_count"]
        )
        assert len(result["stitched_points"]) == expected_count
        assert result["reconstructed_point_count"] == expected_count

    def test_compression_ratio_is_none_for_empty_input(self):
        result = reconstruct_medium_separated_map(
            [], water_surface_height=_WATER_HEIGHT
        )
        assert result["compression_ratio"] is None
        assert result["input_point_count"] == 0

    def test_different_voxel_sizes_per_region(self):
        # Place all points in the same two locations per region so voxel size
        # determines whether they collapse or not.
        # Above: 4 points in a single voxel of size 2.0 → collapses to 1
        above = [make_hit(0.1 * i, 0.0, 1.0, sensor_id="lidar") for i in range(4)]
        # Below: same pattern
        below = [make_hit(0.1 * i, 0.0, -1.0, sensor_id="sonar") for i in range(4)]
        result_large = reconstruct_medium_separated_map(
            above + below,
            above_water_voxel_size=5.0,
            below_water_voxel_size=5.0,
            water_surface_height=_WATER_HEIGHT,
        )
        result_small = reconstruct_medium_separated_map(
            above + below,
            above_water_voxel_size=0.01,
            below_water_voxel_size=0.01,
            water_surface_height=_WATER_HEIGHT,
        )
        # Large voxel collapses more points → fewer reconstructed
        assert (
            result_large["reconstructed_point_count"]
            <= result_small["reconstructed_point_count"]
        )

    def test_use_noisy_measurements_flag(self):
        # Hits with noiseLocation above water but clean location below water:
        # use_noisy_measurements=True should classify them as above water.
        hits = [
            make_hit(
                0.0, 0.0, -1.0, noise_x=0.0, noise_y=0.0, noise_z=1.0, sensor_id="lidar"
            )
            for _ in range(3)
        ]
        result_clean = reconstruct_medium_separated_map(
            hits, water_surface_height=_WATER_HEIGHT, use_noisy_measurements=False
        )
        result_noisy = reconstruct_medium_separated_map(
            hits, water_surface_height=_WATER_HEIGHT, use_noisy_measurements=True
        )
        assert result_clean["regions"]["below_water"]["input_point_count"] == 3
        assert result_noisy["regions"]["above_water"]["input_point_count"] == 3

    def test_no_water_height_produces_empty_regions(self):
        # Without a water surface height all hits are 'unknown' and neither
        # region is populated.
        hits = _above_hits(3) + _below_hits(3)
        result = reconstruct_medium_separated_map(hits, water_surface_height=None)
        assert result["regions"]["above_water"]["input_point_count"] == 0
        assert result["regions"]["below_water"]["input_point_count"] == 0
        assert result["input_point_count"] == 0


# ---------------------------------------------------------------------------
# reconstruct_medium_height_grid_map
# ---------------------------------------------------------------------------


class TestReconstructMediumHeightGridMap:
    def test_output_dict_keys(self):
        hits = _above_hits(2) + _below_hits(2)
        result = reconstruct_medium_height_grid_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        assert result["reconstruction_mode"] == "medium_height_grid_map"
        assert "input_point_count" in result
        assert "reconstructed_point_count" in result
        assert "compression_ratio" in result
        assert "stitched_points" in result
        assert "regions" in result
        assert "above_water" in result["regions"]
        assert "below_water" in result["regions"]

    def test_splits_hits_by_medium(self):
        above = _above_hits(3)
        below = _below_hits(2)
        result = reconstruct_medium_height_grid_map(
            above + below, water_surface_height=_WATER_HEIGHT
        )
        assert result["regions"]["above_water"]["input_point_count"] == 3
        assert result["regions"]["below_water"]["input_point_count"] == 2

    def test_total_input_count(self):
        hits = _above_hits(5) + _below_hits(3)
        result = reconstruct_medium_height_grid_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        assert result["input_point_count"] == 8

    def test_stitched_points_count_matches_reconstructed(self):
        hits = _above_hits(4) + _below_hits(4)
        result = reconstruct_medium_height_grid_map(
            hits, water_surface_height=_WATER_HEIGHT
        )
        assert len(result["stitched_points"]) == result["reconstructed_point_count"]

    def test_compression_ratio_none_for_empty(self):
        result = reconstruct_medium_height_grid_map(
            [], water_surface_height=_WATER_HEIGHT
        )
        assert result["compression_ratio"] is None
        assert result["reconstructed_point_count"] == 0

    def test_use_noisy_measurements_picks_noise_location(self):
        # Hits with clean location below water, noisy location above water.
        hits = [
            make_hit(
                0.0, 0.0, -1.0, noise_x=0.0, noise_y=0.0, noise_z=1.0, sensor_id="lidar"
            )
            for _ in range(3)
        ]
        result_clean = reconstruct_medium_height_grid_map(
            hits, water_surface_height=_WATER_HEIGHT, use_noisy_measurements=False
        )
        result_noisy = reconstruct_medium_height_grid_map(
            hits, water_surface_height=_WATER_HEIGHT, use_noisy_measurements=True
        )
        assert result_clean["regions"]["below_water"]["input_point_count"] == 3
        assert result_noisy["regions"]["above_water"]["input_point_count"] == 3

    def test_no_water_height_produces_empty_regions(self):
        hits = _above_hits(3) + _below_hits(3)
        result = reconstruct_medium_height_grid_map(hits, water_surface_height=None)
        assert result["regions"]["above_water"]["input_point_count"] == 0
        assert result["regions"]["below_water"]["input_point_count"] == 0
