"""
Unit tests for range_scanner.validation.alignment.

Pure numpy — no Blender context needed.
"""

import numpy as np
import pytest

from tests.conftest import make_hit
from range_scanner.validation.alignment import (
    _estimate_translation_to_point_cloud,
    _sample_points,
    align_hits_blind,
    align_hits_to_reference,
)


# ---------------------------------------------------------------------------
# _sample_points
# ---------------------------------------------------------------------------

class TestSamplePoints:
    def test_below_max_returns_unchanged(self):
        pts = list(range(10))
        result = _sample_points(pts, 20)
        assert result == pts

    def test_at_max_returns_unchanged(self):
        pts = list(range(10))
        assert _sample_points(pts, 10) == pts

    def test_above_max_reduces_length(self):
        pts = list(range(100))
        result = _sample_points(pts, 10)
        assert len(result) <= 10

    def test_first_element_preserved(self):
        pts = list(range(50))
        result = _sample_points(pts, 5)
        assert result[0] == 0

    def test_empty_input(self):
        assert _sample_points([], 10) == []


# ---------------------------------------------------------------------------
# _estimate_translation_to_point_cloud
# ---------------------------------------------------------------------------

class TestEstimateTranslation:
    def test_empty_source_returns_zero(self):
        result = _estimate_translation_to_point_cloud(
            np.empty((0, 3)), np.array([[1.0, 0.0, 0.0]])
        )
        assert np.allclose(result, [0.0, 0.0, 0.0])

    def test_empty_target_returns_zero(self):
        result = _estimate_translation_to_point_cloud(
            np.array([[0.0, 0.0, 0.0]]), np.empty((0, 3))
        )
        assert np.allclose(result, [0.0, 0.0, 0.0])

    def test_perfect_offset_recovered(self):
        # Three-point cloud with unique nearest-neighbour correspondences;
        # the constant +3 X shift is unambiguous so ICP converges exactly.
        source = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        target = source + np.array([3.0, 0.0, 0.0])
        result = _estimate_translation_to_point_cloud(source, target)
        assert np.allclose(result, [3.0, 0.0, 0.0], atol=0.01)

    def test_zero_offset_source_equals_target(self):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        result = _estimate_translation_to_point_cloud(pts, pts.copy())
        assert np.allclose(result, [0.0, 0.0, 0.0], atol=1e-4)

    def test_3d_offset_recovered(self):
        # Single point: unambiguous correspondence, converges in one step.
        offset = np.array([1.0, 2.0, 3.0])
        source = np.array([[0.0, 0.0, 0.0]])
        target = source + offset
        result = _estimate_translation_to_point_cloud(source, target)
        assert np.allclose(result, offset, atol=0.01)

    def test_result_is_numpy_array(self):
        source = np.array([[0.0, 0.0, 0.0]])
        target = np.array([[1.0, 0.0, 0.0]])
        result = _estimate_translation_to_point_cloud(source, target)
        assert isinstance(result, np.ndarray)
        assert result.shape == (3,)


# ---------------------------------------------------------------------------
# align_hits_to_reference
# ---------------------------------------------------------------------------

class TestAlignHitsToReference:
    def _make_drifted_hits(self, drift_x, frame=1):
        """Hit where clean location != noisy (drifted) location."""
        return make_hit(
            x=1.0, y=0.0, z=0.0,       # clean reference location
            noise_x=1.0 - drift_x,      # drifted noisy location
            noise_y=0.0, noise_z=0.0,
            frame=frame,
        )

    def test_no_drift_corrected_equals_location(self):
        hit = make_hit(1.0, 2.0, 3.0, noise_x=1.0, noise_y=2.0, noise_z=3.0)
        result = align_hits_to_reference([hit], use_noisy_measurements=True)
        corrected = result["corrected_points"][0]
        assert corrected == pytest.approx((1.0, 2.0, 3.0), abs=1e-6)

    def test_known_drift_corrected(self):
        """Drift of +0.5 in X should be corrected back to clean location."""
        hit = self._make_drifted_hits(drift_x=0.5, frame=1)
        result = align_hits_to_reference([hit], use_noisy_measurements=True)
        corrected = result["corrected_points"][0]
        # noisy = 0.5, correction = +0.5, result ≈ 1.0
        assert corrected[0] == pytest.approx(1.0, abs=1e-6)

    def test_translation_summary_populated(self):
        hit = self._make_drifted_hits(drift_x=1.0, frame=1)
        result = align_hits_to_reference([hit], use_noisy_measurements=True)
        assert result["translation_summary"]["frame_count"] == 1
        assert result["translation_summary"]["max_translation_m"] == pytest.approx(
            1.0, abs=1e-6
        )

    def test_multiple_frames_independent_corrections(self):
        # Frame 1 drifted by 0.5, frame 2 drifted by 1.0
        hit1 = self._make_drifted_hits(drift_x=0.5, frame=1)
        hit2 = self._make_drifted_hits(drift_x=1.0, frame=2)
        result = align_hits_to_reference(
            [hit1, hit2], use_noisy_measurements=True
        )
        assert result["translation_summary"]["frame_count"] == 2
        corrected1, corrected2 = result["corrected_points"]
        assert corrected1[0] == pytest.approx(1.0, abs=1e-6)
        assert corrected2[0] == pytest.approx(1.0, abs=1e-6)

    def test_alignment_mode_label(self):
        result = align_hits_to_reference([make_hit(0.0, 0.0, 0.0)])
        assert "reference" in result["alignment_mode"]

    def test_empty_hits(self):
        result = align_hits_to_reference([])
        assert result["corrected_points"] == []
        assert result["translation_summary"]["frame_count"] == 0

    def test_frame_translations_in_output(self):
        hit = make_hit(1.0, 0.0, 0.0, noise_x=0.5, noise_y=0.0, noise_z=0.0,
                       frame=3)
        result = align_hits_to_reference([hit], use_noisy_measurements=True)
        assert "3" in result["frame_translations"]


# ---------------------------------------------------------------------------
# align_hits_blind
# ---------------------------------------------------------------------------

class TestAlignHitsBlind:
    def test_single_frame_no_correction(self):
        """First frame is always the anchor — no translation applied."""
        hits = [make_hit(1.0, 0.0, 0.0, frame=0) for _ in range(3)]
        result = align_hits_blind(hits)
        # All corrected points should equal original locations
        for pt in result["corrected_points"]:
            assert pt[0] == pytest.approx(1.0)

    def test_two_frames_second_corrected_toward_first(self):
        """Frame 1 shifted by +5 in X; blind ICP applies a correction.

        ICP aligns frame 1 onto frame 0 using nearest-neighbour correspondences.
        With two evenly spaced points, the estimated correction is the median of
        residuals.  We verify that the translation is non-zero and reduces the
        centroid error.
        """
        frame0_hits = [make_hit(0.0, 0.0, 0.0, frame=0),
                       make_hit(1.0, 0.0, 0.0, frame=0)]
        frame1_hits = [make_hit(5.0, 0.0, 0.0, frame=1),
                       make_hit(6.0, 0.0, 0.0, frame=1)]
        result = align_hits_blind(frame0_hits + frame1_hits)
        # The accumulated translation for frame 1 should be negative X (toward frame 0)
        t1 = result["frame_translations"]["1"]
        assert t1[0] < 0, "Frame 1 correction should be negative X"
        # Corrected centroid of frame 1 should be closer to frame 0 centroid (0.5)
        f1_corrected = result["corrected_points"][2:]  # last two entries
        centroid_x = (f1_corrected[0][0] + f1_corrected[1][0]) / 2.0
        assert centroid_x == pytest.approx(0.5, abs=0.1)

    def test_plausibility_cap_prevents_large_translation(self):
        """A 100 m jump between frames exceeds the cap → zero correction."""
        frame0 = [make_hit(0.0, 0.0, 0.0, frame=0)]
        frame1 = [make_hit(200.0, 0.0, 0.0, frame=1)]
        result = align_hits_blind(
            frame0 + frame1,
            max_plausible_translation_m=5.0,
        )
        # Frame 1 point should be uncorrected (remains at 200)
        pt1 = result["corrected_points"][1]
        assert pt1[0] == pytest.approx(200.0)

    def test_frame_count_in_summary(self):
        hits = [make_hit(0.0, 0.0, 0.0, frame=i) for i in range(4)]
        result = align_hits_blind(hits)
        assert result["translation_summary"]["frame_count"] == 4

    def test_alignment_mode_label(self):
        result = align_hits_blind([make_hit(0.0, 0.0, 0.0, frame=0)])
        assert "blind" in result["alignment_mode"]

    def test_empty_hits(self):
        result = align_hits_blind([])
        assert result["corrected_points"] == []

    def test_accumulated_translations_in_output(self):
        hits = [make_hit(float(i), 0.0, 0.0, frame=i) for i in range(3)]
        result = align_hits_blind(hits)
        # Keys are string frame numbers
        assert "0" in result["frame_translations"]
        assert "1" in result["frame_translations"]
