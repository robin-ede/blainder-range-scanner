"""
Unit tests for range_scanner.error_distribution.

All functions are pure numpy — no Blender context needed.
"""

import pytest

from range_scanner import error_distribution as ed


# ---------------------------------------------------------------------------
# setSeed / applyNoise — reproducibility
# ---------------------------------------------------------------------------


class TestSetSeed:
    def test_same_seed_produces_same_sequence(self):
        ed.setSeed(42)
        v1 = ed.applyNoise(0.0, 1.0)
        ed.setSeed(42)
        v2 = ed.applyNoise(0.0, 1.0)
        assert v1 == v2

    def test_different_seeds_produce_different_values(self):
        ed.setSeed(1)
        v1 = ed.applyNoise(0.0, 1.0)
        ed.setSeed(2)
        v2 = ed.applyNoise(0.0, 1.0)
        assert v1 != v2

    def test_reseed_mid_sequence_resets_reproducibly(self):
        ed.setSeed(99)
        ed.applyNoise(0.0, 1.0)  # advance the sequence
        ed.applyNoise(0.0, 1.0)
        ed.setSeed(99)  # reset
        v_reset = ed.applyNoise(0.0, 1.0)

        ed.setSeed(99)
        v_fresh = ed.applyNoise(0.0, 1.0)

        assert v_reset == v_fresh


# ---------------------------------------------------------------------------
# applyNoise — output characteristics
# ---------------------------------------------------------------------------


class TestApplyNoise:
    def test_returns_scalar(self):
        ed.setSeed(0)
        result = ed.applyNoise(0.0, 1.0)
        # Must be a scalar (float-like), not an array
        assert hasattr(result, "__float__")

    def test_zero_sigma_returns_mu(self):
        ed.setSeed(0)
        mu = 5.0
        result = ed.applyNoise(mu, 0.0)
        assert float(result) == pytest.approx(mu)

    def test_mean_converges_to_mu(self):
        # Over many draws the sample mean should be close to mu
        ed.setSeed(123)
        mu, sigma = 3.0, 1.0
        n = 10_000
        total = sum(float(ed.applyNoise(mu, sigma)) for _ in range(n))
        sample_mean = total / n
        assert sample_mean == pytest.approx(mu, abs=0.05)

    def test_negative_mu_works(self):
        ed.setSeed(7)
        result = ed.applyNoise(-10.0, 0.0)
        assert float(result) == pytest.approx(-10.0)


# ---------------------------------------------------------------------------
# getSeededStatePreview
# ---------------------------------------------------------------------------


class TestGetSeededStatePreview:
    def test_returns_non_empty_string(self):
        ed.setSeed(0)
        preview = ed.getSeededStatePreview()
        assert isinstance(preview, str)
        assert len(preview) > 0

    def test_changes_after_seeding(self):
        ed.setSeed(1)
        p1 = ed.getSeededStatePreview()
        ed.setSeed(2)
        p2 = ed.getSeededStatePreview()
        assert p1 != p2
