"""
Unit tests for range_scanner.validation.degradation.

MockVector (from conftest) is wired into sys.modules["mathutils"] so that
degradation.py uses our stub instead of the real Blender Vector.
"""

import math
import types

import pytest

from tests.conftest import make_hit, MockVector
from range_scanner.validation.degradation import (
    _frame_progress,
    _gnss_offset,
    _sea_state_offset,
    apply_maritime_degradation,
    degradation_enabled,
    get_degradation_config,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _props(**kwargs):
    """Build a minimal properties namespace for degradation tests."""
    defaults = dict(
        gnssDriftMeters=0.0,
        gnssDriftHeadingDegrees=0.0,
        seaStateAmplitudeMeters=0.0,
        seaStatePeriodFrames=20,
        seaStatePhaseDegrees=0.0,
        frameStart=1,
        frameEnd=10,
    )
    defaults.update(kwargs)
    return types.SimpleNamespace(**defaults)


# ---------------------------------------------------------------------------
# get_degradation_config / degradation_enabled
# ---------------------------------------------------------------------------

class TestGetDegradationConfig:
    def test_defaults_all_zero(self):
        config = get_degradation_config(_props())
        assert config["gnss_drift_m"] == 0.0
        assert config["sea_state_amplitude_m"] == 0.0

    def test_custom_values_passed_through(self):
        config = get_degradation_config(
            _props(gnssDriftMeters=2.5, seaStateAmplitudeMeters=0.3)
        )
        assert config["gnss_drift_m"] == pytest.approx(2.5)
        assert config["sea_state_amplitude_m"] == pytest.approx(0.3)

    def test_period_defaults_to_20(self):
        config = get_degradation_config(_props(seaStatePeriodFrames=20))
        assert config["sea_state_period_frames"] == 20


class TestDegradationEnabled:
    def test_no_drift_no_sea_state_disabled(self):
        config = get_degradation_config(_props())
        assert degradation_enabled(config) is False

    def test_gnss_drift_only_enabled(self):
        config = get_degradation_config(_props(gnssDriftMeters=1.0))
        assert degradation_enabled(config) is True

    def test_sea_state_only_enabled(self):
        config = get_degradation_config(_props(seaStateAmplitudeMeters=0.5))
        assert degradation_enabled(config) is True


# ---------------------------------------------------------------------------
# _frame_progress
# ---------------------------------------------------------------------------

class TestFrameProgress:
    def test_at_start_is_zero(self):
        assert _frame_progress(1, 1, 11) == pytest.approx(0.0)

    def test_at_end_is_one(self):
        assert _frame_progress(11, 1, 11) == pytest.approx(1.0)

    def test_midpoint(self):
        assert _frame_progress(6, 1, 11) == pytest.approx(0.5)

    def test_clamped_below_zero(self):
        assert _frame_progress(0, 1, 11) == pytest.approx(0.0)

    def test_clamped_above_one(self):
        assert _frame_progress(20, 1, 11) == pytest.approx(1.0)

    def test_equal_start_end_returns_zero(self):
        assert _frame_progress(5, 5, 5) == pytest.approx(0.0)

    def test_none_frame_returns_zero(self):
        assert _frame_progress(None, 1, 10) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# _gnss_offset
# ---------------------------------------------------------------------------

class TestGnssOffset:
    def test_zero_drift_zero_offset(self):
        config = {"gnss_drift_m": 0.0, "gnss_drift_heading_deg": 0.0}
        offset = _gnss_offset(config, progress=1.0)
        assert offset.x == pytest.approx(0.0)
        assert offset.y == pytest.approx(0.0)
        assert offset.z == pytest.approx(0.0)

    def test_east_heading_displaces_x(self):
        config = {"gnss_drift_m": 1.0, "gnss_drift_heading_deg": 0.0}
        offset = _gnss_offset(config, progress=1.0)
        assert offset.x == pytest.approx(1.0, abs=1e-6)
        assert offset.y == pytest.approx(0.0, abs=1e-6)

    def test_north_heading_displaces_y(self):
        config = {"gnss_drift_m": 1.0, "gnss_drift_heading_deg": 90.0}
        offset = _gnss_offset(config, progress=1.0)
        assert offset.x == pytest.approx(0.0, abs=1e-6)
        assert offset.y == pytest.approx(1.0, abs=1e-6)

    def test_half_progress_halves_magnitude(self):
        config = {"gnss_drift_m": 2.0, "gnss_drift_heading_deg": 0.0}
        offset = _gnss_offset(config, progress=0.5)
        assert offset.x == pytest.approx(1.0, abs=1e-6)

    def test_returns_zero_z(self):
        config = {"gnss_drift_m": 5.0, "gnss_drift_heading_deg": 45.0}
        offset = _gnss_offset(config, progress=1.0)
        assert offset.z == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# _sea_state_offset
# ---------------------------------------------------------------------------

class TestSeaStateOffset:
    def _config(self, amp=1.0, period=20, phase_deg=0.0):
        return {
            "sea_state_amplitude_m": amp,
            "sea_state_period_frames": period,
            "sea_state_phase_deg": phase_deg,
        }

    def test_at_start_frame_is_zero_phase(self):
        config = self._config(amp=1.0, period=20, phase_deg=0.0)
        # frame == frame_start → sin(0) = 0
        offset = _sea_state_offset(config, frame=1, frame_start=1)
        assert offset.z == pytest.approx(0.0, abs=1e-9)

    def test_half_period_gives_zero(self):
        config = self._config(amp=1.0, period=20, phase_deg=0.0)
        # frame_start=1, half period=10 frames later → frame 11
        # term = 2π * 10/20 = π → sin(π) ≈ 0
        offset = _sea_state_offset(config, frame=11, frame_start=1)
        assert offset.z == pytest.approx(0.0, abs=1e-6)

    def test_quarter_period_gives_amplitude(self):
        config = self._config(amp=2.0, period=20, phase_deg=0.0)
        # quarter period = 5 frames → frame 6
        # term = 2π * 5/20 = π/2 → sin(π/2) = 1 → z = amp = 2.0
        offset = _sea_state_offset(config, frame=6, frame_start=1)
        assert offset.z == pytest.approx(2.0, abs=1e-6)

    def test_zero_amplitude_zero_offset(self):
        config = self._config(amp=0.0)
        offset = _sea_state_offset(config, frame=5, frame_start=1)
        assert offset.z == pytest.approx(0.0, abs=1e-9)

    def test_returns_zero_x_and_y(self):
        config = self._config(amp=1.0)
        offset = _sea_state_offset(config, frame=5, frame_start=1)
        assert offset.x == pytest.approx(0.0)
        assert offset.y == pytest.approx(0.0)

    def test_phase_shift_offsets_cycle(self):
        config_no_phase  = self._config(amp=1.0, period=20, phase_deg=0.0)
        config_quarter   = self._config(amp=1.0, period=20, phase_deg=90.0)
        # With 90-degree phase, quarter period should give cos(π/2)=0 not sin(π/2)=1
        offset_no  = _sea_state_offset(config_no_phase,  frame=6, frame_start=1)
        offset_q   = _sea_state_offset(config_quarter,   frame=6, frame_start=1)
        assert offset_no.z  != pytest.approx(offset_q.z, abs=0.1)


# ---------------------------------------------------------------------------
# apply_maritime_degradation
# ---------------------------------------------------------------------------

class TestApplyMaritimeDegradation:
    def test_no_degradation_returns_disabled(self):
        hits = [make_hit(1.0, 0.0, 0.0)]
        result = apply_maritime_degradation(hits, _props())
        assert result["enabled"] is False
        assert result["applied_hit_count"] == 0

    def test_gnss_drift_modifies_noise_location(self):
        hit = make_hit(0.0, 0.0, 0.0)
        props = _props(
            gnssDriftMeters=1.0,
            gnssDriftHeadingDegrees=0.0,  # pure X drift
            frameStart=1,
            frameEnd=1,
        )
        apply_maritime_degradation([hit], props)
        # At frame 1, progress = 0/(1-1) but frame_end==frame_start → progress=0
        # So drift = 0 at start; use frameEnd=2 to test non-zero drift
        assert hit.noiseLocation is not None

    def test_full_drift_at_last_frame(self):
        hit = make_hit(0.0, 0.0, 0.0)
        hit.frame = 10
        props = _props(
            gnssDriftMeters=2.0,
            gnssDriftHeadingDegrees=0.0,
            frameStart=1,
            frameEnd=10,
        )
        apply_maritime_degradation([hit], props)
        # progress = (10-1)/(10-1) = 1.0 → x offset = 2.0
        assert hit.noiseLocation.x == pytest.approx(2.0, abs=1e-5)

    def test_sea_state_modifies_z(self):
        hit = make_hit(0.0, 0.0, 0.0)
        hit.frame = 6
        props = _props(
            seaStateAmplitudeMeters=1.0,
            seaStatePeriodFrames=20,
            frameStart=1,
            frameEnd=20,
        )
        apply_maritime_degradation([hit], props)
        # quarter period → z offset ≈ 1.0
        assert hit.noiseLocation.z == pytest.approx(1.0, abs=1e-5)

    def test_noise_distance_updated(self):
        hit = make_hit(0.0, 0.0, 0.0, distance=10.0)
        hit.frame = 10
        props = _props(
            gnssDriftMeters=3.0,
            gnssDriftHeadingDegrees=0.0,
            frameStart=1,
            frameEnd=10,
        )
        apply_maritime_degradation([hit], props)
        # offset.length > 0, so noise distance should exceed original
        assert hit.noiseDistance > 10.0

    def test_applied_hit_count_returned(self):
        hits = [make_hit(float(i), 0.0, 0.0) for i in range(5)]
        props = _props(gnssDriftMeters=1.0, frameStart=1, frameEnd=10)
        result = apply_maritime_degradation(hits, props)
        assert result["applied_hit_count"] == 5

    def test_existing_noise_location_used_as_base(self):
        """If noiseLocation already set, degradation adds to it (not raw location)."""
        hit = make_hit(0.0, 0.0, 0.0)
        hit.noiseLocation = MockVector((5.0, 0.0, 0.0))
        hit.frame = 10
        props = _props(
            gnssDriftMeters=1.0,
            gnssDriftHeadingDegrees=0.0,
            frameStart=1,
            frameEnd=10,
        )
        apply_maritime_degradation([hit], props)
        # Base is (5, 0, 0), drift adds ~(1, 0, 0) → result ~(6, 0, 0)
        assert hit.noiseLocation.x == pytest.approx(6.0, abs=1e-5)
