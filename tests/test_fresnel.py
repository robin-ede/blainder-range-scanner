"""
Unit tests for range_scanner.fresnel.

All functions are pure numpy — no Blender context needed.

Key analytical values used:
  - For m=1.5 (glass in air): Brewster angle = arctan(1.5) ≈ 56.31°
  - Normal incidence reflectance: ((n-1)/(n+1))^2 = (0.5/2.5)^2 = 0.04
  - Energy conservation: R + T = 1 at normal incidence (lossless dielectric)
  - At Brewster angle: R_par = 0 (defining property)

Note: brewster() and critical() accept real m; amplitude/power functions
require complex m (even if imaginary part is zero) because they call m.imag.
"""

import math

import numpy as np
import pytest

from range_scanner.fresnel import (
    R_par,
    R_per,
    R_unpolarized,
    T_par,
    T_per,
    T_unpolarized,
    brewster,
    critical,
    r_par_amplitude,
    r_per_amplitude,
    t_par_amplitude,
    t_per_amplitude,
)

# Convenience: glass index used throughout
_N_GLASS = 1.5
_M_GLASS = complex(_N_GLASS)  # 1.5+0j  — required for m.imag access


# ---------------------------------------------------------------------------
# brewster
# ---------------------------------------------------------------------------


class TestBrewster:
    def test_glass_in_air(self):
        # arctan(1.5) ≈ 0.9828 rad ≈ 56.31°
        expected = math.atan(_N_GLASS)
        assert brewster(_N_GLASS) == pytest.approx(expected, rel=1e-6)

    def test_water_in_air(self):
        # n_water ≈ 1.333
        n_water = 1.333
        expected = math.atan(n_water)
        assert brewster(n_water) == pytest.approx(expected, rel=1e-6)

    def test_normal_medium_n1_returns_pi_over_4(self):
        # arctan(1) = pi/4
        assert brewster(1.0) == pytest.approx(math.pi / 4, rel=1e-6)


# ---------------------------------------------------------------------------
# critical
# ---------------------------------------------------------------------------


class TestCritical:
    def test_glass_to_air_critical_angle(self):
        # Light going from glass (n=1.5) to air (n=1): critical angle = arcsin(1/1.5)
        expected = math.asin(1.0 / _N_GLASS)
        assert critical(1.0 / _N_GLASS) == pytest.approx(expected, rel=1e-6)

    def test_equal_media_returns_pi_over_2(self):
        # n_i = n_t = 1 → critical angle = arcsin(1) = pi/2
        assert critical(1.0) == pytest.approx(math.pi / 2, rel=1e-6)


# ---------------------------------------------------------------------------
# Normal incidence: r amplitudes and reflectance
# ---------------------------------------------------------------------------


class TestNormalIncidence:
    """At theta_i = 0, R_par = R_per = ((n-1)/(n+1))^2 and R+T = 1."""

    def test_r_par_amplitude_at_normal(self):
        # r_par at normal incidence = (n-1)/(n+1) in magnitude for real n
        expected = (_N_GLASS - 1) / (_N_GLASS + 1)
        result = r_par_amplitude(_M_GLASS, 0.0)
        assert abs(result) == pytest.approx(expected, rel=1e-6)

    def test_r_per_amplitude_at_normal(self):
        # Magnitude same as r_par at normal incidence
        expected = (_N_GLASS - 1) / (_N_GLASS + 1)
        result = r_per_amplitude(_M_GLASS, 0.0)
        assert abs(result) == pytest.approx(expected, rel=1e-6)

    def test_R_par_equals_R_per_at_normal(self):
        assert R_par(_M_GLASS, 0.0) == pytest.approx(R_per(_M_GLASS, 0.0), rel=1e-6)

    def test_R_unpolarized_known_value_at_normal(self):
        # ((1.5-1)/(1.5+1))^2 = (0.5/2.5)^2 = 0.04
        assert R_unpolarized(_M_GLASS, 0.0) == pytest.approx(0.04, rel=1e-4)

    def test_energy_conservation_R_par(self):
        assert R_par(_M_GLASS, 0.0) + T_par(_M_GLASS, 0.0) == pytest.approx(
            1.0, abs=1e-6
        )

    def test_energy_conservation_R_per(self):
        assert R_per(_M_GLASS, 0.0) + T_per(_M_GLASS, 0.0) == pytest.approx(
            1.0, abs=1e-6
        )

    def test_energy_conservation_unpolarized(self):
        assert R_unpolarized(_M_GLASS, 0.0) + T_unpolarized(
            _M_GLASS, 0.0
        ) == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Brewster angle: R_par = 0
# ---------------------------------------------------------------------------


class TestBrewsterAngle:
    def test_R_par_zero_at_brewster_angle(self):
        # At Brewster's angle, parallel-polarized light has zero reflectance
        theta_b = brewster(_N_GLASS)
        assert R_par(_M_GLASS, theta_b) == pytest.approx(0.0, abs=1e-6)

    def test_R_per_nonzero_at_brewster_angle(self):
        # Perpendicular component is still reflected
        theta_b = brewster(_N_GLASS)
        assert R_per(_M_GLASS, theta_b) > 0.01


# ---------------------------------------------------------------------------
# R_unpolarized consistency
# ---------------------------------------------------------------------------


class TestRUnpolarizedConsistency:
    def test_is_average_of_R_par_and_R_per(self):
        theta = 0.5  # some angle in radians
        expected = (R_par(_M_GLASS, theta) + R_per(_M_GLASS, theta)) / 2.0
        assert R_unpolarized(_M_GLASS, theta) == pytest.approx(expected, rel=1e-6)

    def test_is_average_of_T_par_and_T_per(self):
        theta = 0.5
        expected = (T_par(_M_GLASS, theta) + T_per(_M_GLASS, theta)) / 2.0
        assert T_unpolarized(_M_GLASS, theta) == pytest.approx(expected, rel=1e-6)


# ---------------------------------------------------------------------------
# Complex index of refraction (absorbing medium)
# ---------------------------------------------------------------------------


class TestComplexRefractiveIndex:
    def test_R_unpolarized_nonzero_for_absorbing_medium(self):
        # A medium with a non-zero extinction coefficient (imaginary part)
        m_absorbing = 1.5 + 0.5j
        result = R_unpolarized(m_absorbing, 0.0)
        assert result > 0.0

    def test_amplitude_functions_accept_complex_m(self):
        m_absorbing = 1.5 + 0.5j
        # Should not raise
        rp = r_par_amplitude(m_absorbing, 0.3)
        rs = r_per_amplitude(m_absorbing, 0.3)
        tp = t_par_amplitude(m_absorbing, 0.3)
        ts = t_per_amplitude(m_absorbing, 0.3)
        assert rp is not None
        assert rs is not None
        assert tp is not None
        assert ts is not None
