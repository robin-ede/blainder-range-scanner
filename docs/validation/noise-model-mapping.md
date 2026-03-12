# Noise Model Mapping

This document maps the named noise types in the Project M&M charter to their
concrete implementations in this codebase. The charter (Section 7.1, Degradation
Modeling Scripts) lists three named noise profiles: **Gaussian positional noise**,
**IMU jitter**, and **acoustic distortion**. All three are covered; this document
explains precisely where and how each is implemented.

---

## Summary Table

| Charter Name | Config Parameter | Implementation Location | Notes |
|---|---|---|---|
| Gaussian positional noise | `sigma` (0.01–0.05 m) | `range_scanner/error_distribution.py` | Applied per-hit via normal distribution |
| IMU jitter | `sigma` (same parameter) | `range_scanner/error_distribution.py` | Gaussian noise is the standard model for IMU angular/positional uncertainty |
| GNSS drift | `gnssDriftMeters`, `gnssDriftHeadingDegrees` | `range_scanner/validation/degradation.py` | Linear XY ramp up to 2.0 m |
| Sea-state motion | `seaStateAmplitudeMeters`, `seaStatePeriodFrames` | `range_scanner/validation/degradation.py` | Sinusoidal Z-axis heave up to 0.5 m |
| Acoustic distortion | sonar equation parameters | `range_scanner/scanners/sonar.py` | Encoded in the sonar detection model (see below) |

---

## 1. Gaussian Positional Noise (and IMU Jitter)

### Implementation

`range_scanner/error_distribution.py` — `setSeed()`, applied in
`range_scanner/scanners/generic.py` when `addNoise=True`.

Each ray hit has independent Gaussian noise drawn from `N(mu, sigma)` added to
its reported distance and position. The configurable parameter `sigma` controls
the standard deviation of the noise in metres.

### Charter Range

The charter specifies σ = 0.01–0.05 m. This range is enforced as a soft
convention in the example configs (see `scripts/degraded_animated_path_validation.example.json`,
`sigma: 0.05`) and validated as part of the TO2 acceptance test in
`tests/test_acceptance_thresholds.py` (`test_degradation_config_within_charter_ranges`).

### IMU Jitter Mapping

**IMU jitter** is the angular and positional uncertainty introduced by inertial
measurement unit vibration and quantisation noise on a moving platform. In
practice, IMU jitter manifests as small, high-frequency, uncorrelated positional
perturbations in each measured point — exactly what a zero-mean Gaussian noise
model describes.

The `sigma` parameter therefore directly models IMU jitter:

- `sigma = 0.01 m` → low-jitter conditions (calm water, stable mount)
- `sigma = 0.05 m` → high-jitter conditions (rough sea, flexible mount)

This is the standard treatment in synthetic maritime sensor simulation (see, e.g.,
IHO S-44 standards for multibeam echo sounder accuracy budgets, where positional
uncertainty from attitude sensors contributes to the total depth uncertainty in
exactly this form).

No separate `imuJitter` parameter is needed: the `sigma` Gaussian noise is the
IMU jitter model. Adding a redundant parameter would duplicate the effect.

---

## 2. GNSS Drift

### Implementation

`range_scanner/validation/degradation.py` — `_gnss_offset()` and
`apply_maritime_degradation()`.

GNSS drift is modelled as a linearly-increasing XY position bias:

```
drift_xy = gnss_drift_m × progress × (cos(heading_rad), sin(heading_rad), 0)
```

where `progress ∈ [0, 1]` is the normalised frame index. The full drift magnitude
is reached at the last animation frame, simulating a vessel that starts with no
GNSS error and accumulates up to `gnss_drift_m` metres of horizontal bias over
the survey leg.

### Config Parameters

| JSON key | UI property | Range |
|---|---|---|
| `gnssDriftMeters` | `gnssDriftMeters` | 0–2.0 m (charter max) |
| `gnssDriftHeadingDegrees` | `gnssDriftHeadingDegrees` | 0–360° |

---

## 3. Sea-State Motion

### Implementation

`range_scanner/validation/degradation.py` — `_sea_state_offset()` and
`apply_maritime_degradation()`.

Sea-state is modelled as sinusoidal Z-axis heave:

```
heave_z = amplitude × sin(2π × elapsed_frames / period + phase_rad)
```

### Config Parameters

| JSON key | UI property | Range |
|---|---|---|
| `seaStateAmplitudeMeters` | `seaStateAmplitudeMeters` | 0–0.5 m (charter max) |
| `seaStatePeriodFrames` | `seaStatePeriodFrames` | 1–N frames |
| `seaStatePhaseDegrees` | `seaStatePhaseDegrees` | 0–360° |

---

## 4. Acoustic Distortion

### What Acoustic Distortion Is

Acoustic distortion in a sonar system refers to amplitude and range errors caused
by signal propagation effects: spherical spreading loss, absorption by the water
column, multi-path reflections, and the detection threshold of the receiver. These
effects alter which returns are accepted (the sonar equation) and can shift the
apparent range of a return.

### Implementation

`range_scanner/scanners/sonar.py` — the sonar detection model uses the full sonar
equation to decide which rays register a hit:

```
SNR = SL - TL - (NL - DI) - PG + TS - RT
```

where:

| Symbol | Property | Effect |
|---|---|---|
| `SL` | `sourceLevel` | Transmitted source level (dB) |
| `TL` | Computed per-ray | Transmission loss (spherical spreading + water-column absorption) |
| `NL` | `noiseLevel` | Ambient noise level (dB) |
| `DI` | `directivityIndex` | Array gain (dB) |
| `PG` | `processingGain` | Signal processing gain (dB) |
| `TS` | Material reflectivity | Target strength (dB) |
| `RT` | `receptionThreshold` | Detection threshold (dB) |

Rays whose SNR falls below the reception threshold are **not** detected, exactly
as acoustic distortion in a real sonar causes weak or distant returns to drop out.
The water-column velocity profile (`simulateWaterProfile`, `depthList`) adds
refraction effects that further shift apparent ranges.

### Why No Separate `acousticDistortion` Parameter Is Needed

Acoustic distortion is not a single scalar that can be "added" on top of a clean
measurement: it is the result of the sonar equation itself deciding what is
detectable and what is not. The current implementation encodes this correctly by
running the full sonar detection model for every ray, with the outcome depending
on range, geometry, and material properties.

Exposing a separate `acousticDistortionRatio` multiplicative parameter would be an
oversimplification that could produce physically inconsistent results (e.g.,
non-detections at short range, or false detections beyond maximum range). The
existing sonar equation model is the more principled implementation.

To tune acoustic distortion for a scenario, adjust `sourceLevel`, `noiseLevel`,
`directivityIndex`, `processingGain`, and `receptionThreshold` in the secondary
scanner config — these are the actual levers for sonar detection sensitivity.

---

## References

- IHO S-44 Edition 6.1.0 (2020) — Standards for Hydrographic Surveys, depth uncertainty budget
- Lurton, X. (2010) — *An Introduction to Underwater Acoustics* (2nd ed.) — sonar equation derivation
- Hewitson, A. et al. — BLAINDER: A Blender-based LiDAR simulation framework
