"""
Unit tests for range_scanner.validation.evaluation — report assembly functions.

Covers:
  - build_validation_report
  - write_validation_report

bpy.path.abspath is already mocked as identity in conftest.py, so
write_validation_report can write to real temp directories without Blender.
"""

import json
import os
import tempfile
import types
from datetime import datetime, timezone

import pytest

from range_scanner.validation.evaluation import (
    build_validation_report,
    write_validation_report,
)


# ---------------------------------------------------------------------------
# Helpers: minimal mock properties and results
# ---------------------------------------------------------------------------


def _make_scanner_object(name="Scanner"):
    obj = types.SimpleNamespace()
    obj.name = name
    obj.location = [1.0, 2.0, 3.0]
    obj.rotation_euler = [0.0, 0.1, 0.2]
    return obj


def _make_properties(**overrides):
    """Return a minimal properties namespace that satisfies build_validation_report."""
    props = types.SimpleNamespace(
        scannerObject=_make_scanner_object("Primary"),
        scannerObject2=None,
        scannerType="lidar",
        scannerType2="sonar",
        addNoise=True,
        noiseType="gaussian",
        mu=0.0,
        sigma=0.01,
        addConstantNoise=False,
        noiseAbsoluteOffset=0.0,
        noiseRelativeOffset=0.0,
        dataFileName="test_data",
        dataFilePath="//data/",
        frameStart=1,
        frameEnd=10,
        frameStep=1,
        singleRay=False,
        enableAnimation=True,
        noiseSeed=42,
        gnssDriftMeters=0.5,
        gnssDriftHeadingDegrees=90.0,
        seaStateAmplitudeMeters=0.1,
        seaStatePeriodFrames=20,
        seaStatePhaseDegrees=0.0,
    )
    for key, value in overrides.items():
        setattr(props, key, value)
    return props


def _make_results(**overrides):
    """Return a minimal results dict that satisfies build_validation_report."""
    results = {
        "evaluation_mode": "bvh_surface",
        "evaluated_point_variant": "noisy",
        "ground_truth_source": "mesh_bvh",
        "hit_counts": {"total": 100, "valid": 95},
        "metrics": {"rmse": 0.02, "mae": 0.015},
        "per_sensor_pre_fusion_metrics": {},
        "acceptance": {"passes_rmse_threshold": True, "rmse_threshold_m": 0.05},
    }
    results.update(overrides)
    return results


# ---------------------------------------------------------------------------
# build_validation_report — structure and values
# ---------------------------------------------------------------------------


class TestBuildValidationReport:
    def test_schema_version_is_always_1(self):
        report = build_validation_report(_make_properties(), _make_results())
        assert report["schema_version"] == 1

    def test_status_pass_when_threshold_met(self):
        results = _make_results(
            acceptance={"passes_rmse_threshold": True, "rmse_threshold_m": 0.05}
        )
        report = build_validation_report(_make_properties(), results)
        assert report["status"] == "pass"

    def test_status_fail_when_threshold_not_met(self):
        results = _make_results(
            acceptance={"passes_rmse_threshold": False, "rmse_threshold_m": 0.05}
        )
        report = build_validation_report(_make_properties(), results)
        assert report["status"] == "fail"

    def test_aggregation_scope_animation(self):
        props = _make_properties(enableAnimation=True)
        report = build_validation_report(props, _make_results())
        assert report["aggregation_scope"] == "accumulated_over_animation_frames"

    def test_aggregation_scope_single_frame(self):
        props = _make_properties(enableAnimation=False)
        report = build_validation_report(props, _make_results())
        assert report["aggregation_scope"] == "single_frame_snapshot"

    def test_scanner_config_primary_name(self):
        props = _make_properties()
        report = build_validation_report(props, _make_results())
        assert report["scanner_config"]["primary"]["object"] == "Primary"

    def test_scanner_config_secondary_none_when_no_scanner2(self):
        props = _make_properties(scannerObject2=None)
        report = build_validation_report(props, _make_results())
        assert report["scanner_config"]["secondary"]["object"] is None
        assert report["scanner_config"]["secondary"]["location"] is None

    def test_scanner_config_both_populated(self):
        props = _make_properties(scannerObject2=_make_scanner_object("Secondary"))
        report = build_validation_report(props, _make_results())
        assert report["scanner_config"]["secondary"]["object"] == "Secondary"
        assert report["scanner_config"]["secondary"]["location"] is not None

    def test_experiment_config_uses_getattr_with_defaults(self):
        # Properties without optional experiment fields should not raise
        props = _make_properties()
        del props.noiseSeed
        del props.gnssDriftMeters
        report = build_validation_report(props, _make_results())
        assert report["experiment_config"]["noise_seed"] is None
        assert report["experiment_config"]["gnss_drift_m"] is None

    def test_warnings_defaults_to_empty_list(self):
        report = build_validation_report(_make_properties(), _make_results())
        assert report["warnings"] == []

    def test_warnings_passed_through(self):
        warnings = ["something degraded", "alignment failed"]
        report = build_validation_report(
            _make_properties(), _make_results(), warnings=warnings
        )
        assert report["warnings"] == warnings

    def test_optional_result_keys_are_none_when_absent(self):
        # results dict has no optional keys; they should all be None
        report = build_validation_report(_make_properties(), _make_results())
        optional_keys = [
            "regional_pre_fusion_metrics",
            "medium_filtering",
            "degradation_modeling",
            "post_processing_fusion",
            "blind_post_processing_fusion",
            "medium_separated_reconstruction",
            "medium_height_grid_reconstruction",
            "alignment_correction",
            "blind_alignment_correction",
            "corrected_grid_reconstruction",
            "blind_corrected_grid_reconstruction",
            "depth_distribution_comparison",
            "performance",
            "technical_objectives",
            "error_reduction",
            "per_frame_degraded_rmse",
        ]
        for key in optional_keys:
            assert report[key] is None, (
                f"Expected {key!r} to be None, got {report[key]!r}"
            )

    def test_optional_result_keys_populated_when_present(self):
        results = _make_results(
            error_reduction={"rmse_reduction_pct": 42.0},
            regional_metrics={"above_water": {}, "below_water": {}},
        )
        report = build_validation_report(_make_properties(), results)
        assert report["error_reduction"]["rmse_reduction_pct"] == 42.0
        assert report["regional_pre_fusion_metrics"] is not None

    def test_timestamp_is_iso8601(self):
        report = build_validation_report(_make_properties(), _make_results())
        ts = report["timestamp_utc"]
        # Should parse without raising
        parsed = datetime.fromisoformat(ts)
        assert parsed.tzinfo is not None

    def test_noise_config_values(self):
        props = _make_properties(
            addNoise=True, noiseType="gaussian", mu=0.1, sigma=0.05
        )
        report = build_validation_report(props, _make_results())
        nc = report["noise_config"]
        assert nc["add_noise"] is True
        assert nc["noise_type"] == "gaussian"
        assert nc["mu"] == pytest.approx(0.1)
        assert nc["sigma"] == pytest.approx(0.05)

    def test_scenario_fields(self):
        props = _make_properties(frameStart=5, frameEnd=50, frameStep=2, singleRay=True)
        report = build_validation_report(props, _make_results())
        sc = report["scenario"]
        assert sc["frame_start"] == 5
        assert sc["frame_end"] == 50
        assert sc["frame_step"] == 2
        assert sc["single_ray"] is True


# ---------------------------------------------------------------------------
# write_validation_report
# ---------------------------------------------------------------------------


class TestWriteValidationReport:
    def test_creates_file_and_returns_path(self):
        report = {"schema_version": 1, "status": "pass"}
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = write_validation_report(tmpdir, "my_scan", report)
            assert os.path.isfile(out_path)
            assert out_path.endswith("my_scan_report.json")

    def test_written_json_roundtrips_correctly(self):
        report = build_validation_report(_make_properties(), _make_results())
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = write_validation_report(tmpdir, "roundtrip_test", report)
            with open(out_path, encoding="utf-8") as fh:
                loaded = json.load(fh)
        assert loaded["schema_version"] == report["schema_version"]
        assert loaded["status"] == report["status"]

    def test_creates_directory_if_missing(self):
        report = {"schema_version": 1}
        with tempfile.TemporaryDirectory() as tmpdir:
            nested = os.path.join(tmpdir, "sub", "nested")
            # Directory does not yet exist
            out_path = write_validation_report(nested, "nested_test", report)
            assert os.path.isfile(out_path)

    def test_output_is_valid_json(self):
        report = build_validation_report(_make_properties(), _make_results())
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = write_validation_report(tmpdir, "valid_json", report)
            with open(out_path, encoding="utf-8") as fh:
                content = fh.read()
        # Should not raise
        parsed = json.loads(content)
        assert isinstance(parsed, dict)
