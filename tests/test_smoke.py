#!/usr/bin/env python3
"""
INAV Toolkit test suite.

Run:  python3 -m pytest tests/ -v
  or: python3 tests/test_smoke.py          (standalone, no pytest needed)

Requires: pip install -e ".[test]"
"""
import json
import os
import subprocess
import sys

import numpy as np
import pytest

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TESTS_DIR = SCRIPT_DIR
FIXTURES_DIR = os.path.join(TESTS_DIR, "fixtures")
sys.path.insert(0, PROJECT_DIR)


# ─── Helpers ──────────────────────────────────────────────────────────────────

def run_module(module, *args):
    """Run a toolkit module via python -m, return (stdout, stderr, rc)."""
    result = subprocess.run(
        [sys.executable, "-m", module, *args],
        capture_output=True, text=True, cwd=PROJECT_DIR)
    return result.stdout, result.stderr, result.returncode


def run_entry(cmd, *args):
    """Run an entry point command, return (stdout, stderr, rc)."""
    result = subprocess.run(
        [cmd, *args],
        capture_output=True, text=True, cwd=PROJECT_DIR)
    return result.stdout, result.stderr, result.returncode


# ═════════════════════════════════════════════════════════════════════════════
# Filter Math (parametrized)
# ═════════════════════════════════════════════════════════════════════════════

class TestFilterMath:
    """Unit tests for filter phase shift calculations."""

    @pytest.mark.parametrize("ftype,freq,cutoff,expected", [
        ("PT1",    100, 100, -45.0),
        ("PT2",    100, 100, -90.0),
        ("PT3",    100, 100, -135.0),
        ("PT1",    50,  100, -26.6),
        ("PT1",    200, 100, -63.4),
    ])
    def test_phase_shift(self, ftype, freq, cutoff, expected):
        from inav_toolkit.blackbox_analyzer import _phase_shift
        result = _phase_shift(ftype, freq, cutoff)
        assert abs(result - expected) < 1.0, f"{ftype} at {freq}Hz: got {result}, expected {expected}"

    @pytest.mark.parametrize("q,expected", [
        (0.5,  -90.0),
        (250,  -90.0),
    ])
    def test_biquad_at_cutoff(self, q, expected):
        from inav_toolkit.blackbox_analyzer import _phase_shift
        result = _phase_shift("BIQUAD", 100, 100, q=q)
        assert abs(result - expected) < 0.1

    def test_biquad_below_cutoff(self):
        from inav_toolkit.blackbox_analyzer import _phase_shift
        result = _phase_shift("BIQUAD", 10, 100, q=0.5)
        assert abs(result) < 15

    def test_filter_phase_lag_returns_dict(self):
        from inav_toolkit.blackbox_analyzer import estimate_filter_phase_lag
        lag = estimate_filter_phase_lag(65, 50, "PT1")
        assert "degrees" in lag and "ms" in lag
        assert abs(lag["degrees"] - (-37.6)) < 1

    def test_filter_phase_lag_zero_cutoff(self):
        from inav_toolkit.blackbox_analyzer import estimate_filter_phase_lag
        lag = estimate_filter_phase_lag(0, 50, "PT1")
        assert lag["degrees"] == 0.0 and lag["ms"] == 0.0


# ═════════════════════════════════════════════════════════════════════════════
# Version Flags
# ═════════════════════════════════════════════════════════════════════════════

class TestVersionFlags:
    """Verify --version works on all entry points."""

    @pytest.mark.parametrize("entry_point", [
        "inav-analyze", "inav-params", "inav-msp", "inav-toolkit",
    ])
    def test_entry_point_version(self, entry_point):
        out, _, rc = run_entry(entry_point, "--version")
        assert rc == 0
        from inav_toolkit import __version__
        assert __version__ in out

    def test_module_version_consistent(self):
        """All modules should have the same version as __init__."""
        from inav_toolkit import __version__
        from inav_toolkit.blackbox_analyzer import REPORT_VERSION
        assert REPORT_VERSION == __version__


# ═════════════════════════════════════════════════════════════════════════════
# Blackbox Analyzer: Imports
# ═════════════════════════════════════════════════════════════════════════════

class TestBlackboxImports:

    def test_report_version(self):
        from inav_toolkit.blackbox_analyzer import REPORT_VERSION
        assert REPORT_VERSION is not None

    def test_fingerprint_callable(self):
        from inav_toolkit.blackbox_analyzer import fingerprint_noise, format_noise_fingerprint_terminal
        assert callable(fingerprint_noise) and callable(format_noise_fingerprint_terminal)


# ═════════════════════════════════════════════════════════════════════════════
# Noise Fingerprinting
# ═════════════════════════════════════════════════════════════════════════════

class TestNoiseFingerprinting:

    def test_empty_results(self):
        from inav_toolkit.blackbox_analyzer import fingerprint_noise
        result = fingerprint_noise([None, None, None], {})
        assert result["dominant_source"] == "none"
        assert result["peaks"] == []

    def test_with_peaks(self):
        from inav_toolkit.blackbox_analyzer import fingerprint_noise
        fake_noise = [
            {
                "axis": "Roll",
                "peaks": [
                    {"freq_hz": 160.0, "power_db": -8.0, "prominence": 15.0},
                    {"freq_hz": 55.0, "power_db": -15.0, "prominence": 10.0},
                ],
                "noise_start_freq": 120.0,
                "rms_low": -20, "rms_mid": -15, "rms_high": -10,
                "freqs": np.array([0]), "psd_db": np.array([0]),
            },
            {
                "axis": "Pitch",
                "peaks": [
                    {"freq_hz": 158.0, "power_db": -9.0, "prominence": 14.0},
                ],
                "noise_start_freq": 115.0,
                "rms_low": -22, "rms_mid": -16, "rms_high": -12,
                "freqs": np.array([0]), "psd_db": np.array([0]),
            },
            None,
        ]
        result = fingerprint_noise(fake_noise, {"_n_motors": 4})
        assert len(result["peaks"]) >= 1
        assert result["dominant_source"] not in ("none", "clean")
        for p in result["peaks"]:
            for key in ("freq_hz", "source", "confidence", "detail"):
                assert key in p

    def test_prop_harmonics_matching(self):
        from inav_toolkit.blackbox_analyzer import fingerprint_noise
        fake_noise = [{
            "axis": "Roll",
            "peaks": [{"freq_hz": 250.0, "power_db": -5.0, "prominence": 20.0}],
            "noise_start_freq": 200.0,
            "rms_low": -30, "rms_mid": -20, "rms_high": -5,
            "freqs": np.array([0]), "psd_db": np.array([0]),
        }]
        harmonics = [{"harmonic": 1, "min_hz": 200, "max_hz": 300, "label": "fundamental"}]
        result = fingerprint_noise(fake_noise, {"_n_motors": 4}, prop_harmonics=harmonics)
        matched = [p for p in result["peaks"] if p["source"] == "prop_harmonics"]
        assert len(matched) >= 1
        assert matched[0]["confidence"] == "high"

    def test_cross_axis_structural(self):
        from inav_toolkit.blackbox_analyzer import fingerprint_noise
        fake_noise = []
        for axis in ["Roll", "Pitch", "Yaw"]:
            fake_noise.append({
                "axis": axis,
                "peaks": [{"freq_hz": 85.0, "power_db": -12.0, "prominence": 10.0}],
                "noise_start_freq": 60.0,
                "rms_low": -18, "rms_mid": -14, "rms_high": -20,
                "freqs": np.array([0]), "psd_db": np.array([0]),
            })
        result = fingerprint_noise(fake_noise, {"_n_motors": 4})
        structural = [p for p in result["peaks"] if p["source"] == "structural"]
        assert len(structural) >= 1

    def test_fingerprint_has_remedies(self):
        """Enhanced fingerprinting should include remedy suggestions."""
        from inav_toolkit.blackbox_analyzer import fingerprint_noise
        fake_noise = [{
            "axis": "Roll",
            "peaks": [{"freq_hz": 160.0, "power_db": -8.0, "prominence": 15.0}],
            "noise_start_freq": 120.0,
            "rms_low": -20, "rms_mid": -15, "rms_high": -10,
            "freqs": np.array([0]), "psd_db": np.array([0]),
        }]
        result = fingerprint_noise(fake_noise, {"_n_motors": 4})
        for p in result["peaks"]:
            assert "remedy" in p, f"Peak at {p['freq_hz']}Hz missing remedy field"
            assert len(p["remedy"]) > 0


# ═════════════════════════════════════════════════════════════════════════════
# RPM / Prop Harmonics
# ═════════════════════════════════════════════════════════════════════════════

class TestRPMEstimation:

    def test_rpm_range(self):
        from inav_toolkit.blackbox_analyzer import estimate_rpm_range
        idle, maxrpm = estimate_rpm_range(motor_kv=900, cell_count=6)
        assert 0 < idle < maxrpm
        assert 15000 < maxrpm < 25000

    def test_prop_harmonics(self):
        from inav_toolkit.blackbox_analyzer import estimate_prop_harmonics
        h = estimate_prop_harmonics((3000, 20000), n_blades=3)
        assert len(h) == 3
        assert h[0]["label"] == "fundamental"
        assert h[0]["min_hz"] < h[0]["max_hz"]
        assert abs(h[0]["max_hz"] - 1000) < 1


# ═════════════════════════════════════════════════════════════════════════════
# Frame Profiles (parametrized)
# ═════════════════════════════════════════════════════════════════════════════

class TestFrameProfiles:

    @pytest.mark.parametrize("size", [5, 7, 10, 12, 15])
    def test_profile_exists(self, size):
        from inav_toolkit.blackbox_analyzer import get_frame_profile
        p = get_frame_profile(size)
        assert p["frame_inches"] == size
        for key in ("gyro_lpf_range", "ok_overshoot", "dterm_lpf_range", "filter_safety"):
            assert key in p

    def test_larger_frame_wider_filters(self):
        """Larger frames should have lower filter cutoffs."""
        from inav_toolkit.blackbox_analyzer import get_frame_profile
        p5 = get_frame_profile(5)
        p10 = get_frame_profile(10)
        assert p10["gyro_lpf_range"][0] < p5["gyro_lpf_range"][0]


# ═════════════════════════════════════════════════════════════════════════════
# Filter Recommendation Engine
# ═════════════════════════════════════════════════════════════════════════════

class TestFilterRecommendation:

    def test_compute_recommended_filter_basic(self):
        from inav_toolkit.blackbox_analyzer import compute_recommended_filter, get_frame_profile
        freqs = np.linspace(0, 250, 500)
        psd = np.full_like(freqs, -45.0)
        psd[freqs > 80] = -20.0

        noise_results = [{
            "axis": "Roll", "freqs": freqs, "psd_db": psd,
            "peaks": [{"freq_hz": 120.0, "power_db": -15.0, "prominence": 10.0}],
            "noise_start_freq": 80.0,
            "rms_low": -40, "rms_mid": -20, "rms_high": -25,
        }]
        profile = get_frame_profile(5)
        result = compute_recommended_filter(noise_results, 100, "gyro", profile)
        assert result is not None
        assert 40 <= result <= 90

    def test_clean_spectrum_no_change(self):
        from inav_toolkit.blackbox_analyzer import compute_recommended_filter, get_frame_profile
        freqs = np.linspace(0, 250, 500)
        psd = np.full_like(freqs, -50.0)
        noise_results = [{
            "axis": "Roll", "freqs": freqs, "psd_db": psd,
            "peaks": [],
            "noise_start_freq": 250.0,
            "rms_low": -50, "rms_mid": -50, "rms_high": -50,
        }]
        profile = get_frame_profile(5)
        result = compute_recommended_filter(noise_results, 100, "gyro", profile)
        assert result is None or result >= 80

    def test_compute_filter_recommendations(self):
        """Comprehensive filter recommendation with notch suggestions."""
        from inav_toolkit.blackbox_analyzer import compute_filter_recommendations, get_frame_profile
        freqs = np.linspace(0, 250, 500)
        psd = np.full_like(freqs, -45.0)
        spike_idx = np.argmin(np.abs(freqs - 160))
        psd[spike_idx - 2:spike_idx + 3] = -5.0

        noise_results = [{
            "axis": "Roll", "freqs": freqs, "psd_db": psd,
            "peaks": [{"freq_hz": 160.0, "power_db": -5.0, "prominence": 20.0}],
            "noise_start_freq": 80.0,
            "rms_low": -40, "rms_mid": -20, "rms_high": -25,
        }]
        profile = get_frame_profile(5)
        recs = compute_filter_recommendations(noise_results, {"_n_motors": 4}, profile)
        assert recs is not None
        assert "gyro_lpf_hz" in recs


# ═════════════════════════════════════════════════════════════════════════════
# Param Analyzer
# ═════════════════════════════════════════════════════════════════════════════

class TestParamAnalyzer:

    @pytest.mark.parametrize("frame_size", ["5", "7", "10"])
    def test_setup_mode(self, frame_size):
        out, err, rc = run_module("inav_toolkit.param_analyzer", "--setup", frame_size, "--voltage", "6S")
        assert rc == 0
        assert "mc_p_roll" in out.lower() or "set mc_p_roll" in out.lower()

    def test_setup_json(self):
        out, err, rc = run_module("inav_toolkit.param_analyzer", "--setup", "10", "--json")
        assert rc == 0
        data = json.loads(out)
        assert isinstance(data, dict)

    def test_diff_analysis(self):
        diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
        if not os.path.exists(diff_path):
            pytest.skip("Fixture not found")
        out, err, rc = run_module("inav_toolkit.param_analyzer", diff_path)
        assert rc == 0
        assert "SUMMARY" in out


# ═════════════════════════════════════════════════════════════════════════════
# VTOL Configurator
# ═════════════════════════════════════════════════════════════════════════════

class TestVTOLConfigurator:

    def test_non_vtol_diff(self):
        diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
        if not os.path.exists(diff_path):
            pytest.skip("Fixture not found")
        out, err, rc = run_module("inav_toolkit.vtol_configurator", diff_path)
        assert rc == 0

    def test_vtol_analysis(self):
        diff_path = os.path.join(TESTS_DIR, "test_vtol_diff.txt")
        if not os.path.exists(diff_path):
            pytest.skip("Fixture not found")
        out, err, rc = run_module("inav_toolkit.vtol_configurator", diff_path)
        assert rc == 0
        assert "TRICOPTER" in out

    def test_vtol_json(self):
        diff_path = os.path.join(TESTS_DIR, "test_vtol_diff.txt")
        if not os.path.exists(diff_path):
            pytest.skip("Fixture not found")
        out, err, rc = run_module("inav_toolkit.vtol_configurator", diff_path, "--json")
        assert rc == 0
        data = json.loads(out)
        assert isinstance(data, list)


# ═════════════════════════════════════════════════════════════════════════════
# End-to-End Pipeline Tests (using synthetic fixtures)
# ═════════════════════════════════════════════════════════════════════════════

class TestE2EPipeline:
    """End-to-end analysis pipeline tests using synthetic CSV data."""

    def test_parse_clean_hover(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log
        csv_path = os.path.join(FIXTURES_DIR, "clean_hover.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        assert data["n_rows"] == 4000
        assert abs(data["sample_rate"] - 500.0) < 10.0
        for key in ("gyro_roll", "gyro_pitch", "gyro_yaw",
                     "setpoint_roll", "motor0", "motor1"):
            assert key in data

    def test_noise_analysis_clean(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log, analyze_noise
        csv_path = os.path.join(FIXTURES_DIR, "clean_hover.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        sr = data["sample_rate"]
        roll_noise = analyze_noise(data, "Roll", "gyro_roll", sr)
        assert roll_noise is not None
        strong_peaks = [p for p in roll_noise["peaks"] if p["power_db"] > -10 and p["freq_hz"] > 20]
        assert len(strong_peaks) == 0, f"Clean hover shouldn't have strong peaks above 20Hz: {strong_peaks}"

    def test_noise_analysis_noisy(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log, analyze_noise
        csv_path = os.path.join(FIXTURES_DIR, "noisy_motors.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        sr = data["sample_rate"]
        roll_noise = analyze_noise(data, "Roll", "gyro_roll", sr)
        assert roll_noise is not None
        peak_freqs = [p["freq_hz"] for p in roll_noise["peaks"]]
        found_160 = any(140 <= f <= 180 for f in peak_freqs)
        found_85 = any(70 <= f <= 100 for f in peak_freqs)
        assert found_160, f"Expected peak near 160Hz, got: {peak_freqs}"
        assert found_85, f"Expected peak near 85Hz, got: {peak_freqs}"

    def test_full_fingerprint_on_noisy_data(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log, analyze_noise, fingerprint_noise
        csv_path = os.path.join(FIXTURES_DIR, "noisy_motors.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        sr = data["sample_rate"]
        noise_results = []
        for axis, key in [("Roll", "gyro_roll"), ("Pitch", "gyro_pitch"), ("Yaw", "gyro_yaw")]:
            noise_results.append(analyze_noise(data, axis, key, sr))
        fp = fingerprint_noise(noise_results, {"_n_motors": 4})
        assert fp["dominant_source"] != "none"
        assert len(fp["peaks"]) >= 1
        for p in fp["peaks"]:
            assert "remedy" in p

    def test_short_flight_survives(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log, analyze_noise
        csv_path = os.path.join(FIXTURES_DIR, "short_flight.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        assert data["n_rows"] == 600
        sr = data["sample_rate"]
        roll_noise = analyze_noise(data, "Roll", "gyro_roll", sr)
        assert roll_noise is not None

    def test_motor_analysis(self):
        from inav_toolkit.blackbox_analyzer import parse_csv_log, analyze_motors
        csv_path = os.path.join(FIXTURES_DIR, "noisy_motors.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        motor_result = analyze_motors(data, data["sample_rate"])
        assert motor_result is not None
        assert "balance_spread_pct" in motor_result

    def test_filter_recommendation_pipeline(self):
        from inav_toolkit.blackbox_analyzer import (
            parse_csv_log, analyze_noise, compute_recommended_filter, get_frame_profile
        )
        csv_path = os.path.join(FIXTURES_DIR, "noisy_motors.csv")
        if not os.path.exists(csv_path):
            pytest.skip("Run generate_fixtures.py first")
        data = parse_csv_log(csv_path)
        sr = data["sample_rate"]
        noise_results = []
        for axis, key in [("Roll", "gyro_roll"), ("Pitch", "gyro_pitch"), ("Yaw", "gyro_yaw")]:
            noise_results.append(analyze_noise(data, axis, key, sr))
        profile = get_frame_profile(5)
        rec = compute_recommended_filter(noise_results, 100, "gyro", profile)
        assert rec is not None
        assert 30 <= rec <= 200


# ═════════════════════════════════════════════════════════════════════════════
# Multi-flight Trend Analysis
# ═════════════════════════════════════════════════════════════════════════════

class TestTrendAnalysis:

    def test_trend_data_generation(self):
        import tempfile
        from inav_toolkit.flight_db import FlightDB
        with tempfile.TemporaryDirectory() as tmpdir:
            db = FlightDB(os.path.join(tmpdir, "test.db"))
            for i, score in enumerate([55, 62, 68, 75]):
                plan = {
                    "scores": {
                        "overall": score, "noise": score + 5, "pid": score - 5,
                        "pid_measurable": True, "motor": 80, "gyro_oscillation": score,
                    },
                    "verdict": "OK" if score > 60 else "NEEDS_WORK",
                    "verdict_text": "Test flight",
                    "actions": [],
                    "noise_fingerprint": {"peaks": [], "dominant_source": "clean", "summary": ""},
                }
                config = {"craft_name": "TEST_QUAD", "_duration_s": 120 + i * 10,
                          "_n_motors": 4, "looptime": "1000"}
                data = {"sample_rate": 500.0, "time_s": np.arange(1000 + i * 500) / 500.0}
                hover_osc = [
                    {"axis": "Roll", "severity": "low", "gyro_rms": 3.0 - i * 0.3, "gyro_p2p": 8.0},
                    {"axis": "Pitch", "severity": "low", "gyro_rms": 2.8 - i * 0.2, "gyro_p2p": 7.0},
                    {"axis": "Yaw", "severity": "low", "gyro_rms": 1.5, "gyro_p2p": 4.0},
                ]
                db.store_flight(plan, config, data, hover_osc=hover_osc)
            prog = db.get_progression("TEST_QUAD")
            assert prog["trend"] in ("improving", "stable"), f"Got: {prog}"
            assert len(prog["flights"]) >= 2
            db.close()

    def test_generate_trend_html(self):
        """Test HTML trend report generation."""
        import tempfile
        from inav_toolkit.flight_db import FlightDB
        try:
            from inav_toolkit.blackbox_analyzer import generate_trend_report
        except ImportError:
            pytest.skip("generate_trend_report not yet implemented")
        with tempfile.TemporaryDirectory() as tmpdir:
            db = FlightDB(os.path.join(tmpdir, "test.db"))
            for i, score in enumerate([55, 62, 68, 75, 80]):
                plan = {
                    "scores": {
                        "overall": score, "noise": score + 5, "pid": score - 5,
                        "pid_measurable": True, "motor": 80, "gyro_oscillation": score,
                    },
                    "verdict": "OK",
                    "verdict_text": "Test flight",
                    "actions": [],
                    "noise_fingerprint": {"peaks": [], "dominant_source": "clean", "summary": ""},
                }
                config = {"craft_name": "TEST_QUAD", "_duration_s": 120 + i * 30,
                          "_n_motors": 4, "looptime": "1000"}
                data = {"sample_rate": 500.0, "time_s": np.arange(1000 + i * 500) / 500.0}
                hover_osc = [
                    {"axis": "Roll", "severity": "low", "gyro_rms": 3.0 - i * 0.2, "gyro_p2p": 8.0},
                    {"axis": "Pitch", "severity": "low", "gyro_rms": 2.8 - i * 0.15, "gyro_p2p": 7.0},
                    {"axis": "Yaw", "severity": "low", "gyro_rms": 1.5, "gyro_p2p": 4.0},
                ]
                db.store_flight(plan, config, data, hover_osc=hover_osc)
            prog = db.get_progression("TEST_QUAD", limit=20)
            html_path = os.path.join(tmpdir, "trend.html")
            generate_trend_report(prog, "TEST_QUAD", html_path)
            assert os.path.exists(html_path)
            with open(html_path) as f:
                html = f.read()
            assert "TEST_QUAD" in html
            assert "Score" in html or "score" in html
            db.close()


# ═════════════════════════════════════════════════════════════════════════════
# Standalone runner (works without pytest)
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    test_items = []
    for name, obj in sorted(globals().items()):
        if isinstance(obj, type) and name.startswith("Test"):
            for method_name in sorted(dir(obj)):
                if method_name.startswith("test_"):
                    method = getattr(obj, method_name)
                    if callable(method):
                        test_items.append((f"{name}.{method_name}", obj, method_name))

    passed = failed = skipped = 0
    print("=" * 60)
    print("  INAV Toolkit -- Test Suite")
    print("=" * 60)

    current_class = ""
    for full_name, cls, method_name in test_items:
        class_name = full_name.split(".")[0]
        if class_name != current_class:
            current_class = class_name
            print(f"\n-- {class_name} --")
        try:
            import inspect
            method = getattr(cls(), method_name)
            sig = inspect.signature(method)
            params = [p for p in sig.parameters if p != "self"]
            if params:
                skipped += 1
                print(f"  ~ {method_name} (parametrized, use pytest)")
                continue
            method()
            passed += 1
            print(f"  + {method_name}")
        except (SystemExit,):
            skipped += 1
            print(f"  ~ {method_name} (skipped)")
        except Exception as e:
            failed += 1
            print(f"  X {method_name}")
            print(f"    {e}")

    print(f"\n{'=' * 60}")
    if failed == 0:
        print(f"  {passed} PASSED, {skipped} skipped")
    else:
        print(f"  {passed} passed, {skipped} skipped, {failed} FAILED")
    print(f"{'=' * 60}")
    sys.exit(1 if failed else 0)
