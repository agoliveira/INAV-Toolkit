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


class TestSanityCheck:
    """Test pre-flight sanity check engine."""

    def _make_diff(self, **overrides):
        """Build a minimal diff all text with overrides."""
        lines = [
            "# INAV/TESTBOARD 9.0.1 Feb 22 2026 / 12:00:00 (abc123)",
        ]
        settings = {
            "name": "TEST_QUAD",
            "platform_type": "MULTIROTOR",
            "motor_pwm_protocol": "DSHOT300",
            "failsafe_procedure": "RTH",
            "mc_p_roll": 40, "mc_p_pitch": 44,
            "mc_d_roll": 25, "mc_d_pitch": 25,
            "roll_rate": 50, "pitch_rate": 50, "yaw_rate": 40,
            "gyro_main_lpf_hz": 110, "dterm_lpf_hz": 110,
            "looptime": 500,
        }
        settings.update(overrides)
        for k, v in settings.items():
            if isinstance(v, bool):
                lines.append(f"set {k} = {'ON' if v else 'OFF'}")
            else:
                lines.append(f"set {k} = {v}")
        # GPS UART by default
        lines.append("serial 1 2 115200 57600 0 115200")
        # RX UART
        lines.append("serial 0 64 115200 57600 0 115200")
        # ARM mode
        lines.append("aux 0 0 1 1800 2100")
        # ANGLE mode
        lines.append("aux 1 1 2 1800 2100")
        # RTH mode
        lines.append("aux 2 11 3 1800 2100")
        return "\n".join(lines)

    def test_good_config_passes(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff()
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        fails = [i for i in items if i.status == SanityItem.FAIL]
        assert len(fails) == 0, f"Unexpected fails: {fails}"

    def test_no_arm_switch(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff()
        # Remove aux lines (no ARM)
        diff = "\n".join(l for l in diff.splitlines() if not l.startswith("aux"))
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        arm_fails = [i for i in items if i.category == "Arming" and i.status == SanityItem.FAIL]
        assert len(arm_fails) >= 1

    def test_drop_failsafe_fails(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(failsafe_procedure="DROP")
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        fs_fails = [i for i in items if i.category == "Failsafe" and i.status == SanityItem.FAIL]
        assert len(fs_fails) >= 1

    def test_inverted_battery_fails(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(bat_voltage_cell_min=450, bat_voltage_cell_max=420)
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        batt_fails = [i for i in items if i.category == "Battery" and i.status == SanityItem.FAIL]
        assert len(batt_fails) >= 1

    def test_motor_inverted_asks(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(motor_direction_inverted=True)
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        motor_asks = [i for i in items if i.category == "Motors" and i.status == SanityItem.ASK]
        assert len(motor_asks) >= 1

    def test_zero_p_term_fails(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(mc_p_roll=0)
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        pid_fails = [i for i in items if i.category == "PIDs" and i.status == SanityItem.FAIL]
        assert len(pid_fails) >= 1

    def test_extreme_pids_fails(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(mc_p_roll=150, mc_p_pitch=150)
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        pid_fails = [i for i in items if i.category == "PIDs" and i.status == SanityItem.FAIL]
        assert len(pid_fails) >= 1

    def test_nav_without_gps_fails(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff()
        # Remove GPS UART
        diff = "\n".join(l for l in diff.splitlines()
                         if not (l.startswith("serial 1") and "2 115200" in l))
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, interactive=False)
        nav_fails = [i for i in items if i.category == "Navigation" and i.status == SanityItem.FAIL]
        assert len(nav_fails) >= 1

    def test_pid_frame_mismatch_asks(self):
        from inav_toolkit.param_analyzer import parse_diff_all, run_sanity_check, SanityItem
        diff = self._make_diff(mc_p_roll=44, mc_p_pitch=46)
        parsed = parse_diff_all(diff)
        items = run_sanity_check(parsed, frame_inches=10, interactive=False)
        pid_asks = [i for i in items if i.category == "PIDs" and i.status == SanityItem.ASK]
        assert len(pid_asks) >= 1

    def test_json_output(self):
        out, err, rc = run_module(
            "inav_toolkit.param_analyzer", "--check", "--no-interactive",
            "--json", os.path.join(TESTS_DIR, "test_basic_diff.txt"))
        data = json.loads(out)
        assert isinstance(data, list)
        assert len(data) > 0
        assert all("status" in i and "category" in i for i in data)

    def test_check_exit_code_clean(self):
        """Good config returns exit code 0."""
        import tempfile
        diff = self._make_diff()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False) as f:
            f.write(diff)
            f.flush()
            out, err, rc = run_module(
                "inav_toolkit.param_analyzer", "--check", "--no-interactive", f.name)
        os.unlink(f.name)
        assert rc == 0, f"Expected rc=0, got {rc}\n{out}"

    def test_check_exit_code_bad(self):
        """Bad config returns exit code 1."""
        import tempfile
        diff = self._make_diff(failsafe_procedure="DROP", mc_p_roll=0)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False) as f:
            f.write(diff)
            f.flush()
            out, err, rc = run_module(
                "inav_toolkit.param_analyzer", "--check", "--no-interactive", f.name)
        os.unlink(f.name)
        assert rc == 1, f"Expected rc=1, got {rc}\n{out}"


class TestComparison:
    """Test comparative flight analysis."""

    def test_analyze_for_compare(self):
        """Test the comparison analysis pipeline with synthetic data."""
        from inav_toolkit.blackbox_analyzer import (
            _analyze_for_compare, generate_action_plan, analyze_noise,
            analyze_pid_response, analyze_motors, analyze_dterm_noise,
            detect_hover_oscillation, fingerprint_noise,
        )
        fixture = os.path.join(FIXTURES_DIR, "clean_hover.csv")
        if not os.path.exists(fixture):
            pytest.skip("clean_hover.csv fixture not found (run generate_fixtures.py)")

        # Create a minimal args namespace
        class Args:
            frame = 5
            props = None
            blades = 3
            cells = None
            kv = None
        args = Args()

        result = _analyze_for_compare(fixture, args)
        assert "plan" in result
        assert "scores" in result["plan"]
        assert "noise_results" in result
        assert "pid_results" in result
        assert "data" in result

    def test_comparison_noise_chart(self):
        """Test comparison noise overlay chart generation."""
        from inav_toolkit.blackbox_analyzer import (
            _create_comparison_noise_chart, analyze_noise
        )
        # Create minimal noise results
        n = 1000
        sr = 500.0
        freqs = np.fft.rfftfreq(256, 1.0 / sr)
        psd_db = -40 + np.random.randn(len(freqs)) * 5

        nr_a = [{"axis": ax, "freqs": freqs, "psd_db": psd_db,
                  "peaks": [{"freq_hz": 150, "power_db": -20}]}
                 for ax in ["Roll", "Pitch", "Yaw"]]
        nr_b = [{"axis": ax, "freqs": freqs, "psd_db": psd_db - 3,
                  "peaks": [{"freq_hz": 150, "power_db": -23}]}
                 for ax in ["Roll", "Pitch", "Yaw"]]

        chart = _create_comparison_noise_chart(nr_a, nr_b, "Flight A", "Flight B")
        assert isinstance(chart, str)
        assert len(chart) > 100  # valid base64

    def test_comparison_html(self):
        """Test comparison HTML generation structure."""
        from inav_toolkit.blackbox_analyzer import _generate_comparison_html

        def _make_res(score, logfile="test.bbl"):
            return {
                "plan": {
                    "scores": {"overall": score, "noise": score + 5, "pid": score - 5, "motor": 80},
                    "verdict_text": "Test verdict",
                    "actions": [],
                },
                "config": {"craft_name": "TEST", "roll_p": 40, "gyro_lowpass_hz": 110},
                "data": {"time_s": np.array([0, 60])},
                "noise_results": [None, None, None],
                "pid_results": [None, None, None],
                "logfile": logfile,
            }

        html = _generate_comparison_html(
            _make_res(60, "A.bbl"), _make_res(75, "B.bbl"),
            {}, "A", "B")
        assert "INAV Flight Comparison" in html
        assert "60" in html
        assert "75" in html


class TestReplay:
    """Test interactive replay HTML generation."""

    def test_downsample(self):
        from inav_toolkit.blackbox_analyzer import _downsample
        arr = np.arange(10000)
        ds = _downsample(arr, 1000)
        assert len(ds) <= 1100  # approximately 1000
        assert ds[0] == 0

    def test_downsample_short(self):
        from inav_toolkit.blackbox_analyzer import _downsample
        arr = np.arange(50)
        ds = _downsample(arr, 1000)
        assert len(ds) == 50  # no downsampling needed

    def test_replay_html_generation(self):
        """Test replay HTML output contains expected Plotly.js elements."""
        from inav_toolkit.blackbox_analyzer import _generate_replay_html

        n = 2000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n) * 10,
            "gyro_pitch": np.random.randn(n) * 10,
            "gyro_yaw": np.random.randn(n) * 5,
            "setpoint_roll": np.random.randn(n) * 10,
            "setpoint_pitch": np.random.randn(n) * 10,
            "setpoint_yaw": np.random.randn(n) * 5,
            "motor0": np.random.uniform(1000, 2000, n),
            "motor1": np.random.uniform(1000, 2000, n),
            "motor2": np.random.uniform(1000, 2000, n),
            "motor3": np.random.uniform(1000, 2000, n),
            "throttle": np.random.uniform(1000, 1800, n),
            "n_rows": n, "sample_rate": sr,
            "_slow_frames": [],
        }
        config = {"craft_name": "TEST_QUAD", "firmware_revision": "INAV 9.0"}
        html = _generate_replay_html(config, data, sr)

        # Plotly.js instead of Chart.js
        assert "plotly" in html.lower()
        assert "TEST_QUAD" in html
        # Plotly div IDs
        assert "plotRoll" in html
        assert "plotMotors" in html
        # Synced x-axis across all panels
        assert "plotly_relayout" in html
        # Flight mode overlay bar
        assert "modeBar" in html
        # Spectrogram waterfall (gyro data present so spectrogram computed)
        assert "plotSpectro" in html
        # WebGL rendering
        assert "scattergl" in html

    def test_replay_with_spectrogram(self):
        """Test replay HTML includes noise spectrogram waterfall."""
        from inav_toolkit.blackbox_analyzer import _generate_replay_html, _compute_spectrogram

        n = 2000
        sr = 500.0
        gyro = np.random.randn(n) * 10
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": gyro,
            "n_rows": n, "sample_rate": sr,
            "_slow_frames": [],
        }

        html = _generate_replay_html({"craft_name": "T"}, data, sr)
        assert "plotSpectro" in html
        assert "heatmap" in html

        # Also test spectrogram computation directly
        times, freqs, power = _compute_spectrogram(gyro, sr, nperseg=256)
        assert len(times) > 0
        assert len(freqs) > 0
        assert len(power) == len(freqs)
        assert len(power[0]) == len(times)
        assert max(freqs) <= 500

    def test_replay_flight_modes(self):
        """Test flight mode extraction from slow frames."""
        from inav_toolkit.blackbox_analyzer import _extract_flight_modes

        sr = 500.0
        n = 5000
        data = {
            "time_s": np.arange(n) / sr,
            "n_rows": n, "sample_rate": sr,
            "_slow_frames": [
                (0, {"flightModeFlags": 1}),      # ARM only
                (1000, {"flightModeFlags": 3}),    # ARM + ANGLE
                (3000, {"flightModeFlags": 259}),  # ARM + ANGLE + NAV POSHOLD (bit 8)
            ],
        }
        modes = _extract_flight_modes(data, sr)
        assert len(modes) == 3
        assert "ARM" in modes[0]["label"]
        assert "ANGLE" in modes[1]["label"]
        assert "NAV POSHOLD" in modes[2]["label"]



class TestLogQuality:
    """Tests for log quality scorer."""

    def test_good_log(self):
        """Test that a well-formed log gets GOOD grade."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 5000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n) * 50,
            "gyro_pitch": np.random.randn(n) * 50,
            "gyro_yaw": np.random.randn(n) * 20,
            "setpoint_roll": np.random.randn(n) * 30,
            "motor0": np.random.uniform(1000, 2000, n),
            "motor1": np.random.uniform(1000, 2000, n),
            "throttle": np.random.uniform(1100, 1800, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll", "gyro_pitch", "gyro_yaw",
                              "setpoint_roll", "motor0", "motor1", "throttle"],
        }
        q = assess_log_quality(data)
        assert q["usable"] is True
        assert q["grade"] == "GOOD"
        assert q["stats"]["has_gyro"] is True

    def test_too_short(self):
        """Test that a very short log is UNUSABLE."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 100
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll"],
        }
        q = assess_log_quality(data)
        assert q["grade"] == "UNUSABLE"
        assert q["usable"] is False
        assert any("short" in i["message"] or "only" in i["message"].lower() for i in q["issues"])

    def test_no_gyro(self):
        """Test that missing gyro data is UNUSABLE."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 5000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "motor0": np.random.uniform(1000, 2000, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["motor0"],
        }
        q = assess_log_quality(data)
        assert q["usable"] is False
        assert any("gyro" in i["message"].lower() for i in q["issues"])

    def test_low_sample_rate(self):
        """Test that low sample rate is flagged."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 500
        sr = 50.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n) * 50,
            "setpoint_roll": np.random.randn(n) * 30,
            "motor0": np.random.uniform(1000, 2000, n),
            "throttle": np.random.uniform(1100, 1800, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll", "setpoint_roll", "motor0", "throttle"],
        }
        q = assess_log_quality(data)
        assert any("sample rate" in i["message"].lower() for i in q["issues"])

    def test_ground_only_detection(self):
        """Test detection of no-flight (ground-only) logs."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 5000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n) * 50,
            "setpoint_roll": np.zeros(n),      # no stick movement
            "setpoint_pitch": np.zeros(n),
            "throttle": np.ones(n) * 1000,     # throttle at minimum
            "motor0": np.random.uniform(1000, 2000, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll", "setpoint_roll", "setpoint_pitch",
                              "throttle", "motor0"],
        }
        q = assess_log_quality(data)
        assert any("stick" in i["message"].lower() or "ground" in i["message"].lower()
                    for i in q["issues"])

    def test_corrupt_frames(self):
        """Test corrupt frame detection from decoder stats."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 5000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.random.randn(n) * 50,
            "setpoint_roll": np.random.randn(n) * 30,
            "motor0": np.random.uniform(1000, 2000, n),
            "throttle": np.random.uniform(1100, 1800, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll", "setpoint_roll", "motor0", "throttle"],
            "_decoder_stats": {"i_frames": 100, "p_frames": 200, "errors": 150},
        }
        q = assess_log_quality(data)
        assert any("corrupt" in i["message"].lower() for i in q["issues"])

    def test_all_zeros_gyro(self):
        """Test dead sensor detection."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality

        n = 5000
        sr = 500.0
        data = {
            "time_s": np.arange(n) / sr,
            "gyro_roll": np.zeros(n),
            "gyro_pitch": np.random.randn(n) * 50,
            "setpoint_roll": np.random.randn(n) * 30,
            "motor0": np.random.uniform(1000, 2000, n),
            "throttle": np.random.uniform(1100, 1800, n),
            "n_rows": n, "sample_rate": sr,
            "found_columns": ["gyro_roll", "gyro_pitch", "setpoint_roll",
                              "motor0", "throttle"],
        }
        q = assess_log_quality(data)
        assert any("zeros" in i["message"].lower() and "roll" in i["message"].lower()
                    for i in q["issues"])


class TestMarkdownReport:
    """Tests for Markdown report generation."""

    def test_basic_report(self):
        """Test markdown report contains expected sections."""
        from inav_toolkit.blackbox_analyzer import generate_markdown_report, get_frame_profile

        profile = get_frame_profile(5, 5, 3)
        config = {"craft_name": "TestQuad", "firmware_revision": "INAV 9.0.0"}
        data = {
            "time_s": np.arange(5000) / 500.0,
            "sample_rate": 500.0,
        }
        noise_results = [None, None, None]
        pid_results = [
            {"tracking_delay_ms": 3.5, "avg_overshoot_pct": 12.0},
            {"tracking_delay_ms": 4.0, "avg_overshoot_pct": 8.0},
            {"tracking_delay_ms": None, "avg_overshoot_pct": None},
        ]
        motor_analysis = None
        plan = {
            "scores": {"overall": 72, "noise": 85, "pid": 60, "motor": 90},
            "verdict": "NEEDS_WORK",
            "verdict_text": "Room for improvement",
            "findings": [
                {"severity": "warning", "message": "Roll delay slightly high"},
            ],
            "noise_fingerprint": [],
            "actions": [
                {"action": "Lower P gain", "param": "mc_p_pitch", "current": "44",
                 "new": "38", "reason": "Reduce overshoot"},
            ],
        }

        md = generate_markdown_report(plan, config, data, noise_results,
                                      pid_results, motor_analysis, profile)
        assert "TestQuad" in md
        assert "72/100" in md
        assert "mc_p_pitch" in md
        assert "Recommended Changes" in md
        assert "set mc_p_pitch = 38" in md
        assert "INAV Toolkit" in md

    def test_report_with_quality(self):
        """Test markdown report includes quality info when provided."""
        from inav_toolkit.blackbox_analyzer import generate_markdown_report, get_frame_profile

        profile = get_frame_profile(5, 5, 3)
        config = {"craft_name": "T", "firmware_revision": "INAV 9"}
        data = {"time_s": np.arange(1000) / 500.0, "sample_rate": 500.0}
        plan = {"scores": {"overall": 50}, "verdict": "OK", "verdict_text": "OK",
                "findings": [], "noise_fingerprint": [], "actions": []}
        quality = {
            "grade": "MARGINAL",
            "issues": [{"severity": "WARN", "message": "Short log"}],
        }

        md = generate_markdown_report(plan, config, data, [None]*3, [None]*3,
                                      None, profile, quality)
        assert "MARGINAL" in md
        assert "Short log" in md

    def test_report_with_deferred_actions(self):
        """Test markdown report shows deferred actions separately."""
        from inav_toolkit.blackbox_analyzer import generate_markdown_report, get_frame_profile

        profile = get_frame_profile(5, 5, 3)
        config = {"craft_name": "T", "firmware_revision": "INAV 9"}
        data = {"time_s": np.arange(5000) / 500.0, "sample_rate": 500.0}
        plan = {
            "scores": {"overall": 80}, "verdict": "GOOD", "verdict_text": "Good tune",
            "findings": [], "noise_fingerprint": [],
            "actions": [
                {"action": "Lower D", "param": "mc_d_roll", "current": "30",
                 "new": "25", "reason": "D-term noise"},
                {"action": "Enable RPM filter", "param": "rpm_filter_enabled",
                 "current": "OFF", "new": "ON", "reason": "Better filtering",
                 "deferred": True},
            ],
        }

        md = generate_markdown_report(plan, config, data, [None]*3, [None]*3,
                                      None, profile)
        assert "mc_d_roll" in md
        assert "Deferred" in md
        assert "rpm_filter_enabled" in md


class TestI18n:
    """Tests for localization system."""

    def test_english_default(self):
        """Test that English is the default locale."""
        from inav_toolkit.i18n import t, set_locale, get_locale
        set_locale("en")
        assert get_locale() == "en"
        assert t("verdict.dialed_in") != "verdict.dialed_in"  # not raw key
        assert "fly" in t("verdict.dialed_in").lower()

    def test_portuguese_translation(self):
        """Test pt_BR translations load and work."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("pt_BR")
        result = t("verdict.dialed_in")
        assert result != "verdict.dialed_in"
        assert "voar" in result.lower() or "perfeito" in result.lower()
        # Reset
        set_locale("en")

    def test_spanish_translation(self):
        """Test es translations load and work."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("es")
        result = t("verdict.dialed_in")
        assert result != "verdict.dialed_in"
        assert "volar" in result.lower() or "perfecto" in result.lower()
        set_locale("en")

    def test_format_substitution(self):
        """Test that {placeholders} are substituted."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("en")
        result = t("quality.too_short", duration="1.2")
        assert "1.2" in result
        assert "{duration}" not in result

    def test_format_substitution_pt_br(self):
        """Test substitution works in translated strings."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("pt_BR")
        result = t("quality.too_short", duration="3.5")
        assert "3.5" in result
        assert "{duration}" not in result
        set_locale("en")

    def test_missing_key_fallback(self):
        """Test that missing keys fall back to key itself."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("en")
        result = t("nonexistent.key.that.does.not.exist")
        assert result == "nonexistent.key.that.does.not.exist"

    def test_locale_fallback_to_english(self):
        """Test that unknown locale falls back to English."""
        from inav_toolkit.i18n import t, set_locale
        set_locale("xx_XX")  # nonexistent locale
        result = t("verdict.dialed_in")
        assert "fly" in result.lower()  # should get English
        set_locale("en")

    def test_available_locales(self):
        """Test that locale listing works."""
        from inav_toolkit.i18n import available_locales
        locales = available_locales()
        assert "en" in locales
        assert "pt_BR" in locales
        assert "es" in locales

    def test_quality_messages_translated(self):
        """Test that quality scorer messages use t() and translate."""
        from inav_toolkit.blackbox_analyzer import assess_log_quality
        from inav_toolkit.i18n import set_locale

        set_locale("pt_BR")
        n = 50
        data = {
            "time_s": np.arange(n) / 500.0,
            "gyro_roll": np.random.randn(n),
            "n_rows": n, "sample_rate": 500.0,
            "found_columns": ["gyro_roll"],
        }
        q = assess_log_quality(data)
        # Should have Portuguese messages
        has_pt = any("apenas" in i["message"].lower() or "necessário" in i["message"].lower()
                     or "análise" in i["message"].lower()
                     for i in q["issues"])
        assert has_pt, f"Expected Portuguese messages, got: {[i['message'] for i in q['issues']]}"
        set_locale("en")

    def test_markdown_report_translated(self):
        """Test markdown report uses translated section headers."""
        from inav_toolkit.blackbox_analyzer import generate_markdown_report, get_frame_profile
        from inav_toolkit.i18n import set_locale

        set_locale("pt_BR")
        profile = get_frame_profile(5, 5, 3)
        config = {"craft_name": "TestQuad", "firmware_revision": "INAV 9"}
        data = {"time_s": np.arange(5000) / 500.0, "sample_rate": 500.0}
        plan = {
            "scores": {"overall": 72}, "verdict": "OK",
            "verdict_text": "Test", "findings": [],
            "noise_fingerprint": {}, "actions": [],
        }
        md = generate_markdown_report(plan, config, data, [None]*3, [None]*3,
                                      None, profile)
        assert "Pontuações" in md or "Pontuação" in md  # Portuguese "Scores"
        assert "Analisador" in md  # Portuguese "Analyzer"
        set_locale("en")


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
