#!/usr/bin/env python3
"""
Smoke tests for the INAV Flight Analyzer Toolkit.

Run: python3 tests/test_smoke.py
"""
import json
import os
import subprocess
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TESTS_DIR = SCRIPT_DIR

# Make project importable
sys.path.insert(0, PROJECT_DIR)

passed = 0
failed = 0


def run(cmd, expect_rc=0):
    """Run a command, return (stdout, stderr, rc)."""
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=PROJECT_DIR)
    return result.stdout, result.stderr, result.returncode


def test(name, condition, detail=""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  + {name}")
    else:
        failed += 1
        print(f"  X {name}")
        if detail:
            print(f"    {detail}")


# ═══════════════════════════════════════════════════════════════
#  Helper: build synthetic flight data
# ═══════════════════════════════════════════════════════════════

def make_synthetic_data(sr=1000, duration_s=20, **overrides):
    """Build a minimal data dict mimicking blackbox decoder output.

    Returns data with clean nav signals by default.
    Use overrides to inject specific arrays or scalars.
    """
    import numpy as np

    n = int(sr * duration_s)
    t = np.linspace(0, duration_s, n)

    # Heading: steady 90 deg, compass updates at ~75Hz with 0.1deg steps
    # Real compass holds value between updates, then steps
    # 0.01 deg noise -> ~0.7 deg/s jitter (clean)
    heading = np.full(n, 900.0) + np.random.normal(0, 0.1, n)  # 0.01 deg noise in decideg

    # GPS position: stationary at origin with small noise (cm)
    pos_n = np.random.normal(0, 10, n)
    pos_e = np.random.normal(0, 10, n)
    # Altitude: smooth climb/descent pattern so estimator has clear signal
    pos_u = 500 * np.sin(2 * np.pi * 0.1 * t)  # 5m gentle oscillation

    # Baro: tracks nav_pos_u with small noise (cm)
    baro = pos_u + np.random.normal(0, 10, n)

    # Throttle: mid-throttle with small variation
    throttle = np.full(n, 1500.0) + np.random.normal(0, 20, n)

    # Nav targets: at origin
    tgt_n = np.zeros(n)
    tgt_e = np.zeros(n)
    tgt_u = np.zeros(n)

    # Velocity: near zero
    vel_n = np.random.normal(0, 5, n)
    vel_e = np.random.normal(0, 5, n)
    vel_u = np.random.normal(0, 5, n)

    # EPH: good GPS
    eph = np.full(n, 50.0) + np.random.normal(0, 5, n)

    # navState: IDLE (0) the whole time by default
    nav_state = np.zeros(n, dtype=float)

    # GPS frames: 10Hz GPS updates with 15 sats
    gps_frames = []
    gps_interval = int(sr / 10)
    for i in range(0, n, gps_interval):
        gps_frames.append((i, {"GPS_numSat": 15}))

    # Slow frames
    slow_frames = []

    # Gyro and setpoint for hover oscillation detector
    gyro_roll = np.random.normal(0, 1.0, n)
    gyro_pitch = np.random.normal(0, 1.0, n)
    gyro_yaw = np.random.normal(0, 0.5, n)
    sp_roll = np.zeros(n)
    sp_pitch = np.zeros(n)
    sp_yaw = np.zeros(n)

    data = {
        "time_s": t,
        "sample_rate": sr,
        "n_rows": n,
        "att_heading": heading,
        "nav_pos_n": pos_n,
        "nav_pos_e": pos_e,
        "nav_pos_u": pos_u,
        "nav_tgt_n": tgt_n,
        "nav_tgt_e": tgt_e,
        "nav_tgt_u": tgt_u,
        "nav_vel_n": vel_n,
        "nav_vel_e": vel_e,
        "nav_vel_u": vel_u,
        "nav_eph": eph,
        "nav_state": nav_state,
        "baro_alt": baro,
        "throttle": throttle,
        "motor0": throttle,
        "gyro_roll": gyro_roll,
        "gyro_pitch": gyro_pitch,
        "gyro_yaw": gyro_yaw,
        "setpoint_roll": sp_roll,
        "setpoint_pitch": sp_pitch,
        "setpoint_yaw": sp_yaw,
        "_gps_frames": gps_frames,
        "_slow_frames": slow_frames,
        "_has_nav": True,
        "_nav_fields": ["nav_pos_n", "nav_pos_e", "nav_pos_u"],
    }

    data.update(overrides)
    return data


# ═══════════════════════════════════════════════════════════════
#  Blackbox Analyzer: Import & Version
# ═══════════════════════════════════════════════════════════════

def test_blackbox_import():
    print("\n-- Blackbox Analyzer: Import --")
    try:
        import inav_blackbox_analyzer as bb
        test("Module imports", True)
        test("Has REPORT_VERSION", hasattr(bb, "REPORT_VERSION"))
        # Don't hardcode version - just check format
        ver = bb.REPORT_VERSION
        parts = ver.split(".")
        test(f"Version is semver ({ver})", len(parts) == 3 and all(p.isdigit() for p in parts))
    except Exception as e:
        test("Module imports", False, str(e))


# ═══════════════════════════════════════════════════════════════
#  Filter Math
# ═══════════════════════════════════════════════════════════════

def test_filter_math():
    print("\n-- Filter Math --")
    from inav_blackbox_analyzer import _phase_shift, estimate_filter_phase_lag

    # PT1 at cutoff = -45 deg
    r = _phase_shift("PT1", 100, 100)
    test("PT1 at cutoff = -45 deg", abs(r - (-45.0)) < 0.1, f"got {r:.2f}")

    # PT2 at cutoff = -90 deg
    r = _phase_shift("PT2", 100, 100)
    test("PT2 at cutoff = -90 deg", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # PT3 at cutoff = -135 deg
    r = _phase_shift("PT3", 100, 100)
    test("PT3 at cutoff = -135 deg", abs(r - (-135.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD Q=0.5 at cutoff = -90 deg
    r = _phase_shift("BIQUAD", 100, 100, q=0.5)
    test("BIQUAD Q=0.5 at cutoff = -90 deg", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD high Q at cutoff = -90 deg
    r = _phase_shift("BIQUAD", 100, 100, q=250)
    test("BIQUAD Q=250 at cutoff = -90 deg", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD far below cutoff: near 0 deg
    r = _phase_shift("BIQUAD", 10, 100, q=0.5)
    test("BIQUAD well below cutoff -> near 0 deg", abs(r) < 15, f"got {r:.2f}")

    # estimate_filter_phase_lag returns dict
    lag = estimate_filter_phase_lag(65, 50, "PT1")
    test("estimate_filter_phase_lag returns degrees", "degrees" in lag)
    test("estimate_filter_phase_lag returns ms", "ms" in lag)
    test("65Hz PT1 at 50Hz -> ~37.6 deg", abs(lag["degrees"] - (-37.6)) < 1, f"got {lag['degrees']:.1f}")

    # Zero cutoff edge case
    lag = estimate_filter_phase_lag(0, 50, "PT1")
    test("Zero cutoff -> zero lag", lag["degrees"] == 0.0 and lag["ms"] == 0.0)


# ═══════════════════════════════════════════════════════════════
#  Nav: Compass Health
# ═══════════════════════════════════════════════════════════════

def test_compass_clean():
    """Clean compass: low jitter, no EMI, no drift."""
    print("\n-- Nav: Compass (clean) --")
    from inav_blackbox_analyzer import analyze_compass_health

    data = make_synthetic_data()
    r = analyze_compass_health(data, 1000)

    test("Returns score", r["score"] is not None)
    test("Score >= 80 (clean heading)", r["score"] >= 80, f"got {r['score']}")
    test("Jitter < 3 deg/s", r["heading_jitter_deg"] is not None and r["heading_jitter_deg"] < 3,
         f"got {r['heading_jitter_deg']}")


def test_compass_emi():
    """Compass with high jitter (EMI or bad mounting)."""
    print("\n-- Nav: Compass (noisy) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_compass_health

    n = 20000
    # Noisy heading: 0.5 deg noise -> ~35 deg/s jitter at 50Hz
    heading = np.full(n, 900.0) + np.random.normal(0, 5, n)  # 0.5 deg noise

    data = make_synthetic_data(att_heading=heading)
    r = analyze_compass_health(data, 1000)

    test("Returns score", r["score"] is not None)
    test("High jitter (> 5 deg/s)", r["heading_jitter_deg"] is not None and r["heading_jitter_deg"] > 5,
         f"got {r['heading_jitter_deg']}")
    test("Score < 80", r["score"] is not None and r["score"] < 80, f"got {r['score']}")


def test_compass_missing():
    """No heading data - returns None score."""
    print("\n-- Nav: Compass (no data) --")
    from inav_blackbox_analyzer import analyze_compass_health

    data = make_synthetic_data()
    del data["att_heading"]
    r = analyze_compass_health(data, 1000)
    test("Score is None", r["score"] is None)


# ═══════════════════════════════════════════════════════════════
#  Nav: GPS Quality
# ═══════════════════════════════════════════════════════════════

def test_gps_clean():
    """Clean GPS: good EPH, many sats, no jumps."""
    print("\n-- Nav: GPS (clean) --")
    from inav_blackbox_analyzer import analyze_gps_quality

    data = make_synthetic_data()
    r = analyze_gps_quality(data, 1000)

    test("Returns score", r["score"] is not None)
    test("Score >= 90 (good GPS)", r["score"] >= 90, f"got {r['score']}")
    test("Avg sats reported", r["avg_sats"] is not None and r["avg_sats"] >= 10,
         f"got {r['avg_sats']}")
    test("Zero position jumps", r["position_jumps"] == 0, f"got {r['position_jumps']}")


def test_gps_poor():
    """Poor GPS: high EPH, low sats."""
    print("\n-- Nav: GPS (poor) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_gps_quality

    n = 20000
    eph = np.full(n, 600.0) + np.random.normal(0, 50, n)
    gps_frames = [(i, {"GPS_numSat": 4}) for i in range(0, n, 100)]

    data = make_synthetic_data(nav_eph=eph, _gps_frames=gps_frames)
    r = analyze_gps_quality(data, 1000)

    test("Score < 60 (poor GPS)", r["score"] is not None and r["score"] < 60,
         f"got {r['score']}")
    test("EPH > 500", r["avg_eph"] is not None and r["avg_eph"] > 500,
         f"got {r['avg_eph']}")
    test("Min sats < 6", r["min_sats"] is not None and r["min_sats"] < 6,
         f"got {r['min_sats']}")
    test("Has warnings", any(s == "WARNING" for s, _ in r["findings"]))


# ═══════════════════════════════════════════════════════════════
#  Nav: Baro Quality
# ═══════════════════════════════════════════════════════════════

def test_baro_clean():
    """Clean baro: low noise."""
    print("\n-- Nav: Baro (clean) --")
    from inav_blackbox_analyzer import analyze_baro_quality

    data = make_synthetic_data()
    r = analyze_baro_quality(data, 1000)

    test("Returns score", r["score"] is not None)
    test("Score >= 80 (clean baro)", r["score"] >= 80, f"got {r['score']}")
    test("Noise < 50cm", r["noise_cm"] is not None and r["noise_cm"] < 50,
         f"got {r['noise_cm']}")


def test_baro_noisy():
    """Noisy baro: high RMS, spikes."""
    print("\n-- Nav: Baro (noisy) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_baro_quality

    n = 20000
    baro = np.random.normal(0, 200, n)
    # Add spikes
    spike_idx = np.random.choice(n, 40, replace=False)
    baro[spike_idx] += np.random.choice([-1500, 1500], 40)

    data = make_synthetic_data(baro_alt=baro)
    r = analyze_baro_quality(data, 1000)

    test("Score < 60 (noisy baro)", r["score"] is not None and r["score"] < 60,
         f"got {r['score']}")
    test("Noise > 100cm", r["noise_cm"] is not None and r["noise_cm"] > 100,
         f"got {r['noise_cm']}")
    test("Spikes detected", r["spikes"] > 0, f"got {r['spikes']}")


# ═══════════════════════════════════════════════════════════════
#  Nav: Estimator Health
# ═══════════════════════════════════════════════════════════════

def test_estimator_healthy():
    """Estimator tracks baro well."""
    print("\n-- Nav: Estimator (healthy) --")
    from inav_blackbox_analyzer import analyze_estimator_health

    data = make_synthetic_data()
    r = analyze_estimator_health(data, 1000)

    test("Returns score", r["score"] is not None)
    test("Score >= 80", r["score"] >= 80, f"got {r['score']}")
    test("Correlation > 0.8", r["baro_vs_nav_corr"] is not None and r["baro_vs_nav_corr"] > 0.8,
         f"got {r['baro_vs_nav_corr']}")


def test_estimator_diverged():
    """Estimator diverges from baro."""
    print("\n-- Nav: Estimator (diverged) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_estimator_health

    n = 20000
    nav_z = np.linspace(0, 5000, n)  # climbing steadily
    baro = np.linspace(0, 500, n)    # baro says barely moving

    data = make_synthetic_data(nav_pos_u=nav_z, baro_alt=baro)
    r = analyze_estimator_health(data, 1000)

    test("Score < 80 (divergence)", r["score"] is not None and r["score"] < 80,
         f"got {r['score']}")
    test("Has divergence finding", len(r["findings"]) > 0)


# ═══════════════════════════════════════════════════════════════
#  Nav: Flight Phase Segmentation
# ═══════════════════════════════════════════════════════════════

def test_phase_segmentation():
    """Detect nav phases from navState."""
    print("\n-- Nav: Phase Segmentation --")
    import numpy as np
    from inav_blackbox_analyzer import segment_flight_phases

    sr = 1000
    n = 30000  # 30s
    ns = np.zeros(n, dtype=float)
    # 0-5s: idle (0), 5-15s: althold (5), 15-25s: poshold (15), 25-30s: idle
    ns[5000:15000] = 5.0
    ns[15000:25000] = 15.0

    data = {"nav_state": ns}
    phases = segment_flight_phases(data, sr)

    test("Found 2 phases", len(phases) == 2, f"got {len(phases)}")
    if len(phases) >= 2:
        test("Phase 1 is navState 5", phases[0][2] == 5)
        test("Phase 2 is navState 15", phases[1][2] == 15)
        test("Phase 1 ~10s long", abs((phases[0][1] - phases[0][0]) / sr - 10) < 0.5)


def test_phase_gating_manual():
    """Althold/poshold analysis skipped when navState=0 (manual flight)."""
    print("\n-- Nav: Phase Gating (manual flight) --")
    from inav_blackbox_analyzer import run_nav_analysis

    data = make_synthetic_data()  # navState=0 throughout
    r = run_nav_analysis(data, 1000)

    test("No althold result", r.get("althold") is None or r["althold"]["score"] is None)
    test("No poshold result", r.get("poshold") is None or r["poshold"]["score"] is None)
    test("Compass still scored", r.get("compass") is not None and r["compass"]["score"] is not None)
    test("GPS still scored", r.get("gps") is not None and r["gps"]["score"] is not None)


# ═══════════════════════════════════════════════════════════════
#  Nav: Altitude Hold (with phase)
# ═══════════════════════════════════════════════════════════════

def test_althold_stable():
    """Stable altitude hold: tight tracking."""
    print("\n-- Nav: Altitude Hold (stable) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_altitude_hold

    n = 10000
    nav_z = np.full(n, 1000.0) + np.random.normal(0, 10, n)
    tgt_z = np.full(n, 1000.0)
    vel_z = np.random.normal(0, 5, n)

    data = make_synthetic_data(nav_pos_u=nav_z, nav_tgt_u=tgt_z, nav_vel_u=vel_z)
    r = analyze_altitude_hold(data, 1000, 0, n)

    test("Score >= 80", r["score"] is not None and r["score"] >= 80, f"got {r['score']}")
    test("Oscillation < 100cm", r["oscillation_cm"] is not None and r["oscillation_cm"] < 100,
         f"got {r['oscillation_cm']}")


def test_althold_oscillating():
    """Bad altitude hold: large oscillation."""
    print("\n-- Nav: Altitude Hold (oscillating) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_altitude_hold

    n = 10000
    t = np.linspace(0, 10, n)
    nav_z = 1000.0 + 300 * np.sin(2 * np.pi * 0.5 * t)  # 300cm oscillation
    tgt_z = np.full(n, 1000.0)

    data = make_synthetic_data(nav_pos_u=nav_z, nav_tgt_u=tgt_z)
    r = analyze_altitude_hold(data, 1000, 0, n)

    test("Score < 70", r["score"] is not None and r["score"] < 70, f"got {r['score']}")
    test("Oscillation > 200cm", r["oscillation_cm"] is not None and r["oscillation_cm"] > 200,
         f"got {r['oscillation_cm']}")
    test("Has warning", any(s == "WARNING" for s, _ in r["findings"]))


# ═══════════════════════════════════════════════════════════════
#  Nav: Position Hold
# ═══════════════════════════════════════════════════════════════

def test_poshold_stable():
    """Stable poshold: tight CEP."""
    print("\n-- Nav: Position Hold (stable) --")
    import numpy as np
    from inav_blackbox_analyzer import analyze_position_hold

    n = 10000
    pn = np.random.normal(0, 30, n)   # 30cm noise around target
    pe = np.random.normal(0, 30, n)
    tn = np.zeros(n)
    te = np.zeros(n)

    data = make_synthetic_data(nav_pos_n=pn, nav_pos_e=pe, nav_tgt_n=tn, nav_tgt_e=te)
    r = analyze_position_hold(data, 1000, 0, n)

    test("Score >= 80", r["score"] is not None and r["score"] >= 80, f"got {r['score']}")
    test("CEP < 100cm", r["cep_cm"] is not None and r["cep_cm"] < 100, f"got {r['cep_cm']}")
    test("No toilet bowl", r["toilet_bowl"] is False)


# ═══════════════════════════════════════════════════════════════
#  Nav: Integrated run_nav_analysis
# ═══════════════════════════════════════════════════════════════

def test_run_nav_analysis():
    """Full nav analysis pipeline with clean data."""
    print("\n-- Nav: Integrated Pipeline --")
    from inav_blackbox_analyzer import run_nav_analysis

    data = make_synthetic_data()
    r = run_nav_analysis(data, 1000)

    test("Returns dict", isinstance(r, dict))
    test("Has nav_score", "nav_score" in r)
    test("Nav score > 0", r["nav_score"] is not None and r["nav_score"] > 0,
         f"got {r.get('nav_score')}")
    test("Has compass result", "compass" in r and r["compass"]["score"] is not None)
    test("Has gps result", "gps" in r and r["gps"]["score"] is not None)
    test("Has baro result", "baro" in r and r["baro"]["score"] is not None)
    test("Has estimator result", "estimator" in r and r["estimator"]["score"] is not None)


def test_run_nav_no_nav_fields():
    """Nav analysis with no nav fields returns gracefully."""
    print("\n-- Nav: No Nav Fields --")
    from inav_blackbox_analyzer import run_nav_analysis

    data = {"time_s": [0, 1, 2], "sample_rate": 1000, "n_rows": 3}
    r = run_nav_analysis(data, 1000)

    test("Returns None or empty", r is None or r.get("nav_score") is None)


# ═══════════════════════════════════════════════════════════════
#  Nav: Hover Oscillation Detection
# ═══════════════════════════════════════════════════════════════

def test_hover_osc_clean():
    """Clean hover: no oscillation."""
    print("\n-- Hover Oscillation (clean) --")
    from inav_blackbox_analyzer import detect_hover_oscillation

    data = make_synthetic_data()
    results = detect_hover_oscillation(data, 1000)

    osc_axes = [r for r in results if r["severity"] != "none"]
    test("No oscillation on clean data", len(osc_axes) == 0,
         f"got {[(r['axis'], r['severity'], r['gyro_rms']) for r in osc_axes]}")


def test_hover_osc_severe():
    """Severe oscillation: high gyro RMS during hover."""
    print("\n-- Hover Oscillation (severe) --")
    import numpy as np
    from inav_blackbox_analyzer import detect_hover_oscillation

    n = 20000
    t = np.linspace(0, 20, n)
    # 4Hz oscillation at 50 deg/s amplitude
    gyro_roll = 50 * np.sin(2 * np.pi * 4 * t) + np.random.normal(0, 2, n)

    data = make_synthetic_data(gyro_roll=gyro_roll)
    results = detect_hover_oscillation(data, 1000)

    roll_results = [r for r in results if r["axis"] == "Roll"]
    test("Roll oscillation detected", len(roll_results) > 0)
    if roll_results:
        r = roll_results[0]
        test("Severity moderate or severe", r["severity"] in ("moderate", "severe"),
             f"got {r['severity']}")
        test("Dominant freq near 4Hz",
             r["dominant_freq_hz"] is not None and abs(r["dominant_freq_hz"] - 4) < 1,
             f"got {r['dominant_freq_hz']}")


# ═══════════════════════════════════════════════════════════════
#  Param Analyzer
# ═══════════════════════════════════════════════════════════════

def test_param_analyzer_setup():
    """Test --setup mode generates valid output."""
    print("\n-- Param Analyzer: Setup Mode --")

    out, err, rc = run([sys.executable, "inav_param_analyzer.py", "--setup", "10", "--voltage", "6S"])
    test("--setup 10 --voltage 6S exits 0", rc == 0, f"rc={rc}")
    test("Output contains 'set mc_p_roll'", "set mc_p_roll" in out.lower() or "mc_p_roll" in out)
    test("Output contains 'gyro' filter setting", "gyro" in out.lower())

    # JSON output
    out, err, rc = run([sys.executable, "inav_param_analyzer.py", "--setup", "10", "--json"])
    test("--setup --json exits 0", rc == 0)
    if rc == 0 and out.strip():
        try:
            data = json.loads(out)
            test("JSON output is valid", True)
        except json.JSONDecodeError as e:
            test("JSON output is valid", False, str(e))


def test_param_analyzer_diff():
    """Test analysis of a diff file."""
    print("\n-- Param Analyzer: Diff Analysis --")

    diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
    if not os.path.exists(diff_path):
        print("  ~ Skipped (no test_basic_diff.txt)")
        return

    out, err, rc = run([sys.executable, "inav_param_analyzer.py", diff_path])
    test("Diff analysis exits 0", rc == 0, f"rc={rc}, stderr={err[:200]}")
    test("Output contains SUMMARY", "SUMMARY" in out)
    test("Output contains FINDINGS or checks passed", "FINDINGS" in out or "checks passed" in out)


# ═══════════════════════════════════════════════════════════════
#  VTOL Configurator
# ═══════════════════════════════════════════════════════════════

def test_vtol_configurator_non_vtol():
    """Test VTOL configurator gracefully handles non-VTOL config."""
    print("\n-- VTOL Configurator: Non-VTOL --")

    diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
    if not os.path.exists(diff_path):
        print("  ~ Skipped (no test_basic_diff.txt)")
        return

    out, err, rc = run([sys.executable, "inav_vtol_configurator.py", diff_path])
    test("Non-VTOL exits 0", rc == 0)
    test("Detects missing FW profile", "No VTOL" in out or "No airplane" in out or "not set" in out)


def test_vtol_configurator_vtol():
    """Test VTOL configurator against a VTOL diff."""
    print("\n-- VTOL Configurator: VTOL Config --")

    diff_path = os.path.join(TESTS_DIR, "test_vtol_diff.txt")
    if not os.path.exists(diff_path):
        print("  ~ Skipped (no test_vtol_diff.txt)")
        return

    out, err, rc = run([sys.executable, "inav_vtol_configurator.py", diff_path])
    test("VTOL analysis exits 0", rc == 0, f"rc={rc}")
    test("Detects TRICOPTER", "TRICOPTER" in out)
    test("Detects AIRPLANE", "AIRPLANE" in out)
    test("Detects tilt motors", "Tilt motors" in out or "tilt" in out.lower())

    # JSON output
    out, err, rc = run([sys.executable, "inav_vtol_configurator.py", diff_path, "--json"])
    test("--json exits 0", rc == 0)
    if rc == 0 and out.strip():
        try:
            data = json.loads(out)
            test("JSON output is valid list", isinstance(data, list))
        except json.JSONDecodeError as e:
            test("JSON output is valid", False, str(e))


# ═══════════════════════════════════════════════════════════════
#  Run All
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  INAV Toolkit - Smoke Tests")
    print("=" * 60)

    # Core
    test_blackbox_import()
    test_filter_math()

    # Nav: individual analyzers
    test_compass_clean()
    test_compass_emi()
    test_compass_missing()
    test_gps_clean()
    test_gps_poor()
    test_baro_clean()
    test_baro_noisy()
    test_estimator_healthy()
    test_estimator_diverged()

    # Nav: phase segmentation & gating
    test_phase_segmentation()
    test_phase_gating_manual()

    # Nav: phase-specific analyzers
    test_althold_stable()
    test_althold_oscillating()
    test_poshold_stable()

    # Nav: integrated pipeline
    test_run_nav_analysis()
    test_run_nav_no_nav_fields()

    # Hover oscillation
    test_hover_osc_clean()
    test_hover_osc_severe()

    # Param analyzer
    test_param_analyzer_setup()
    test_param_analyzer_diff()

    # VTOL
    test_vtol_configurator_non_vtol()
    test_vtol_configurator_vtol()

    print(f"\n{'=' * 60}")
    total = passed + failed
    if failed == 0:
        print(f"  ALL {total} TESTS PASSED")
    else:
        print(f"  {passed}/{total} passed, {failed} FAILED")
    print(f"{'=' * 60}")
    sys.exit(1 if failed else 0)
