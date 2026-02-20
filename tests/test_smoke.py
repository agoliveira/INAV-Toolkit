#!/usr/bin/env python3
"""
Basic smoke tests for the INAV Flight Analyzer Toolkit.

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
        print(f"  ✓ {name}")
    else:
        failed += 1
        print(f"  ✗ {name}")
        if detail:
            print(f"    {detail}")


def test_filter_math():
    """Verify filter phase lag calculations."""
    print("\n── Filter Math ──")
    sys.path.insert(0, PROJECT_DIR)
    from inav_blackbox_analyzer import _phase_shift, estimate_filter_phase_lag
    import numpy as np

    # PT1 at cutoff → -45°
    r = _phase_shift("PT1", 100, 100)
    test("PT1 at cutoff = -45°", abs(r - (-45.0)) < 0.1, f"got {r:.2f}")

    # PT2 at cutoff → -90°
    r = _phase_shift("PT2", 100, 100)
    test("PT2 at cutoff = -90°", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # PT3 at cutoff → -135°
    r = _phase_shift("PT3", 100, 100)
    test("PT3 at cutoff = -135°", abs(r - (-135.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD Q=0.5 at cutoff → -90°
    r = _phase_shift("BIQUAD", 100, 100, q=0.5)
    test("BIQUAD Q=0.5 at cutoff = -90°", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD high Q at cutoff → still -90° (at resonance)
    r = _phase_shift("BIQUAD", 100, 100, q=250)
    test("BIQUAD Q=250 at cutoff = -90°", abs(r - (-90.0)) < 0.1, f"got {r:.2f}")

    # BIQUAD far below cutoff → near 0°
    r = _phase_shift("BIQUAD", 10, 100, q=0.5)
    test("BIQUAD well below cutoff → near 0°", abs(r) < 15, f"got {r:.2f}")

    # estimate_filter_phase_lag returns dict
    lag = estimate_filter_phase_lag(65, 50, "PT1")
    test("estimate_filter_phase_lag returns degrees", "degrees" in lag)
    test("estimate_filter_phase_lag returns ms", "ms" in lag)
    test("65Hz PT1 at 50Hz signal → ~37.6°", abs(lag["degrees"] - (-37.6)) < 1, f"got {lag['degrees']:.1f}")

    # Zero/negative edge cases
    lag = estimate_filter_phase_lag(0, 50, "PT1")
    test("Zero cutoff → zero lag", lag["degrees"] == 0.0 and lag["ms"] == 0.0)


def test_param_analyzer_setup():
    """Test --setup mode generates valid output."""
    print("\n── Param Analyzer: Setup Mode ──")

    # 10-inch 6S
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
    print("\n── Param Analyzer: Diff Analysis ──")

    diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
    if not os.path.exists(diff_path):
        print("  ⊘ Skipped (no test_basic_diff.txt)")
        return

    out, err, rc = run([sys.executable, "inav_param_analyzer.py", diff_path])
    test("Diff analysis exits 0", rc == 0, f"rc={rc}, stderr={err[:200]}")
    test("Output contains SUMMARY", "SUMMARY" in out)
    test("Output contains FINDINGS or checks passed", "FINDINGS" in out or "checks passed" in out)


def test_vtol_configurator_non_vtol():
    """Test VTOL configurator gracefully handles non-VTOL config."""
    print("\n── VTOL Configurator: Non-VTOL ──")

    diff_path = os.path.join(TESTS_DIR, "test_basic_diff.txt")
    if not os.path.exists(diff_path):
        print("  ⊘ Skipped (no test_basic_diff.txt)")
        return

    out, err, rc = run([sys.executable, "inav_vtol_configurator.py", diff_path])
    test("Non-VTOL exits 0", rc == 0)
    # A config with platform_type=MULTIROTOR but no FW profile → "No airplane"
    test("Detects missing FW profile", "No VTOL" in out or "No airplane" in out or "not set" in out)


def test_vtol_configurator_vtol():
    """Test VTOL configurator against a VTOL diff."""
    print("\n── VTOL Configurator: VTOL Config ──")

    diff_path = os.path.join(TESTS_DIR, "test_vtol_diff.txt")
    if not os.path.exists(diff_path):
        print("  ⊘ Skipped (no test_vtol_diff.txt)")
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


def test_blackbox_import():
    """Test that blackbox analyzer imports without errors."""
    print("\n── Blackbox Analyzer: Import ──")
    try:
        sys.path.insert(0, PROJECT_DIR)
        import inav_blackbox_analyzer
        test("Module imports", True)
        test("Has REPORT_VERSION", hasattr(inav_blackbox_analyzer, "REPORT_VERSION"))
        test("Version is 2.9.0", inav_blackbox_analyzer.REPORT_VERSION == "2.9.0")
    except Exception as e:
        test("Module imports", False, str(e))


if __name__ == "__main__":
    print("═" * 60)
    print("  INAV Flight Analyzer Toolkit — Smoke Tests")
    print("═" * 60)

    test_filter_math()
    test_blackbox_import()
    test_param_analyzer_setup()
    test_param_analyzer_diff()
    test_vtol_configurator_non_vtol()
    test_vtol_configurator_vtol()

    print(f"\n{'═' * 60}")
    total = passed + failed
    if failed == 0:
        print(f"  ALL {total} TESTS PASSED ✓")
    else:
        print(f"  {passed}/{total} passed, {failed} FAILED")
    print(f"{'═' * 60}")
    sys.exit(1 if failed else 0)
