#!/usr/bin/env python3
"""
INAV Blackbox Analyzer - Multirotor Tuning Tool v2.4.1
=====================================================
Analyzes INAV blackbox logs and tells you EXACTLY what to change.

Output: specific "change X from A to B" instructions, not vague advice.
Supports iterative tuning: run → adjust → fly → run again → repeat.

Usage:
    inav-analyze <logfile.bbl> [options]
    inav-analyze flight.bbl --previous old_state.json  # compare iterations
"""

import sys
import os
import csv
import json
import re
import argparse
import warnings
from datetime import datetime
from io import BytesIO, StringIO
import base64

import numpy as np
from scipy import signal
from scipy.fft import rfft, rfftfreq

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", message=".*tight_layout.*")

# ─── Localization ──────────────────────────────────────────────────────────────
try:
    from inav_toolkit.i18n import t
except ImportError:
    try:
        from i18n import t
    except ImportError:
        # Fallback: no-op translation if i18n module is missing
        def t(key, **kwargs):
            text = key
            if kwargs:
                try:
                    text = text.format(**kwargs)
                except (KeyError, IndexError, ValueError):
                    pass
            return text

# ─── Constants ────────────────────────────────────────────────────────────────

def _enable_ansi_colors():
    """Enable ANSI color support. Returns True if colors are available.

    Respects NO_COLOR environment variable (https://no-color.org/).
    On Windows 10+, enables VT100 processing in the console.
    On Linux/macOS, ANSI is natively supported in all terminals.
    Returns False if stdout is not a terminal (piped/redirected).
    """
    if os.environ.get("NO_COLOR") is not None:
        return False

    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False

    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            # STD_OUTPUT_HANDLE = -11
            handle = kernel32.GetStdHandle(-11)
            # Get current mode
            mode = ctypes.c_ulong()
            kernel32.GetConsoleMode(handle, ctypes.byref(mode))
            # ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
            return True
        except Exception:
            return False

    return True  # Linux/macOS always support ANSI

_ANSI_ENABLED = _enable_ansi_colors()

def _colors():
    """Return (R, B, C, G, Y, RED, DIM) color codes.

    Returns ANSI codes when colors are enabled, empty strings otherwise.
    """
    if _ANSI_ENABLED:
        return ("\033[0m", "\033[1m", "\033[96m", "\033[92m",
                "\033[93m", "\033[91m", "\033[2m")
    return ("", "", "", "", "", "", "")

def _disable_colors():
    """Disable ANSI colors (called by --no-color)."""
    global _ANSI_ENABLED
    _ANSI_ENABLED = False

AXIS_NAMES = ["Roll", "Pitch", "Yaw"]
AXIS_COLORS = ["#FF6B6B", "#4ECDC4", "#FFD93D"]
MOTOR_COLORS = ["#FF6B6B", "#4ECDC4", "#FFD93D", "#A78BFA"]
REPORT_VERSION = "2.4.1"

# ─── Frame and Prop Profiles ─────────────────────────────────────────────────
# Two separate concerns:
#
# FRAME SIZE determines response characteristics:
#   - Moment of inertia → how fast the quad CAN rotate
#   - Arm resonance → where structural vibration lives
#   - PID thresholds → what's "good" overshoot/delay for this airframe
#   - Adjustment factor → how conservative each tuning step should be
#
# PROP CONFIGURATION determines noise characteristics:
#   - Diameter + blade count + RPM → where vibration harmonics land
#   - Larger/more blades at lower RPM → harmonics closer to signal band
#   - Filter ranges must match where noise actually lives
#
# A 5" frame with triblade 5" props has SAME response thresholds but
# DIFFERENT noise frequencies vs a 5" frame with bullnose 6" biblades.

FRAME_PROFILES = {
    # Keyed by frame size (inches). Drives PID response thresholds.
    3: {
        "name": "3-inch", "class": "micro",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 6, "ok_delay_ms": 12, "bad_delay_ms": 25,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.35,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Fast response, high RPM. Noise well separated from signal. Easy to tune.",
    },
    4: {
        "name": "4-inch", "class": "micro",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 7, "ok_delay_ms": 13, "bad_delay_ms": 28,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.32,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 3.5, "motor_imbal_warn": 9,
        "notes": "Similar to 5-inch but slightly faster motor response.",
    },
    5: {
        "name": "5-inch", "class": "standard",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 8, "ok_delay_ms": 15, "bad_delay_ms": 30,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.30,
        "motor_sat_ok": 2, "motor_sat_warn": 8,
        "motor_imbal_ok": 3, "motor_imbal_warn": 8,
        "notes": "The reference class. Most defaults and community tunes are calibrated for 5-inch.",
    },
    6: {
        "name": "6-inch", "class": "standard",
        "good_overshoot": 10, "ok_overshoot": 18, "bad_overshoot": 35,
        "good_delay_ms": 10, "ok_delay_ms": 18, "bad_delay_ms": 35,
        "good_noise_db": -42, "ok_noise_db": -32, "bad_noise_db": -22,
        "pid_adjust_factor": 0.28,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 3.5, "motor_imbal_warn": 9,
        "notes": "Transition zone. Still manageable but noise starts creeping into signal band.",
    },
    7: {
        "name": "7-inch", "class": "mid",
        "good_overshoot": 12, "ok_overshoot": 20, "bad_overshoot": 38,
        "good_delay_ms": 12, "ok_delay_ms": 22, "bad_delay_ms": 40,
        "good_noise_db": -40, "ok_noise_db": -30, "bad_noise_db": -20,
        "pid_adjust_factor": 0.25,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Long-range territory. Lower KV, noise-signal overlap begins. More filtering needed.",
    },
    8: {
        "name": "8-inch", "class": "mid",
        "good_overshoot": 14, "ok_overshoot": 22, "bad_overshoot": 40,
        "good_delay_ms": 15, "ok_delay_ms": 25, "bad_delay_ms": 45,
        "good_noise_db": -38, "ok_noise_db": -28, "bad_noise_db": -18,
        "pid_adjust_factor": 0.22,
        "motor_sat_ok": 4, "motor_sat_warn": 12,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Significant prop inertia. Motor response is noticeably slower. Conservative tuning essential.",
    },
    9: {
        "name": "9-inch", "class": "large",
        "good_overshoot": 15, "ok_overshoot": 25, "bad_overshoot": 42,
        "good_delay_ms": 18, "ok_delay_ms": 30, "bad_delay_ms": 50,
        "good_noise_db": -36, "ok_noise_db": -26, "bad_noise_db": -16,
        "pid_adjust_factor": 0.20,
        "motor_sat_ok": 4, "motor_sat_warn": 12,
        "motor_imbal_ok": 5, "motor_imbal_warn": 12,
        "notes": "Large quad territory. Signal and noise overlap significantly. Phase lag management critical.",
    },
    10: {
        "name": "10-inch", "class": "large",
        "good_overshoot": 18, "ok_overshoot": 28, "bad_overshoot": 45,
        "good_delay_ms": 20, "ok_delay_ms": 35, "bad_delay_ms": 55,
        "good_noise_db": -34, "ok_noise_db": -24, "bad_noise_db": -14,
        "pid_adjust_factor": 0.20,
        "motor_sat_ok": 5, "motor_sat_warn": 14,
        "motor_imbal_ok": 5, "motor_imbal_warn": 12,
        "notes": "Noise lives in signal band. Aggressive filtering adds dangerous phase lag. Every change must be small.",
    },
    12: {
        "name": "12-inch+", "class": "heavy",
        "good_overshoot": 20, "ok_overshoot": 32, "bad_overshoot": 50,
        "good_delay_ms": 25, "ok_delay_ms": 40, "bad_delay_ms": 65,
        "good_noise_db": -30, "ok_noise_db": -20, "bad_noise_db": -12,
        "pid_adjust_factor": 0.15,
        "motor_sat_ok": 6, "motor_sat_warn": 16,
        "motor_imbal_ok": 6, "motor_imbal_warn": 14,
        "notes": "Heavy lift / cinema. Extremely conservative tuning. Phase lag is the main enemy. "
                 "Consider reducing PID loop rate if not already at 500Hz or lower.",
    },
}

# Prop configuration drives noise prediction and filter ranges.
# Filter ranges depend on PROP diameter (not frame), because that's what
# determines where vibration energy lands in the spectrum.
# Blade count is handled separately in the harmonic prediction.
PROP_NOISE_PROFILES = {
    # prop_diameter_inches: { filter ranges, noise bands, safety margins }
    3:  {"gyro_lpf_range": (80, 350), "dterm_lpf_range": (60, 200), "filter_safety": 0.80,
         "noise_band_mid": (120, 400), "noise_band_high": 400},
    4:  {"gyro_lpf_range": (70, 320), "dterm_lpf_range": (50, 180), "filter_safety": 0.80,
         "noise_band_mid": (110, 380), "noise_band_high": 380},
    5:  {"gyro_lpf_range": (60, 300), "dterm_lpf_range": (40, 150), "filter_safety": 0.80,
         "noise_band_mid": (100, 350), "noise_band_high": 350},
    6:  {"gyro_lpf_range": (50, 250), "dterm_lpf_range": (35, 130), "filter_safety": 0.78,
         "noise_band_mid": (80, 300), "noise_band_high": 300},
    7:  {"gyro_lpf_range": (40, 200), "dterm_lpf_range": (30, 110), "filter_safety": 0.75,
         "noise_band_mid": (60, 250), "noise_band_high": 250},
    8:  {"gyro_lpf_range": (35, 180), "dterm_lpf_range": (25, 100), "filter_safety": 0.72,
         "noise_band_mid": (50, 220), "noise_band_high": 220},
    9:  {"gyro_lpf_range": (30, 150), "dterm_lpf_range": (20, 85),  "filter_safety": 0.70,
         "noise_band_mid": (40, 180), "noise_band_high": 180},
    10: {"gyro_lpf_range": (25, 120), "dterm_lpf_range": (18, 75),  "filter_safety": 0.68,
         "noise_band_mid": (30, 150), "noise_band_high": 150},
    12: {"gyro_lpf_range": (20, 100), "dterm_lpf_range": (15, 60),  "filter_safety": 0.65,
         "noise_band_mid": (20, 120), "noise_band_high": 120},
}

# Map sizes to nearest profile key
FRAME_SIZE_MAP = {3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 12, 12: 12, 13: 12, 14: 12, 15: 12}
PROP_SIZE_MAP = {3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 12, 12: 12, 13: 12, 14: 12, 15: 12}


def _nearest_key(size_map, inches):
    """Find nearest key in a size map."""
    key = size_map.get(inches)
    if key is None:
        key = min(size_map.keys(), key=lambda k: abs(k - inches))
        key = size_map[key]
    return key


def get_frame_profile(frame_inches=None, prop_inches=None, n_blades=3):
    """Build a complete profile by merging frame response thresholds with prop noise config.

    Args:
        frame_inches: Frame size (determines PID thresholds). Default: 5.
        prop_inches: Prop diameter (determines filter ranges). Default: same as frame.
        n_blades: Number of prop blades (2, 3, 4+). Default: 3 (triblade).
                  Affects harmonic prediction and filter range adjustment.

    Returns:
        Merged profile dict with all thresholds, filter ranges, and prop config.
    """
    if frame_inches is None:
        frame_inches = 5
    if prop_inches is None:
        prop_inches = frame_inches  # most common: prop matches frame

    frame_key = _nearest_key(FRAME_SIZE_MAP, frame_inches)
    prop_key = _nearest_key(PROP_SIZE_MAP, prop_inches)

    frame = FRAME_PROFILES[frame_key]
    prop_noise = PROP_NOISE_PROFILES[prop_key]

    # Blade count affects filter ranges - more blades push harmonics higher,
    # which actually gives more room between signal and noise.
    # 2-blade = baseline. 3-blade = harmonics 1.5× higher. Adjust ranges up.
    blade_factor = n_blades / 2.0  # 1.0 for biblade, 1.5 for triblade
    adjusted_noise = dict(prop_noise)
    if blade_factor > 1.0:
        # More blades = harmonics shift up = can afford higher filter cutoffs
        low, high = prop_noise["gyro_lpf_range"]
        adjusted_noise["gyro_lpf_range"] = (
            int(low * min(blade_factor, 1.3)),   # don't go crazy, cap at 30% boost
            int(high * min(blade_factor, 1.3)),
        )
        low, high = prop_noise["dterm_lpf_range"]
        adjusted_noise["dterm_lpf_range"] = (
            int(low * min(blade_factor, 1.3)),
            int(high * min(blade_factor, 1.3)),
        )
        mid_lo, mid_hi = prop_noise["noise_band_mid"]
        adjusted_noise["noise_band_mid"] = (
            int(mid_lo * blade_factor),
            int(mid_hi * blade_factor),
        )
        adjusted_noise["noise_band_high"] = int(prop_noise["noise_band_high"] * blade_factor)

    # Merge: frame thresholds + adjusted prop noise config + metadata
    profile = {**frame, **adjusted_noise}
    profile["frame_inches"] = frame_inches
    profile["prop_inches"] = prop_inches
    profile["n_blades"] = n_blades
    profile["name"] = f"{frame_inches}\"-frame / {prop_inches}\"×{n_blades}-blade"
    if frame_inches == prop_inches and n_blades == 3:
        profile["name"] = frame["name"]  # clean default: "5-inch"
    elif frame_inches == prop_inches and n_blades == 2:
        profile["name"] = f"{frame['name']} (biblade)"

    return profile


# ─── RPM and Phase Lag Estimation ─────────────────────────────────────────────

def estimate_rpm_range(motor_kv=None, cell_count=None, vbat_avg=None):
    """Estimate motor RPM range from KV and voltage.
    Returns (idle_rpm, max_rpm) or None if insufficient data."""
    voltage = None
    if vbat_avg and vbat_avg > 5:
        voltage = vbat_avg  # use measured
    elif cell_count:
        voltage = cell_count * 3.7  # nominal cell voltage

    if motor_kv and voltage:
        max_rpm = motor_kv * voltage
        idle_rpm = int(max_rpm * 0.15)  # ~15% throttle idle
        return (idle_rpm, int(max_rpm))
    return None


def estimate_prop_harmonics(rpm_range, n_blades=2):
    """Estimate expected vibration frequencies from RPM and blade count.

    The fundamental vibration frequency of a prop is:
        freq = (RPM × n_blades) / 60

    A 2-blade prop at 25000 RPM: 833 Hz fundamental
    A 3-blade prop at 25000 RPM: 1250 Hz fundamental (1.5× higher)
    A 2-blade prop at 10000 RPM: 333 Hz fundamental

    Returns list of harmonics with frequency ranges (min=idle, max=full throttle).
    """
    if rpm_range is None:
        return []
    idle_rpm, max_rpm = rpm_range
    fundamental_min = (idle_rpm * n_blades) / 60.0
    fundamental_max = (max_rpm * n_blades) / 60.0
    harmonics = []
    for h in range(1, 4):  # fundamental + 2 harmonics
        harmonics.append({
            "harmonic": h,
            "min_hz": fundamental_min * h,
            "max_hz": fundamental_max * h,
            "label": ["fundamental", "2nd harmonic", "3rd harmonic"][h - 1],
        })
    return harmonics


def _phase_shift(filter_type, freq_hz, cutoff_hz, q=None):
    """
    Compute phase lag (degrees) for a given filter at a specific frequency.

    Parameters
    ----------
    filter_type : str
        One of 'PT1', 'PT2', 'PT3', 'BIQUAD'.
    freq_hz : float
        Signal frequency of interest (Hz).
    cutoff_hz : float
        Filter cutoff frequency (Hz).
    q : float, optional
        Q factor (only used for BIQUAD). Default 0.5 (Butterworth).

    Returns
    -------
    float
        Phase lag in degrees (negative, indicating lag).
    """
    if cutoff_hz <= 0 or freq_hz <= 0:
        return 0.0

    filter_type = str(filter_type).upper()

    # PT1: first-order low-pass
    if filter_type == "PT1":
        return -np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # PT2: two PT1 in series
    elif filter_type == "PT2":
        return -2 * np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # PT3: three PT1 in series
    elif filter_type == "PT3":
        return -3 * np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # BIQUAD (2nd order low-pass)
    elif "BIQUAD" in filter_type:
        if q is None:
            q = 0.5  # default Butterworth
        w = 2 * np.pi * freq_hz
        wc = 2 * np.pi * cutoff_hz
        # Transfer function: H(s) = wc² / (s² + (wc/q)s + wc²)
        # Phase = -atan2( (w/q)*wc , wc² - w² )
        imag = (w / q) * wc
        real = wc * wc - w * w
        return -np.degrees(np.arctan2(imag, real))

    # Fallback for unknown types
    else:
        order = {"PT1": 1, "PT2": 2, "PT3": 3, "BIQUAD": 2}.get(filter_type, 1)
        return -order * np.degrees(np.arctan2(freq_hz, cutoff_hz))


def estimate_filter_phase_lag(filter_hz, signal_freq, filter_type="PT1", q=None):
    """
    Estimate phase lag in degrees and milliseconds for a single filter.

    Parameters
    ----------
    filter_hz : float
        Filter cutoff frequency (Hz).
    signal_freq : float
        Signal frequency of interest (Hz).
    filter_type : str
        Filter type (e.g., 'PT1', 'BIQUAD').
    q : float, optional
        Q factor for BIQUAD (default None → 0.5).

    Returns
    -------
    dict
        {'degrees': phase_lag_deg, 'ms': delay_ms}
    """
    if filter_hz <= 0 or signal_freq <= 0:
        return {"degrees": 0.0, "ms": 0.0}

    phase_deg = _phase_shift(filter_type, signal_freq, filter_hz, q)

    # Convert phase at this frequency to a time delay
    delay_ms = (abs(phase_deg) / 360.0) / signal_freq * 1000.0

    return {"degrees": phase_deg, "ms": delay_ms}


def estimate_total_phase_lag(config, profile, signal_freq=50.0):
    """Estimate total phase lag through the entire filter chain at a given frequency.
    This is the key metric for large quads - too much lag and the PID loop fights itself."""

    total_deg = 0.0
    total_ms = 0.0
    chain = []

    # Gyro lowpass 1
    glp = config.get("gyro_lowpass_hz")
    if glp and isinstance(glp, (int, float)) and glp > 0:
        glp_type = config.get("gyro_lowpass_type", "PT1")
        lag = estimate_filter_phase_lag(glp, signal_freq, glp_type)
        total_deg += lag["degrees"]
        total_ms += lag["ms"]
        chain.append({"name": f"Gyro LPF ({glp}Hz, {glp_type})", **lag})

    # Gyro lowpass 2
    glp2 = config.get("gyro_lowpass2_hz")
    if glp2 and isinstance(glp2, (int, float)) and glp2 > 0:
        glp2_type = config.get("gyro_lowpass2_type", "PT1")
        lag = estimate_filter_phase_lag(glp2, signal_freq, glp2_type)
        total_deg += lag["degrees"]
        total_ms += lag["ms"]
        chain.append({"name": f"Gyro LPF2 ({glp2}Hz, {glp2_type})", **lag})

    # D-term lowpass (affects D only, but contributes to PID output timing)
    dlp = config.get("dterm_lpf_hz")
    if dlp and isinstance(dlp, (int, float)) and dlp > 0:
        dlp_type = config.get("dterm_lpf_type", "PT1")
        lag = estimate_filter_phase_lag(dlp, signal_freq, dlp_type)
        chain.append({"name": f"D-term LPF ({dlp}Hz, {dlp_type})", **lag,
                       "note": "affects D-term only"})

    return {
        "total_degrees": total_deg,
        "total_ms": total_ms,
        "signal_freq": signal_freq,
        "chain": chain,
    }


def analyze_motor_response(data, sr):
    """Measure how fast motors respond to PID demands.
    Cross-correlates PID output sum with motor output to estimate motor lag."""
    # Sum P+I+D for roll as proxy for total PID command
    pid_sum = None
    for axis in ["roll"]:
        components = []
        for term in ["P", "I", "D"]:
            key = f"axis{term}_{axis}"
            if key in data:
                components.append(data[key])
        if components:
            pid_sum = np.sum(components, axis=0)
            break

    if pid_sum is None or "motor0" not in data:
        return None

    # Use motor 0 as reference
    motor = data["motor0"]
    mask = ~(np.isnan(pid_sum) | np.isnan(motor))
    pid_clean = pid_sum[mask]
    motor_clean = motor[mask]

    if len(pid_clean) < 500:
        return None

    # Cross-correlate to find motor response delay
    n_corr = min(10000, len(pid_clean))
    pid_seg = pid_clean[:n_corr] - np.mean(pid_clean[:n_corr])
    mot_seg = motor_clean[:n_corr] - np.mean(motor_clean[:n_corr])

    corr = np.correlate(mot_seg, pid_seg, mode="full")
    mid = len(corr) // 2
    max_lag = int(sr * 0.05)  # search up to 50ms
    search = corr[mid:mid + max(1, max_lag)]

    if len(search) > 0 and np.max(search) > 0:
        delay_samples = np.argmax(search)
        delay_ms = float(delay_samples / sr * 1000.0)
    else:
        delay_ms = 0.0

    return {"motor_response_ms": delay_ms}


# Legacy constants for backward compat (overridden by profile when available)
GOOD_OVERSHOOT = 8.0
OK_OVERSHOOT = 15.0
BAD_OVERSHOOT = 30.0
GOOD_DELAY_MS = 8.0
OK_DELAY_MS = 15.0
BAD_DELAY_MS = 30.0
GOOD_NOISE_HIGH_DB = -45
OK_NOISE_HIGH_DB = -35
BAD_NOISE_HIGH_DB = -25
MOTOR_SAT_OK = 2.0
MOTOR_SAT_WARN = 8.0
MOTOR_IMBAL_OK = 3.0
MOTOR_IMBAL_WARN = 8.0
PID_ADJUST_FACTOR = 0.25
FILTER_ADJUST_FACTOR = 0.3


# ─── INAV Parameter Extraction ───────────────────────────────────────────────

INAV_PARAM_MAP = {
    "rollPID": "pid_roll", "pitchPID": "pid_pitch", "yawPID": "pid_yaw",
    "roll_p": "roll_p", "roll_i": "roll_i", "roll_d": "roll_d",
    "pitch_p": "pitch_p", "pitch_i": "pitch_i", "pitch_d": "pitch_d",
    "yaw_p": "yaw_p", "yaw_i": "yaw_i", "yaw_d": "yaw_d",
    # Filter settings (underscore format - Betaflight-style headers)
    "gyro_lpf": "gyro_lpf_type", "gyro_hardware_lpf": "gyro_hw_lpf",
    "gyro_lowpass_hz": "gyro_lowpass_hz", "gyro_lowpass_type": "gyro_lowpass_type",
    "gyro_lowpass2_hz": "gyro_lowpass2_hz", "gyro_lowpass2_type": "gyro_lowpass2_type",
    "dterm_lpf_hz": "dterm_lpf_hz", "dterm_lpf_type": "dterm_lpf_type",
    "dterm_lpf2_hz": "dterm_lpf2_hz", "dterm_lpf2_type": "dterm_lpf2_type",
    "yaw_lpf_hz": "yaw_lpf_hz",
    # Filter settings (camelCase - INAV 9.x blackbox header format)
    "gyro_lpf_hz": "gyro_lowpass_hz",
    # Dynamic notch (underscore - Betaflight-style)
    "dyn_notch_width_percent": "dyn_notch_width", "dyn_notch_q": "dyn_notch_q",
    "dyn_notch_min_hz": "dyn_notch_min_hz", "dyn_notch_max_hz": "dyn_notch_max_hz",
    "dynamic_gyro_notch_enabled": "dyn_notch_enabled",
    "dynamic_gyro_notch_q": "dyn_notch_q",
    "dynamic_gyro_notch_min_hz": "dyn_notch_min_hz",
    # Dynamic notch (camelCase - INAV blackbox headers)
    "dynamicGyroNotchQ": "dyn_notch_q",
    "dynamicGyroNotchMinHz": "dyn_notch_min_hz",
    "dynamicGyroNotchEnabled": "dyn_notch_enabled",
    "dynamicGyroNotchMode": "dyn_notch_mode",
    # RPM filter
    "rpm_gyro_filter_enabled": "rpm_filter_enabled",
    "rpm_gyro_harmonics": "rpm_filter_harmonics",
    "rpm_gyro_min_hz": "rpm_filter_min_hz",
    "rpm_gyro_q": "rpm_filter_q",
    # PID headers (camelCase INAV format - parsed separately below)
    "rollPID": "_rollPID", "pitchPID": "_pitchPID", "yawPID": "_yawPID",
    # Timing
    "looptime": "looptime", "gyro_sync_denom": "gyro_sync_denom",
    "pid_process_denom": "pid_denom",
    # Motor/protocol
    "motor_pwm_protocol": "motor_protocol", "motor_pwm_rate": "motor_pwm_rate",
    # Identity
    "firmwareVersion": "firmware_version", "firmwareType": "firmware_type",
    "Firmware revision": "firmware_revision",
    "Firmware type": "firmware_type", "Firmware date": "firmware_date",
    "craftName": "craft_name", "Craft name": "craft_name",
    "motorPwmProtocol": "motor_protocol", "features": "features",
}


def parse_headers_from_bbl(filepath):
    raw_params = {}
    try:
        with open(filepath, "rb") as f:
            for line in f:
                try:
                    text = line.decode("utf-8", errors="ignore").strip()
                except:
                    continue
                if text.startswith("H "):
                    parts = text[2:].split(":", 1)
                    if len(parts) == 2:
                        raw_params[parts[0].strip()] = parts[1].strip()
                elif text and not text.startswith("H"):
                    break
    except:
        pass
    return raw_params


def extract_fc_config(raw_params):
    config = {"raw": raw_params}
    for raw_key, our_key in INAV_PARAM_MAP.items():
        if raw_key in raw_params:
            config[our_key] = raw_params[raw_key]

    # Parse PID values from "rollPID:26,75,35,100" format (INAV header style)
    for axis in ["roll", "pitch", "yaw"]:
        pid_raw_key = f"_{axis}PID"
        if pid_raw_key in config:
            try:
                parts = config[pid_raw_key].split(",")
                if len(parts) >= 3:
                    config[f"{axis}_p"] = int(parts[0].strip())
                    config[f"{axis}_i"] = int(parts[1].strip())
                    config[f"{axis}_d"] = int(parts[2].strip())
                if len(parts) >= 4:
                    config[f"{axis}_ff"] = int(parts[3].strip())
            except (ValueError, IndexError):
                pass

    # Also parse "pid_roll" style keys (Betaflight/older format)
    for axis in ["roll", "pitch", "yaw"]:
        pid_key = f"pid_{axis}"
        if pid_key in config and f"{axis}_p" not in config:
            try:
                parts = config[pid_key].split(",")
                if len(parts) >= 3:
                    config[f"{axis}_p"] = int(parts[0].strip())
                    config[f"{axis}_i"] = int(parts[1].strip())
                    config[f"{axis}_d"] = int(parts[2].strip())
            except (ValueError, IndexError):
                pass

    # Convert numeric fields
    for key in ["gyro_lowpass_hz", "gyro_lowpass2_hz", "dterm_lpf_hz", "dterm_lpf2_hz",
                "yaw_lpf_hz", "dterm_lpf_type",
                "dyn_notch_min_hz", "dyn_notch_max_hz", "dyn_notch_q", "dyn_notch_width",
                "rpm_filter_enabled", "rpm_filter_harmonics", "rpm_filter_min_hz", "rpm_filter_q",
                "looptime", "gyro_sync_denom", "pid_denom", "motor_pwm_rate"]:
        if key in config:
            try:
                config[key] = int(config[key])
            except:
                try:
                    config[key] = float(config[key])
                except:
                    pass

    # Infer dynamic notch enabled state if not explicitly logged
    # INAV blackbox headers don't always include the enabled flag,
    # but Q > 0 means the filter is configured and active
    if "dyn_notch_enabled" not in config or config["dyn_notch_enabled"] is None:
        q = config.get("dyn_notch_q")
        if isinstance(q, (int, float)) and q > 0:
            config["dyn_notch_enabled"] = 1
        elif q is not None:
            config["dyn_notch_enabled"] = 0

    # Normalize enabled flags to int
    for flag_key in ["dyn_notch_enabled", "rpm_filter_enabled"]:
        val = config.get(flag_key)
        if isinstance(val, str):
            config[flag_key] = 1 if val.upper() in ("ON", "1", "TRUE", "YES") else 0

    for axis in ["roll", "pitch", "yaw"]:
        for term in ["p", "i", "d", "ff"]:
            key = f"{axis}_{term}"
            if key in config and isinstance(config[key], str):
                try:
                    config[key] = int(config[key])
                except:
                    pass

    # Map numeric filter types to names (INAV logs numeric codes)
    # INAV: 0=PT1, 1=PT1(alt), 2=BIQUAD, 3=PT2, 4=PT3
    FILTER_TYPE_NAMES = {0: "PT1", 1: "PT1", 2: "BIQUAD", 3: "PT2", 4: "PT3"}
    for ftype_key in ["dterm_lpf_type", "dterm_lpf2_type", "gyro_lowpass_type", "gyro_lowpass2_type"]:
        val = config.get(ftype_key)
        if isinstance(val, (int, float)):
            config[ftype_key] = FILTER_TYPE_NAMES.get(int(val), f"UNKNOWN({val})")

    # Infer gyro_lowpass_type from gyro_lpf_type if not explicitly set
    # INAV 9 logs "gyro_lpf" (hardware LPF enum) but not the software filter type
    if config.get("gyro_lowpass_type") is None or config.get("gyro_lowpass_type") == 0:
        config.setdefault("gyro_lowpass_type", "PT1")  # safe default

    # Parse motor output range from header: "motorOutput = 1080,2000"
    mo = raw_params.get("motorOutput", "")
    if "," in mo:
        try:
            parts = mo.split(",")
            config["motor_output_low"] = int(parts[0].strip())
            config["motor_output_high"] = int(parts[1].strip())
        except (ValueError, IndexError):
            pass
    mt = raw_params.get("minthrottle")
    if mt:
        try:
            config["minthrottle"] = int(mt)
        except ValueError:
            pass

    # Map numeric motor protocol to name for DSHOT detection
    # INAV: 0=PWM, 1=ONESHOT125, 2=ONESHOT42, 3=MULTISHOT, 4=BRUSHED,
    #        5=DSHOT150, 6=DSHOT300, 7=DSHOT600, 8=DSHOT1200
    MOTOR_PROTOCOL_NAMES = {
        0: "PWM", 1: "ONESHOT125", 2: "ONESHOT42", 3: "MULTISHOT",
        4: "BRUSHED", 5: "DSHOT150", 6: "DSHOT300", 7: "DSHOT600", 8: "DSHOT1200",
    }
    mp = config.get("motor_protocol")
    if isinstance(mp, (int, float)):
        config["motor_protocol"] = MOTOR_PROTOCOL_NAMES.get(int(mp), str(mp))
    elif isinstance(mp, str):
        try:
            mp_int = int(mp)
            config["motor_protocol"] = MOTOR_PROTOCOL_NAMES.get(mp_int, mp)
        except ValueError:
            pass  # already a string like "DSHOT300"

    return config


def config_has_pid(config):
    return isinstance(config.get("roll_p"), (int, float))

def config_has_filters(config):
    return isinstance(config.get("gyro_lowpass_hz"), (int, float))


# ── CLI diff → config merge ──────────────────────────────────────────────────

# Reverse map: INAV CLI setting name → our internal config key
# This is comprehensive - covers PID, filters, rates, nav, motors, and more
CLI_TO_CONFIG = {
    # PID gains (multicopter)
    "mc_p_roll": "roll_p", "mc_i_roll": "roll_i", "mc_d_roll": "roll_d",
    "mc_p_pitch": "pitch_p", "mc_i_pitch": "pitch_i", "mc_d_pitch": "pitch_d",
    "mc_p_yaw": "yaw_p", "mc_i_yaw": "yaw_i", "mc_d_yaw": "yaw_d",
    "mc_cd_roll": "roll_ff", "mc_cd_pitch": "pitch_ff", "mc_cd_yaw": "yaw_ff",

    # Gyro filters
    "gyro_main_lpf_hz": "gyro_lowpass_hz",
    "gyro_main_lpf_type": "gyro_lowpass_type",
    "gyro_main_lpf2_hz": "gyro_lowpass2_hz",

    # D-term filters
    "dterm_lpf_hz": "dterm_lpf_hz",
    "dterm_lpf_type": "dterm_lpf_type",
    "dterm_lpf2_hz": "dterm_lpf2_hz",
    "dterm_lpf2_type": "dterm_lpf2_type",

    # Dynamic notch
    "dynamic_gyro_notch_enabled": "dyn_notch_enabled",
    "dynamic_gyro_notch_q": "dyn_notch_q",
    "dynamic_gyro_notch_min_hz": "dyn_notch_min_hz",
    "dynamic_gyro_notch_count": "dyn_notch_count",

    # RPM filter
    "rpm_gyro_filter_enabled": "rpm_filter_enabled",
    "rpm_gyro_filter_harmonics": "rpm_filter_harmonics",
    "rpm_gyro_filter_min_hz": "rpm_filter_min_hz",
    "rpm_gyro_filter_q": "rpm_filter_q",

    # Motor
    "motor_pwm_protocol": "motor_protocol",
    "motor_pwm_rate": "motor_pwm_rate",
    "motor_poles": "motor_poles",
    "throttle_idle": "motor_idle",
    "3d_deadband_throttle": "deadband_3d",

    # Rates
    "rc_expo": "rc_expo",
    "rc_yaw_expo": "rc_yaw_expo",
    "roll_rate": "roll_rate",
    "pitch_rate": "pitch_rate",
    "yaw_rate": "yaw_rate",

    # Navigation (position hold, altitude)
    "nav_mc_pos_z_p": "nav_alt_p",
    "nav_mc_vel_z_p": "nav_vel_z_p",
    "nav_mc_vel_z_i": "nav_vel_z_i",
    "nav_mc_vel_z_d": "nav_vel_z_d",
    "nav_mc_pos_xy_p": "nav_pos_p",
    "nav_mc_vel_xy_p": "nav_vel_xy_p",
    "nav_mc_vel_xy_i": "nav_vel_xy_i",
    "nav_mc_vel_xy_d": "nav_vel_xy_d",
    "nav_mc_heading_p": "nav_heading_p",
    "nav_rth_altitude": "nav_rth_altitude",
    "nav_mc_hover_thr": "nav_hover_thr",

    # Compass / magnetometer
    "mag_hardware": "mag_hardware",
    "align_mag": "align_mag",
    "mag_calibration_0": "mag_cal_x",
    "mag_calibration_1": "mag_cal_y",
    "mag_calibration_2": "mag_cal_z",
    "mag_declination": "mag_declination",

    # GPS
    "gps_ublox_use_galileo": "gps_use_galileo",
    "gps_ublox_use_beidou": "gps_use_beidou",
    "gps_ublox_use_glonass": "gps_use_glonass",
    "gps_provider": "gps_provider",

    # Estimator weights
    "inav_w_z_baro_p": "inav_w_z_baro_p",
    "inav_w_z_gps_p": "inav_w_z_gps_p",
    "inav_w_xy_gps_p": "inav_w_xy_gps_p",

    # Level mode / angle
    "mc_p_level": "level_p",
    "mc_i_level": "level_i",
    "mc_d_level": "level_d",

    # Anti-gravity
    "antigravity_gain": "antigravity_gain",
    "antigravity_cutoff": "antigravity_cutoff",

    # Misc
    "looptime": "looptime",
    "gyro_sync": "gyro_sync",
    "blackbox_rate_num": "bb_rate_num",
    "blackbox_rate_denom": "bb_rate_denom",
    "blackbox_device": "bb_device",
    "name": "craft_name",
}

# Settings that should be interpreted as integers
CLI_INT_KEYS = {
    "roll_p", "roll_i", "roll_d", "roll_ff",
    "pitch_p", "pitch_i", "pitch_d", "pitch_ff",
    "yaw_p", "yaw_i", "yaw_d", "yaw_ff",
    "gyro_lowpass_hz", "gyro_lowpass2_hz",
    "dterm_lpf_hz", "dterm_lpf2_hz",
    "dyn_notch_q", "dyn_notch_min_hz", "dyn_notch_count",
    "rpm_filter_harmonics", "rpm_filter_min_hz", "rpm_filter_q",
    "motor_pwm_rate", "motor_poles", "motor_idle",
    "looptime", "bb_rate_num", "bb_rate_denom",
    "nav_alt_p", "nav_vel_z_p", "nav_vel_z_i", "nav_vel_z_d",
    "nav_pos_p", "nav_vel_xy_p", "nav_vel_xy_i", "nav_vel_xy_d",
    "nav_heading_p", "nav_rth_altitude", "nav_hover_thr",
    "level_p", "level_i", "level_d",
    "roll_rate", "pitch_rate", "yaw_rate",
    "rc_expo", "rc_yaw_expo",
    "antigravity_gain", "antigravity_cutoff",
    "mag_cal_x", "mag_cal_y", "mag_cal_z", "mag_declination",
    "inav_w_z_baro_p", "inav_w_z_gps_p", "inav_w_xy_gps_p",
}

# Settings that are boolean ON/OFF flags
CLI_BOOL_KEYS = {
    "dyn_notch_enabled", "rpm_filter_enabled", "gyro_sync",
    "gps_use_galileo", "gps_use_beidou", "gps_use_glonass",
}


def merge_diff_into_config(config, config_raw):
    """Merge INAV CLI 'diff all' output into the analysis config dict.

    Strategy:
    - Settings MISSING from blackbox headers: add from diff (nav PIDs,
      motor poles, rates, anti-gravity, etc.)
    - Settings PRESENT in blackbox headers: keep blackbox values (they
      represent what was actually flying), but store diff values as
      _diff_<key> for mismatch detection
    - All diff settings stored as cli_<name> for database reference

    Args:
        config: Existing config dict from extract_fc_config()
        config_raw: Raw 'diff all' output string

    Returns:
        Number of settings merged
    """
    if not config_raw:
        return 0

    try:
        from inav_toolkit.flight_db import parse_diff_output
    except ImportError:
        from inav_flight_db import parse_diff_output
    diff_settings = parse_diff_output(config_raw)

    merged = 0
    mismatches = []

    for cli_name, cli_value in diff_settings.items():
        config_key = CLI_TO_CONFIG.get(cli_name)

        # Store all diff settings with cli_ prefix for database
        config[f"cli_{cli_name}"] = cli_value

        if config_key is None:
            continue

        # Convert to appropriate type
        if config_key in CLI_BOOL_KEYS:
            val = 1 if cli_value.upper() in ("ON", "1", "TRUE", "YES") else 0
        elif config_key in CLI_INT_KEYS:
            try:
                val = int(cli_value)
            except ValueError:
                try:
                    val = float(cli_value)
                except ValueError:
                    val = cli_value
        else:
            val = cli_value

        existing = config.get(config_key)
        if existing is not None:
            # Blackbox header has this value - keep it, store diff as reference
            config[f"_diff_{config_key}"] = val
            # Detect mismatch (compare numerically if possible)
            try:
                if int(existing) != int(val):
                    mismatches.append((config_key, existing, val))
            except (ValueError, TypeError):
                if str(existing) != str(val):
                    mismatches.append((config_key, existing, val))
        else:
            # New setting not in blackbox headers - add it
            config[config_key] = val
            merged += 1

    config["_diff_merged"] = True
    config["_diff_settings_count"] = merged
    config["_diff_unmapped_count"] = len([k for k in diff_settings if CLI_TO_CONFIG.get(k) is None])
    config["_diff_mismatches"] = mismatches

    return merged


# ─── Blackbox Decode ──────────────────────────────────────────────────────────

# ─── Native Blackbox Binary Decoder ─────────────────────────────────────────
# Decodes .bbl/.bfl/.bbs binary logs directly - no external tools needed.
# Implements the INAV/Betaflight blackbox encoding: variable-byte integers,
# ZigZag signed encoding, grouped tag encodings (TAG2_3S32, TAG8_4S16,
# TAG8_8SVB), I-frame/P-frame predictors, and frame type dispatching.

class BlackboxDecoder:
    """Native Python decoder for INAV/Betaflight blackbox binary logs."""

    # Encoding types (from blackbox.c)
    ENC_SIGNED_VB = 0
    ENC_UNSIGNED_VB = 1
    ENC_NEG_14BIT = 3
    ENC_TAG8_8SVB = 6
    ENC_TAG2_3S32 = 7
    ENC_TAG8_4S16 = 8
    ENC_NULL = 9

    # Predictor types
    PRED_ZERO = 0
    PRED_PREVIOUS = 1
    PRED_STRAIGHT_LINE = 2
    PRED_AVERAGE_2 = 3
    PRED_MINTHROTTLE = 4
    PRED_MOTOR_0 = 5
    PRED_INC = 6
    PRED_1500 = 8
    PRED_VBATREF = 9

    FRAME_I, FRAME_P, FRAME_E = ord('I'), ord('P'), ord('E')
    FRAME_S, FRAME_G, FRAME_H = ord('S'), ord('G'), ord('H')
    VALID_FRAMES = {FRAME_I, FRAME_P, FRAME_E, FRAME_S, FRAME_G, FRAME_H}

    def __init__(self, raw_params):
        self.params = raw_params
        self.i_def = self._parse_field_def('I')
        self.p_def = self._parse_field_def('P', fallback_names=self.i_def)
        self.s_def = self._parse_field_def('S')
        self.g_def = self._parse_field_def('G')
        self._prev_gps = None  # Previous G-frame values for delta accumulation
        self._gps_home = [0, 0]  # GPS home position from H-frame
        self._h_def = self._parse_field_def('H')  # Home frame definition

        self.minthrottle = self._int_param('minthrottle', 1050)
        self.vbatref = self._int_param('vbatref', 0)
        mo = raw_params.get('motorOutput', '')
        self.motor_output_low = int(mo.split(',')[0]) if ',' in mo else self.minthrottle

        self._motor0_idx = None
        if self.i_def:
            try:
                self._motor0_idx = self.i_def['names'].index('motor[0]')
            except ValueError:
                pass

        self.stats = {'i_frames': 0, 'p_frames': 0, 'errors': 0,
                      'skipped_events': 0, 'skipped_slow': 0, 'skipped_gps': 0}
        # Storage for decoded auxiliary frames
        self.slow_frames = []   # (frame_index, {field: value}) - flight modes, failsafe
        self.gps_frames = []    # (frame_index, {field: value}) - GPS position/sats

    def _int_param(self, key, default=0):
        try:
            return int(self.params.get(key, default))
        except (ValueError, TypeError):
            return default

    def _parse_field_def(self, frame_type, fallback_names=None):
        names_str = self.params.get(f'Field {frame_type} name', '')
        if names_str:
            names = [n.strip() for n in names_str.split(',')]
        elif fallback_names:
            # P-frames reuse I-frame field names
            names = fallback_names['names']
        else:
            return None
        n = len(names)
        pred_str = self.params.get(f'Field {frame_type} predictor', '')
        enc_str = self.params.get(f'Field {frame_type} encoding', '')
        if not pred_str and not enc_str:
            return None  # No predictor/encoding = no frame definition
        return {
            'names': names,
            'signed': self._int_list(f'Field {frame_type} signed', n),
            'predictor': self._int_list(f'Field {frame_type} predictor', n),
            'encoding': self._int_list(f'Field {frame_type} encoding', n),
            'count': n,
        }

    def _int_list(self, key, pad_to=0):
        val = self.params.get(key, '')
        if not val:
            return [0] * pad_to
        result = []
        for x in val.split(','):
            x = x.strip()
            if x:
                try:
                    result.append(int(x))
                except ValueError:
                    result.append(0)
        return (result + [0] * pad_to)[:pad_to]

    # ─── Binary Primitives ───────────────────────────────────────────────────

    def _read_byte(self):
        if self.pos >= self.end:
            raise IndexError("EOF")
        b = self.buf[self.pos]
        self.pos += 1
        return b

    def _read_unsigned_vb(self):
        result, shift = 0, 0
        for _ in range(5):
            b = self._read_byte()
            result |= (b & 0x7F) << shift
            if not (b & 0x80):
                return result
            shift += 7
        return result

    def _read_signed_vb(self):
        u = self._read_unsigned_vb()
        return (u >> 1) ^ (-(u & 1))  # ZigZag decode

    def _read_neg_14bit(self):
        # NEG_14BIT: value is negated then encoded as unsigned variable-byte.
        # Decode: read unsigned VB, then negate.
        return -self._read_unsigned_vb()

    def _read_tag2_3s32(self):
        """TAG2_3S32: top 2 bits of lead byte select field size for all 3 values.
        Reference: betaflight/blackbox-tools/src/decoders.c streamReadTag2_3S32"""
        lead = self._read_byte()
        selector = lead >> 6
        if selector == 0:
            # 2-bit fields, all packed in the remaining 6 bits of lead byte
            v0 = (lead >> 4) & 0x03; v0 = v0 - 4 if v0 >= 2 else v0
            v1 = (lead >> 2) & 0x03; v1 = v1 - 4 if v1 >= 2 else v1
            v2 = lead & 0x03;        v2 = v2 - 4 if v2 >= 2 else v2
            return [v0, v1, v2]
        elif selector == 1:
            # 4-bit fields: first in low nibble of lead, next 2 in second byte
            v0 = lead & 0x0F; v0 = v0 - 16 if v0 >= 8 else v0
            b = self._read_byte()
            v1 = b >> 4;     v1 = v1 - 16 if v1 >= 8 else v1
            v2 = b & 0x0F;   v2 = v2 - 16 if v2 >= 8 else v2
            return [v0, v1, v2]
        elif selector == 2:
            # 6-bit fields: first in low 6 bits of lead, next 2 each in 1 byte
            v0 = lead & 0x3F; v0 = v0 - 64 if v0 >= 32 else v0
            b1 = self._read_byte()
            v1 = b1 & 0x3F;  v1 = v1 - 64 if v1 >= 32 else v1
            b2 = self._read_byte()
            v2 = b2 & 0x3F;  v2 = v2 - 64 if v2 >= 32 else v2
            return [v0, v1, v2]
        else:
            # selector == 3: per-field variable size (0/8/16/24 bit)
            values = [0, 0, 0]
            for i in range(3):
                tag = (lead >> (i * 2)) & 0x03
                if tag == 0:
                    values[i] = 0
                elif tag == 1:
                    v = self._read_byte()
                    values[i] = v - 256 if v >= 128 else v
                elif tag == 2:
                    b1, b2 = self._read_byte(), self._read_byte()
                    v = (b1 << 8) | b2
                    values[i] = v - 65536 if v >= 32768 else v
                elif tag == 3:
                    b1, b2, b3 = self._read_byte(), self._read_byte(), self._read_byte()
                    v = (b1 << 16) | (b2 << 8) | b3
                    values[i] = v - 0x1000000 if v >= 0x800000 else v
            return values

    def _read_tag8_4s16(self):
        """TAG8_4S16 v2: nibble-buffered, 2-bit selectors for 4 fields.
        Reference: atomgomba/orangebox/decoders.py _tag8_4s16_v2"""
        selector = self._read_byte()
        values = [0, 0, 0, 0]
        nibble_idx = 0  # 0 = aligned, 1 = have pending low nibble in buf
        buf = 0
        for i in range(4):
            ft = selector & 0x03
            if ft == 0:  # zero
                values[i] = 0
            elif ft == 1:  # 4-bit nibble
                if nibble_idx == 0:
                    buf = self._read_byte()
                    v = buf >> 4
                    nibble_idx = 1
                else:
                    v = buf & 0x0F
                    nibble_idx = 0
                values[i] = v - 16 if v >= 8 else v
            elif ft == 2:  # 8-bit
                if nibble_idx == 0:
                    v = self._read_byte()
                else:
                    v = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    v |= buf >> 4
                    # nibble_idx stays 1
                values[i] = v - 256 if v >= 128 else v
            elif ft == 3:  # 16-bit
                if nibble_idx == 0:
                    b1, b2 = self._read_byte(), self._read_byte()
                    v = (b1 << 8) | b2
                else:
                    b1 = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    b1 |= buf >> 4
                    b2 = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    b2 |= buf >> 4
                    v = (b1 << 8) | b2
                    # nibble_idx stays 1
                values[i] = v - 65536 if v >= 32768 else v
            selector >>= 2
        return values

    def _read_tag8_8svb(self, count=8):
        header = self._read_byte()
        values = [0] * count
        for i in range(min(8, count)):
            if header & (1 << i):
                values[i] = self._read_signed_vb()
        return values

    # ─── Frame Decoding ──────────────────────────────────────────────────────

    def _decode_raw_values(self, field_def):
        count = field_def['count']
        enc = field_def['encoding']
        values = [0] * count
        i = 0
        while i < count:
            e = enc[i]
            if e == self.ENC_NULL:
                values[i] = 0; i += 1
            elif e == self.ENC_SIGNED_VB:
                values[i] = self._read_signed_vb(); i += 1
            elif e == self.ENC_UNSIGNED_VB:
                values[i] = self._read_unsigned_vb(); i += 1
            elif e == self.ENC_NEG_14BIT:
                values[i] = self._read_neg_14bit(); i += 1
            elif e == self.ENC_TAG2_3S32:
                grp = self._read_tag2_3s32()
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += 3
            elif e == self.ENC_TAG8_4S16:
                grp = self._read_tag8_4s16()
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += 4
            elif e == self.ENC_TAG8_8SVB:
                # Count consecutive TAG8_8SVB fields (up to 8)
                run = 0
                while i + run < count and enc[i + run] == self.ENC_TAG8_8SVB and run < 8:
                    run += 1
                grp = self._read_tag8_8svb(run)
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += run
            else:
                values[i] = self._read_signed_vb(); i += 1
        return values

    def _apply_i_predictors(self, raw):
        values = list(raw)
        for i in range(self.i_def['count']):
            pred = self.i_def['predictor'][i]
            if pred == self.PRED_MINTHROTTLE:
                values[i] = raw[i] + self.minthrottle
            elif pred == self.PRED_MOTOR_0:
                if self._motor0_idx is not None and self._motor0_idx < i:
                    values[i] = raw[i] + values[self._motor0_idx]
            elif pred == self.PRED_VBATREF:
                values[i] = raw[i] + self.vbatref
            elif pred == self.PRED_1500:
                values[i] = raw[i] + 1500
        return values

    def _apply_p_predictors(self, raw, prev, prev_prev):
        values = list(raw)
        CLAMP = 2**31  # prevent overflow snowball from straight-line predictor
        for i in range(self.p_def['count']):
            pred = self.p_def['predictor'][i]
            if pred == self.PRED_ZERO:
                pass
            elif pred == self.PRED_PREVIOUS:
                values[i] = raw[i] + prev[i]
            elif pred == self.PRED_STRAIGHT_LINE:
                predicted = (2 * prev[i] - prev_prev[i]) if prev_prev else prev[i]
                values[i] = raw[i] + predicted
            elif pred == self.PRED_AVERAGE_2:
                predicted = ((prev[i] + prev_prev[i]) // 2) if prev_prev else prev[i]
                values[i] = raw[i] + predicted
            elif pred == self.PRED_MINTHROTTLE:
                values[i] = raw[i] + self.minthrottle
            elif pred == self.PRED_MOTOR_0:
                if self._motor0_idx is not None:
                    values[i] = raw[i] + values[self._motor0_idx]
                else:
                    values[i] = raw[i] + prev[i]
            elif pred == self.PRED_INC:
                values[i] = prev[i] + 1
            elif pred == self.PRED_1500:
                values[i] = raw[i] + 1500
            elif pred == self.PRED_VBATREF:
                values[i] = raw[i] + self.vbatref
            else:
                values[i] = raw[i] + prev[i]
            # Clamp to prevent overflow compounding across frames
            if values[i] > CLAMP or values[i] < -CLAMP:
                values[i] = prev[i] if prev else 0
        return values

    def _resync(self):
        while self.pos < self.end:
            if self.buf[self.pos] in self.VALID_FRAMES:
                return True
            self.pos += 1
        return False

    def _skip_frame(self, field_def):
        if field_def is None:
            self._resync()
            return
        try:
            self._decode_raw_values(field_def)
        except (IndexError, ValueError):
            self._resync()

    def _decode_aux_frame(self, field_def):
        """Decode an S frame and return {field_name: value} dict.
        S-frames use predictor 0 (raw values) - no delta prediction needed."""
        if field_def is None:
            self._resync()
            return None
        try:
            raw = self._decode_raw_values(field_def)
            return dict(zip(field_def['names'], raw))
        except (IndexError, ValueError):
            self._resync()
            return None

    def _decode_gps_frame(self):
        """Decode a G-frame with predictor-aware accumulation.

        GPS coordinates use predictor 7 (delta from GPS home position stored
        in H-frame). Other fields may use predictor 1 (delta from previous)
        or predictor 0 (raw). Without this, GPS coords show as small deltas
        instead of absolute lat/lon values.
        """
        field_def = self.g_def
        if field_def is None:
            self._resync()
            return None
        try:
            raw = self._decode_raw_values(field_def)
            predictors = field_def.get('predictor', [])

            # Initialize previous values on first G-frame
            if self._prev_gps is None:
                self._prev_gps = [0] * len(raw)

            # Apply predictors per field
            accumulated = []
            home_idx = 0  # tracks which GPS_home coordinate to use
            for i, val in enumerate(raw):
                pred = predictors[i] if i < len(predictors) else 0
                if pred == 7:
                    # Predictor 7: GPS home + delta
                    if home_idx < len(self._gps_home):
                        acc = self._gps_home[home_idx] + val
                    else:
                        acc = val
                    home_idx += 1
                    accumulated.append(acc)
                elif pred == 1:
                    # Predictor 1: delta from previous value
                    acc = self._prev_gps[i] + val
                    accumulated.append(acc)
                else:
                    # Predictor 0 and others: raw value
                    accumulated.append(val)

            # Store for predictor-1 fields on next frame
            self._prev_gps = list(accumulated)

            return dict(zip(field_def['names'], accumulated))
        except (IndexError, ValueError):
            self._resync()
            return None

    def _find_binary_start(self):
        """Find byte offset where headers end and binary data begins."""
        pos = 0
        last_header_end = 0
        while pos < min(self.end, 65536):  # headers are always in first 64KB
            nl = -1
            for i in range(pos, min(pos + 2000, self.end)):
                if self.buf[i] == ord('\n'):
                    nl = i
                    break
            if nl == -1:
                break
            if nl - pos >= 2 and self.buf[pos] == ord('H') and self.buf[pos + 1] == ord(' '):
                last_header_end = nl + 1
                pos = nl + 1
                continue
            if self.buf[pos] in self.VALID_FRAMES:
                return pos
            pos = nl + 1
        return last_header_end

    # ─── Event Frame Parsing ────────────────────────────────────────────────

    # INAV/BF event types and their data sizes
    EVT_SYNC_BEEP = 0           # data: 1 unsigned VB (time)
    EVT_INFLIGHT_ADJUSTMENT = 13 # data: 1 byte (func) + 1 signed VB or 4 bytes
    EVT_LOGGING_RESUME = 14      # data: unsigned VB (iter) + unsigned VB (time)
    EVT_FLIGHT_MODE = 15         # data: unsigned VB (flags) + unsigned VB (last)
    EVT_LOG_END = 255            # end of this log

    def _skip_event_frame(self):
        """Parse and skip an event frame by reading its type-specific data."""
        try:
            evt_type = self._read_byte()
            if evt_type == self.EVT_SYNC_BEEP:
                self._read_unsigned_vb()  # time
            elif evt_type == self.EVT_INFLIGHT_ADJUSTMENT:
                func = self._read_byte()
                if func < 128:
                    self._read_signed_vb()  # int value
                else:
                    for _ in range(4): self._read_byte()  # float value
            elif evt_type == self.EVT_LOGGING_RESUME:
                self._read_unsigned_vb()  # iteration
                self._read_unsigned_vb()  # currentTime
            elif evt_type == self.EVT_FLIGHT_MODE:
                self._read_unsigned_vb()  # flags
                self._read_unsigned_vb()  # lastFlags
            elif evt_type == self.EVT_LOG_END:
                # End of this log - scan for next set of headers or EOF
                self._skip_to_next_log()
                return 'log_end'
            else:
                # Unknown event type - try to resync
                self._resync()
            return 'ok'
        except (IndexError, ValueError):
            self._resync()
            return 'error'

    def _skip_to_next_log(self):
        """Skip forward past any new header block to the next binary data."""
        while self.pos < self.end:
            # Look for 'H ' (start of new headers)
            if self.pos + 1 < self.end and self.buf[self.pos] == ord('H') and self.buf[self.pos + 1] == ord(' '):
                # Skip header lines until we hit binary data
                while self.pos < self.end:
                    # Find newline
                    nl = -1
                    for i in range(self.pos, min(self.pos + 2000, self.end)):
                        if self.buf[i] == ord('\n'):
                            nl = i
                            break
                    if nl == -1:
                        self.pos = self.end
                        return
                    self.pos = nl + 1
                    # Check if next line is still a header
                    if self.pos + 1 < self.end and self.buf[self.pos] == ord('H') and self.buf[self.pos + 1] == ord(' '):
                        continue
                    else:
                        return  # Now at binary data start
            elif self.buf[self.pos] in self.VALID_FRAMES:
                return  # Found frame data
            self.pos += 1

    def _validate_i_frame(self, values):
        """Sanity check I-frame values. Returns False if clearly garbage."""
        n = len(values)
        # Check motor[0] - should be near minthrottle..maxthrottle range
        if self._motor0_idx is not None and self._motor0_idx < n:
            m0 = values[self._motor0_idx]
            max_motor = self._int_param('maxthrottle', 2000)
            if m0 < self.minthrottle - 200 or m0 > max_motor + 500:
                return False
        # Check time (field 1) - should be non-negative and < 1 hour in us
        if n > 1:
            t = values[1]
            if t < 0 or t > 3600_000_000:
                return False
        return True

    # ─── Main Decode ─────────────────────────────────────────────────────────

    def decode_file(self, filepath):
        """Decode a blackbox binary log. Returns (frames, field_names)."""
        with open(filepath, 'rb') as f:
            self.buf = f.read()
        self.end = len(self.buf)

        if self.i_def is None:
            print("  ERROR: No I-frame field definitions in headers")
            return [], []

        self.pos = self._find_binary_start()
        all_frames = []
        prev, prev_prev = None, None
        consec_errors = 0

        # Progress tracking
        last_pct = -1
        show_progress = self.end > 500_000 and not getattr(self, '_quiet', False)

        while self.pos < self.end:
            # Progress indicator
            if show_progress:
                pct = (self.pos * 100) // self.end
                if pct != last_pct and pct % 5 == 0:
                    frames_so_far = self.stats['i_frames'] + self.stats['p_frames']
                    print(f"\r  Decoding: {pct}% ({frames_so_far:,} frames)", end="", flush=True)
                    last_pct = pct

            byte = self.buf[self.pos]

            # Detect new log headers mid-stream (multi-log files)
            if byte == ord('H') and self.pos + 1 < self.end and self.buf[self.pos + 1] == ord(' '):
                self._skip_to_next_log()
                prev, prev_prev = None, None  # reset predictor state
                self.stats['skipped_events'] += 1
                continue

            if byte == self.FRAME_I:
                self.pos += 1
                saved = self.pos
                try:
                    raw = self._decode_raw_values(self.i_def)
                    values = self._apply_i_predictors(raw)
                    if self._validate_i_frame(values):
                        all_frames.append(values)
                        prev_prev, prev = prev, values
                        self.stats['i_frames'] += 1
                        consec_errors = 0
                    else:
                        # Invalid I-frame - likely false positive from resync
                        self.stats['errors'] += 1
                        self.pos = saved
                        if not self._resync(): break
                except (IndexError, ValueError):
                    consec_errors += 1
                    self.pos = saved
                    if not self._resync(): break
                    if consec_errors > 500: break

            elif byte == self.FRAME_P:
                self.pos += 1
                if prev is None:
                    if not self._resync(): break
                    continue
                saved = self.pos
                try:
                    raw = self._decode_raw_values(self.p_def)
                    values = self._apply_p_predictors(raw, prev, prev_prev)
                    all_frames.append(values)
                    prev_prev, prev = prev, values
                    self.stats['p_frames'] += 1
                    consec_errors = 0
                except (IndexError, ValueError):
                    consec_errors += 1
                    self.pos = saved
                    if not self._resync(): break
                    if consec_errors > 500: break

            elif byte == self.FRAME_E:
                self.pos += 1
                result = self._skip_event_frame()
                self.stats['skipped_events'] += 1
                if result == 'log_end':
                    prev, prev_prev = None, None  # reset for new log

            elif byte == self.FRAME_S:
                self.pos += 1
                frame_idx = len(all_frames)  # associate with current main frame position
                result = self._decode_aux_frame(self.s_def)
                if result is not None:
                    self.slow_frames.append((frame_idx, result))
                self.stats['skipped_slow'] += 1

            elif byte == self.FRAME_G:
                self.pos += 1
                frame_idx = len(all_frames)
                result = self._decode_gps_frame()
                if result is not None:
                    self.gps_frames.append((frame_idx, result))
                self.stats['skipped_gps'] += 1

            elif byte == self.FRAME_H:
                self.pos += 1
                # GPS Home frame — decode to capture home position
                if self._h_def:
                    try:
                        raw = self._decode_raw_values(self._h_def)
                        home_dict = dict(zip(self._h_def['names'], raw))
                        # Store home coordinates
                        h0 = home_dict.get("GPS_home[0]")
                        h1 = home_dict.get("GPS_home[1]")
                        if h0 is not None and h1 is not None:
                            self._gps_home = [int(h0), int(h1)]
                    except (IndexError, ValueError):
                        self._resync()
                elif self.g_def:
                    h_def = self._parse_field_def('H')
                    if h_def:
                        self._skip_frame(h_def)
                    else:
                        self._resync()
                else:
                    self._resync()

            else:
                self.pos += 1
                consec_errors += 1
                if consec_errors > 500: break

        if show_progress:
            print("\r  Decoding: done                              ", flush=True)
        return all_frames, self.i_def['names']

    def frames_to_data_dict(self, frames, field_names):
        """Convert decoded frames to analysis data dict (same format as parse_csv_log)."""
        if not frames:
            return None

        n_rows = len(frames)
        n_fields = len(field_names)

        # Clamp limit: any value beyond this is a decode error
        CLAMP = 2**31

        # Build arrays per field
        raw_arrays = {}
        for fi, name in enumerate(field_names):
            arr = np.empty(n_rows, dtype=np.float64)
            for ri in range(n_rows):
                f = frames[ri]
                if fi < len(f):
                    v = f[fi]
                    if -CLAMP <= v <= CLAMP:
                        arr[ri] = float(v)
                    else:
                        arr[ri] = np.nan
                else:
                    arr[ri] = np.nan
            raw_arrays[name] = arr

        # Map to standard analysis keys
        col_map = {
            "time": ["time"],
            "loopiter": ["loopIteration"],
            "gyro_roll": ["gyroADC[0]", "gyroData[0]", "gyro[0]"],
            "gyro_pitch": ["gyroADC[1]", "gyroData[1]", "gyro[1]"],
            "gyro_yaw": ["gyroADC[2]", "gyroData[2]", "gyro[2]"],
            "setpoint_roll": ["rcCommand[0]", "setpoint[0]"],
            "setpoint_pitch": ["rcCommand[1]", "setpoint[1]"],
            "setpoint_yaw": ["rcCommand[2]", "setpoint[2]"],
            "axisP_roll": ["axisP[0]"], "axisP_pitch": ["axisP[1]"], "axisP_yaw": ["axisP[2]"],
            "axisI_roll": ["axisI[0]"], "axisI_pitch": ["axisI[1]"], "axisI_yaw": ["axisI[2]"],
            "axisD_roll": ["axisD[0]"], "axisD_pitch": ["axisD[1]"], "axisD_yaw": ["axisD[2]"],
            "motor0": ["motor[0]"], "motor1": ["motor[1]"],
            "motor2": ["motor[2]"], "motor3": ["motor[3]"],
            # Navigation fields (decoded at full rate when present)
            "nav_pos_n": ["navPos[0]"], "nav_pos_e": ["navPos[1]"], "nav_pos_u": ["navPos[2]"],
            "nav_vel_n": ["navVel[0]"], "nav_vel_e": ["navVel[1]"], "nav_vel_u": ["navVel[2]"],
            "nav_tgt_n": ["navTgtPos[0]"], "nav_tgt_e": ["navTgtPos[1]"], "nav_tgt_u": ["navTgtPos[2]"],
            "nav_tgt_vel_n": ["navTgtVel[0]"], "nav_tgt_vel_e": ["navTgtVel[1]"], "nav_tgt_vel_u": ["navTgtVel[2]"],
            "nav_acc_n": ["navAcc[0]"], "nav_acc_e": ["navAcc[1]"], "nav_acc_u": ["navAcc[2]"],
            "nav_state": ["navState"], "nav_flags": ["navFlags"],
            "nav_eph": ["navEPH"], "nav_epv": ["navEPV"],
            "nav_surface": ["navSurf", "navSurf[0]"],
            "att_roll": ["attitude[0]"], "att_pitch": ["attitude[1]"], "att_heading": ["attitude[2]"],
            "baro_alt": ["BaroAlt"],
            "acc_x": ["accSmooth[0]"], "acc_y": ["accSmooth[1]"], "acc_z": ["accSmooth[2]"],
            "throttle": ["rcCommand[3]"],
            "rc_roll": ["rcData[0]"], "rc_pitch": ["rcData[1]"],
            "rc_yaw": ["rcData[2]"], "rc_throttle": ["rcData[3]"],
            "mc_pos_p_0": ["mcPosAxisP[0]"], "mc_pos_p_1": ["mcPosAxisP[1]"],
            "mc_pos_p_2": ["mcPosAxisP[2]"],
            "mc_vel_p_0": ["mcVelAxisP[0]"], "mc_vel_p_1": ["mcVelAxisP[1]"], "mc_vel_p_2": ["mcVelAxisP[2]"],
            "mc_vel_i_0": ["mcVelAxisI[0]"], "mc_vel_i_1": ["mcVelAxisI[1]"], "mc_vel_i_2": ["mcVelAxisI[2]"],
            "mc_vel_d_0": ["mcVelAxisD[0]"], "mc_vel_d_1": ["mcVelAxisD[1]"], "mc_vel_d_2": ["mcVelAxisD[2]"],
            "mc_vel_ff_0": ["mcVelAxisFF[0]"], "mc_vel_ff_1": ["mcVelAxisFF[1]"], "mc_vel_ff_2": ["mcVelAxisFF[2]"],
            "mc_vel_out_0": ["mcVelAxisOut[0]"], "mc_vel_out_1": ["mcVelAxisOut[1]"], "mc_vel_out_2": ["mcVelAxisOut[2]"],
            "mc_surf_p": ["mcSurfaceP"], "mc_surf_i": ["mcSurfaceI"],
            "mc_surf_d": ["mcSurfaceD"], "mc_surf_out": ["mcSurfaceOut"],
            "nav_tgt_hdg": ["navTgtHdg"],
            "vbat": ["vbat", "vbatLatest"],
            "amperage": ["amperage", "amperageLatest"],
            "rssi": ["rssi"],
        }

        data = {}
        for our_key, candidates in col_map.items():
            for fn in candidates:
                if fn in raw_arrays:
                    data[our_key] = raw_arrays[fn]
                    break

        # Time and sample rate
        # Prefer header-derived sample rate (reliable) over decoded time diffs (noisy)
        header_rate = None
        looptime = self._int_param('looptime', 0)
        p_interval_str = self.params.get('P interval', '')
        if looptime > 0 and '/' in p_interval_str:
            try:
                p_num, p_denom = p_interval_str.split('/')
                p_ratio = int(p_denom) / max(int(p_num), 1)  # e.g. "1/2" → every 2nd loop
                sample_period_us = looptime * p_ratio
                if sample_period_us > 0:
                    header_rate = 1e6 / sample_period_us
            except (ValueError, ZeroDivisionError):
                pass

        if header_rate and header_rate > 10:
            data["sample_rate"] = header_rate
            data["time_s"] = np.arange(n_rows) / header_rate
        elif "time" in data:
            time_us = data["time"]
            valid_time = time_us[~np.isnan(time_us)]
            if len(valid_time) > 10:
                diffs = np.diff(valid_time)
                good_diffs = diffs[(diffs > 0) & (diffs < 1e7)]
                if len(good_diffs) > 10:
                    data["sample_rate"] = float(1e6 / np.median(good_diffs))
                    data["time_s"] = np.arange(n_rows) / data["sample_rate"]
                else:
                    data["sample_rate"] = 1000.0
                    data["time_s"] = np.arange(n_rows) / 1000.0
            else:
                data["sample_rate"] = 1000.0
                data["time_s"] = np.arange(n_rows) / 1000.0
        else:
            data["sample_rate"] = 1000.0
            data["time_s"] = np.arange(n_rows) / 1000.0

        data["headers"] = field_names
        data["n_rows"] = n_rows
        data["found_columns"] = [k for k in col_map if k in data]
        return data


def decode_blackbox_native(filepath, raw_params, quiet=False):
    """Decode a blackbox binary log file using the native decoder.
    Returns a data dict in the same format as parse_csv_log."""
    decoder = BlackboxDecoder(raw_params)
    decoder._quiet = quiet
    frames, field_names = decoder.decode_file(filepath)

    total = decoder.stats['i_frames'] + decoder.stats['p_frames']
    if total == 0:
        if not quiet:
            print("  ERROR: No frames decoded from blackbox log.")
            print(f"    Stats: {decoder.stats}")
        sys.exit(1)

    if not quiet:
        print(f"  Decoded: {total:,} frames "
              f"({decoder.stats['i_frames']} I + {decoder.stats['p_frames']} P)")
        if decoder.stats['skipped_events'] or decoder.stats['skipped_gps']:
            print(f"    Aux frames: {decoder.stats['skipped_events']} event, "
                  f"{decoder.stats['skipped_slow']} slow, {decoder.stats['skipped_gps']} GPS")

    data = decoder.frames_to_data_dict(frames, field_names)
    if data is None:
        if not quiet:
            print("  ERROR: Failed to convert decoded frames to analysis data.")
        sys.exit(1)

    # Attach decoded auxiliary frame data for nav analysis
    data["_slow_frames"] = decoder.slow_frames    # [(frame_idx, {field: value})]
    data["_gps_frames"] = decoder.gps_frames      # [(frame_idx, {field: value})]
    data["_decoder_stats"] = decoder.stats         # {i_frames, p_frames, errors, ...}

    # Detect available nav fields for downstream analysis
    nav_fields = [k for k in data if k.startswith("nav_") or k.startswith("att_") or k == "baro_alt"]
    data["_has_nav"] = len(nav_fields) > 0
    data["_nav_fields"] = nav_fields

    return data


# ─── CSV Parsing ──────────────────────────────────────────────────────────────

def flexible_col_match(headers, patterns):
    for i, h in enumerate(headers):
        h_clean = h.strip().lower()
        for pat in patterns:
            if pat.lower() in h_clean:
                return i
    return None


def parse_csv_log(csv_path):
    with open(csv_path, "r", errors="ignore") as f:
        lines = [l.strip() for l in f if l.strip() and not l.strip().startswith(("#", "H "))]
    if len(lines) < 2:
        print("ERROR: CSV has insufficient data.")
        sys.exit(1)

    reader = csv.reader(lines)
    headers = [h.strip() for h in next(reader)]
    rows = list(reader)
    data = {}
    col_map = {
        "time": ["time (us)", "time(us)", "time_us", "time"],
        "loopiter": ["loopiteration", "loop_iteration"],
        "gyro_roll": ["gyroadc[0]", "gyrodata[0]", "gyro[0]", "gyro_roll"],
        "gyro_pitch": ["gyroadc[1]", "gyrodata[1]", "gyro[1]", "gyro_pitch"],
        "gyro_yaw": ["gyroadc[2]", "gyrodata[2]", "gyro[2]", "gyro_yaw"],
        "setpoint_roll": ["setpoint[0]", "rccommand[0]"],
        "setpoint_pitch": ["setpoint[1]", "rccommand[1]"],
        "setpoint_yaw": ["setpoint[2]", "rccommand[2]"],
        "axisP_roll": ["axisp[0]", "pidp[0]", "p[0]"],
        "axisP_pitch": ["axisp[1]", "pidp[1]", "p[1]"],
        "axisP_yaw": ["axisp[2]", "pidp[2]", "p[2]"],
        "axisI_roll": ["axisi[0]", "pidi[0]", "i[0]"],
        "axisI_pitch": ["axisi[1]", "pidi[1]", "i[1]"],
        "axisI_yaw": ["axisi[2]", "pidi[2]", "i[2]"],
        "axisD_roll": ["axisd[0]", "pidd[0]", "d[0]"],
        "axisD_pitch": ["axisd[1]", "pidd[1]", "d[1]"],
        "axisD_yaw": ["axisd[2]", "pidd[2]", "d[2]"],
        "motor0": ["motor[0]", "motor_0"], "motor1": ["motor[1]", "motor_1"],
        "motor2": ["motor[2]", "motor_2"], "motor3": ["motor[3]", "motor_3"],
        # Navigation fields
        "nav_pos_n": ["navpos[0]"], "nav_pos_e": ["navpos[1]"], "nav_pos_u": ["navpos[2]"],
        "nav_vel_n": ["navvel[0]"], "nav_vel_e": ["navvel[1]"], "nav_vel_u": ["navvel[2]"],
        "nav_tgt_n": ["navtgtpos[0]"], "nav_tgt_e": ["navtgtpos[1]"], "nav_tgt_u": ["navtgtpos[2]"],
        "nav_tgt_vel_n": ["navtgtvel[0]"], "nav_tgt_vel_e": ["navtgtvel[1]"], "nav_tgt_vel_u": ["navtgtvel[2]"],
        "nav_acc_n": ["navacc[0]"], "nav_acc_e": ["navacc[1]"], "nav_acc_u": ["navacc[2]"],
        "nav_state": ["navstate"], "nav_flags": ["navflags"],
        "nav_eph": ["naveph"], "nav_epv": ["navepv"],
        "nav_surface": ["navsurf", "navsurf[0]"],
        "att_roll": ["attitude[0]"], "att_pitch": ["attitude[1]"], "att_heading": ["attitude[2]"],
        "baro_alt": ["baroalt"],
        "acc_x": ["accsmooth[0]"], "acc_y": ["accsmooth[1]"], "acc_z": ["accsmooth[2]"],
        "throttle": ["rccommand[3]"],
        "rc_roll": ["rcdata[0]"], "rc_pitch": ["rcdata[1]"],
        "rc_yaw": ["rcdata[2]"], "rc_throttle": ["rcdata[3]"],
        "mc_pos_p_0": ["mcposaxisp[0]"], "mc_pos_p_1": ["mcposaxisp[1]"],
        "mc_pos_p_2": ["mcposaxisp[2]"],
        "mc_vel_p_0": ["mcvelaxisp[0]"], "mc_vel_p_1": ["mcvelaxisp[1]"], "mc_vel_p_2": ["mcvelaxisp[2]"],
        "mc_vel_i_0": ["mcvelaxisi[0]"], "mc_vel_i_1": ["mcvelaxisi[1]"], "mc_vel_i_2": ["mcvelaxisi[2]"],
        "mc_vel_d_0": ["mcvelaxisd[0]"], "mc_vel_d_1": ["mcvelaxisd[1]"], "mc_vel_d_2": ["mcvelaxisd[2]"],
        "mc_vel_ff_0": ["mcvelaxisff[0]"], "mc_vel_ff_1": ["mcvelaxisff[1]"], "mc_vel_ff_2": ["mcvelaxisff[2]"],
        "mc_vel_out_0": ["mcvelaxisout[0]"], "mc_vel_out_1": ["mcvelaxisout[1]"], "mc_vel_out_2": ["mcvelaxisout[2]"],
        "mc_surf_p": ["mcsurfacep"], "mc_surf_i": ["mcsurfacei"],
        "mc_surf_d": ["mcsurfaced"], "mc_surf_out": ["mcsurfaceout"],
        "nav_tgt_hdg": ["navtgthdg"],
        "vbat": ["vbat", "vbatlatest"],
        "amperage": ["amperage", "amperagelatest"],
        "rssi": ["rssi"],
    }
    found = {}
    for key, patterns in col_map.items():
        idx = flexible_col_match(headers, patterns)
        if idx is not None:
            found[key] = idx

    n_rows = len(rows)
    for key, idx in found.items():
        arr = np.zeros(n_rows, dtype=np.float64)
        for i, row in enumerate(rows):
            if idx < len(row):
                try:
                    arr[i] = float(row[idx])
                except:
                    arr[i] = np.nan
        data[key] = arr

    if "time" in data:
        time_us = data["time"]
        valid = np.diff(time_us)
        valid = valid[valid > 0]
        if len(valid) > 10:
            data["sample_rate"] = 1e6 / np.median(valid)
            data["time_s"] = (time_us - time_us[0]) / 1e6
        else:
            data["sample_rate"] = 1000.0
            data["time_s"] = np.arange(n_rows) / 1000.0
    else:
        data["sample_rate"] = 1000.0
        data["time_s"] = np.arange(n_rows) / 1000.0

    data["headers"] = headers
    data["n_rows"] = n_rows
    data["found_columns"] = list(found.keys())
    return data


# ─── Log Quality Scorer ──────────────────────────────────────────────────────

def assess_log_quality(data, config=None, filepath=None):
    """Assess whether a blackbox log is suitable for analysis.

    Returns dict with:
        usable: bool — True if the log is good enough for full analysis
        grade: str — "GOOD", "MARGINAL", "UNUSABLE"
        issues: list of {severity, check, message} dicts
        summary: str — one-line human-readable summary
        stats: dict of computed metrics
    """
    issues = []
    stats = {}

    n_rows = data.get("n_rows", 0)
    sr = data.get("sample_rate", 0)
    duration = data["time_s"][-1] if "time_s" in data and len(data.get("time_s", [])) > 0 else 0
    stats["n_rows"] = n_rows
    stats["sample_rate_hz"] = sr
    stats["duration_s"] = duration

    # ── Check 1: Duration ──
    if duration < 2.0:
        issues.append({"severity": "FAIL", "check": "duration",
                       "message": t("quality.too_short", duration=f"{duration:.1f}")})
    elif duration < 5.0:
        issues.append({"severity": "WARN", "check": "duration",
                       "message": t("quality.short", duration=f"{duration:.1f}")})

    # ── Check 2: Sample rate ──
    if sr < 50:
        issues.append({"severity": "FAIL", "check": "sample_rate",
                       "message": t("quality.low_sr_fail", sr=f"{sr:.0f}")})
    elif sr < 200:
        issues.append({"severity": "WARN", "check": "sample_rate",
                       "message": t("quality.low_sr_warn", sr=f"{sr:.0f}", nyquist=f"{sr/2:.0f}")})
    stats["nyquist_hz"] = sr / 2 if sr > 0 else 0

    # ── Check 3: Sample count ──
    min_samples_for_fft = 512
    if n_rows < min_samples_for_fft:
        issues.append({"severity": "FAIL", "check": "sample_count",
                       "message": t("quality.few_samples", n=n_rows, min=min_samples_for_fft)})

    # ── Check 4: Required fields ──
    has_gyro = any(f"gyro_{ax}" in data for ax in ["roll", "pitch", "yaw"])
    has_motors = any(f"motor{i}" in data for i in range(8))
    has_setpoint = any(f"setpoint_{ax}" in data for ax in ["roll", "pitch", "yaw"])
    stats["has_gyro"] = has_gyro
    stats["has_motors"] = has_motors
    stats["has_setpoint"] = has_setpoint
    stats["field_count"] = len(data.get("found_columns", []))

    if not has_gyro:
        issues.append({"severity": "FAIL", "check": "missing_gyro",
                       "message": t("quality.no_gyro")})
    if not has_motors:
        issues.append({"severity": "WARN", "check": "missing_motors",
                       "message": t("quality.no_motors")})
    if not has_setpoint:
        issues.append({"severity": "WARN", "check": "missing_setpoint",
                       "message": t("quality.no_setpoint")})

    # ── Check 5: Stick activity (was the pilot actually flying?) ──
    stick_active = False
    for ax in ["roll", "pitch", "yaw"]:
        sk = f"setpoint_{ax}"
        if sk in data and hasattr(data[sk], '__len__') and len(data[sk]) > 0:
            sp = data[sk]
            sp_clean = sp[~np.isnan(sp)] if hasattr(sp, '__len__') else sp
            if len(sp_clean) > 10:
                sp_std = float(np.std(sp_clean))
                if sp_std > 5.0:  # meaningful stick movement
                    stick_active = True
                    break

    throttle_active = False
    if "throttle" in data and hasattr(data["throttle"], '__len__') and len(data["throttle"]) > 0:
        thr = data["throttle"]
        thr_clean = thr[~np.isnan(thr)] if hasattr(thr, '__len__') else thr
        if len(thr_clean) > 10:
            thr_range = float(np.max(thr_clean) - np.min(thr_clean))
            stats["throttle_range"] = thr_range
            if thr_range > 50:
                throttle_active = True

    stats["stick_active"] = stick_active
    stats["throttle_active"] = throttle_active

    if not stick_active and not throttle_active:
        issues.append({"severity": "WARN", "check": "no_stick_activity",
                       "message": t("quality.no_stick")})

    # ── Check 6: All-zeros channels (dead/disconnected sensors) ──
    for ax in ["roll", "pitch", "yaw"]:
        gk = f"gyro_{ax}"
        if gk in data and hasattr(data[gk], '__len__') and len(data[gk]) > 100:
            g = data[gk]
            if np.all(g == 0) or (np.std(g) < 0.01 and np.mean(np.abs(g)) < 0.01):
                issues.append({"severity": "FAIL", "check": f"zeros_{ax}",
                               "message": t("quality.zeros_gyro", axis=ax)})

    for i in range(4):
        mk = f"motor{i}"
        if mk in data and hasattr(data[mk], '__len__') and len(data[mk]) > 100:
            m = data[mk]
            if np.all(m == 0):
                issues.append({"severity": "WARN", "check": f"motor{i}_zeros",
                               "message": t("quality.zeros_motor", n=i)})

    # ── Check 7: Corrupt frame ratio (if decoder stats available) ──
    decoder_stats = data.get("_decoder_stats", {})
    if decoder_stats:
        total = decoder_stats.get("i_frames", 0) + decoder_stats.get("p_frames", 0)
        errors = decoder_stats.get("errors", 0)
        stats["decode_errors"] = errors
        stats["decode_total"] = total
        if total > 0:
            error_pct = errors / (total + errors) * 100
            stats["error_pct"] = error_pct
            if error_pct > 20:
                issues.append({"severity": "FAIL", "check": "corrupt_frames",
                               "message": t("quality.corrupt_fail", pct=f"{error_pct:.0f}")})
            elif error_pct > 5:
                issues.append({"severity": "WARN", "check": "corrupt_frames",
                               "message": t("quality.corrupt_warn", pct=f"{error_pct:.1f}")})

    # ── Check 8: NaN ratio in gyro data ──
    for ax in ["roll", "pitch", "yaw"]:
        gk = f"gyro_{ax}"
        if gk in data and hasattr(data[gk], '__len__') and len(data[gk]) > 0:
            nan_pct = np.sum(np.isnan(data[gk])) / len(data[gk]) * 100
            if nan_pct > 50:
                issues.append({"severity": "FAIL", "check": f"nan_{ax}",
                               "message": t("quality.nan_fail", axis=ax, pct=f"{nan_pct:.0f}")})
            elif nan_pct > 10:
                issues.append({"severity": "WARN", "check": f"nan_{ax}",
                               "message": t("quality.nan_warn", axis=ax, pct=f"{nan_pct:.1f}")})
            stats[f"nan_pct_{ax}"] = nan_pct

    # ── Compute overall grade ──
    fails = sum(1 for i in issues if i["severity"] == "FAIL")
    warns = sum(1 for i in issues if i["severity"] == "WARN")
    stats["fails"] = fails
    stats["warns"] = warns

    if fails > 0:
        grade = "UNUSABLE"
        usable = False
    elif warns >= 3:
        grade = "MARGINAL"
        usable = True
    elif warns > 0:
        grade = "MARGINAL"
        usable = True
    else:
        grade = "GOOD"
        usable = True

    # One-line summary
    parts = [f"{duration:.1f}s", f"{sr:.0f}Hz", f"{n_rows} rows"]
    if not has_gyro:
        parts.append("NO GYRO")
    if not stick_active and not throttle_active:
        parts.append("ground-only?")
    if fails:
        parts.append(f"{fails} FAIL")
    if warns:
        parts.append(f"{warns} WARN")
    summary = f"[{grade}] {' | '.join(parts)}"

    return {
        "usable": usable,
        "grade": grade,
        "issues": issues,
        "summary": summary,
        "stats": stats,
    }


def print_log_quality(quality, verbose=False):
    """Print log quality assessment to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    grade = quality["grade"]
    gc = G if grade == "GOOD" else Y if grade == "MARGINAL" else RED
    grade_text = t(f"quality.{grade.lower()}")

    print(f"\n  {B}{t('quality.title')}: {gc}{grade_text}{R}")
    stats = quality["stats"]
    print(f"  {DIM}{stats['duration_s']:.1f}s | {stats['sample_rate_hz']:.0f}Hz | "
          f"{stats['n_rows']} rows | {stats['field_count']} {t('quality.fields_label').lower()}{R}")

    if quality["issues"]:
        for issue in quality["issues"]:
            sev = issue["severity"]
            sc = RED if sev == "FAIL" else Y
            sym = "✗" if sev == "FAIL" else "⚠"
            print(f"  {sc}{sym} {sev}: {issue['message']}{R}")
    else:
        print(f"  {G}✓ {t('quality.all_passed')}{R}")

    if verbose:
        gyro_sym = '✓' if stats.get('has_gyro') else '✗'
        motor_sym = '✓' if stats.get('has_motors') else '✗'
        sp_sym = '✓' if stats.get('has_setpoint') else '✗'
        stick_st = t('quality.active') if stats.get('stick_active') else t('quality.idle')
        thr_st = t('quality.active') if stats.get('throttle_active') else t('quality.idle')
        print(f"\n  {DIM}{t('quality.fields_label')}: gyro={gyro_sym} "
              f"motors={motor_sym} setpoint={sp_sym} "
              f"stick={stick_st} throttle={thr_st}{R}")
    print()


# ─── Signal Analysis ──────────────────────────────────────────────────────────

def compute_psd(arr, sr, nperseg=None):
    clean = arr[~np.isnan(arr)]
    if nperseg is None:
        nperseg = min(4096, len(clean) // 4)
    nperseg = max(256, min(nperseg, len(clean) // 2))
    freqs, psd = signal.welch(clean, fs=sr, nperseg=nperseg, window="hann", scaling="density")
    return freqs, 10 * np.log10(psd + 1e-20)


def find_noise_peaks(freqs, psd_db, n_peaks=5, min_height_db=-30, min_prominence=6):
    peaks, props = signal.find_peaks(psd_db, height=min_height_db, prominence=min_prominence, distance=10)
    if len(peaks) == 0:
        return []
    prominences = props.get("prominences", np.zeros(len(peaks)))
    order = np.argsort(prominences)[::-1][:n_peaks]
    results = [{"freq_hz": float(freqs[peaks[i]]), "power_db": float(psd_db[peaks[i]]),
                "prominence": float(prominences[i])} for i in order]
    results.sort(key=lambda x: x["freq_hz"])
    return results


def analyze_noise(data, axis_name, gyro_key, sr):
    if gyro_key not in data:
        return None
    freqs, psd_db = compute_psd(data[gyro_key], sr)
    peaks = find_noise_peaks(freqs, psd_db)
    low_band = psd_db[(freqs >= 10) & (freqs < 100)]
    mid_band = psd_db[(freqs >= 100) & (freqs < 300)]
    high_band = psd_db[(freqs >= 300)]
    window = min(20, len(psd_db) // 10)
    smoothed = np.convolve(psd_db, np.ones(max(1,window))/max(1,window), mode="same") if window > 1 else psd_db
    noise_mask = smoothed > -35
    noise_start_freq = float(freqs[np.argmax(noise_mask)]) if np.any(noise_mask) else float(sr / 2)
    return {
        "axis": axis_name, "freqs": freqs, "psd_db": psd_db, "peaks": peaks,
        "noise_start_freq": noise_start_freq,
        "rms_low": float(np.mean(low_band)) if len(low_band) > 0 else -80,
        "rms_mid": float(np.mean(mid_band)) if len(mid_band) > 0 else -80,
        "rms_high": float(np.mean(high_band)) if len(high_band) > 0 else -80,
    }


# ─── Noise Source Fingerprinting ──────────────────────────────────────────────

# Prescriptive remedies for each noise source type
_NOISE_REMEDIES = {
    "prop_harmonics": (
        "Enable RPM filter if available (most effective for prop harmonics). "
        "Otherwise, ensure dynamic_gyro_notch is enabled and covers the frequency range. "
        "Check propeller balance, condition, and tightness."
    ),
    "motor_imbalance": (
        "Check all propellers for damage, balance, and correct torque. "
        "Swap props between motors to isolate the culprit. "
        "Inspect motor shafts for bends. Soft-mount the FC if not already done."
    ),
    "structural": (
        "Tighten all frame hardware — standoffs, arm bolts, and stack screws. "
        "Check for cracked arms or loose components. "
        "Add vibration dampening (soft-mount FC, use O-rings on standoffs)."
    ),
    "electrical": (
        "Route motor wires away from the FC/gyro. "
        "Add capacitors to the power bus (low-ESR 470μF–1000μF on the ESC pads). "
        "Check for ESC desync — try increasing motor_pwm_protocol timing or switching to bidirectional DSHOT."
    ),
    "bearing_wear": (
        "Spin each motor by hand — listen and feel for roughness or grinding. "
        "Replace motors with worn bearings. "
        "Lubricate if applicable (not for sealed bearings)."
    ),
    "propwash": (
        "This is aerodynamic turbulence, not a hardware fault. "
        "Reduce D-term LPF to let D respond faster, raise D gain slightly. "
        "Fly smoother descent profiles or enable feed-forward."
    ),
    "motor_noise": (
        "Enable dynamic_gyro_notch_count = 2 and ensure min_hz covers this range. "
        "Consider enabling RPM filter for precise motor harmonic tracking. "
        "Check for ESC timing issues — try bidirectional DSHOT."
    ),
    "mechanical": (
        "Inspect all mechanical connections: camera mount, antenna mount, battery strap. "
        "Loose items vibrate at mid-frequencies. "
        "Soft-mount the FC if hard-mounted."
    ),
    "vibration": (
        "Very low frequency — check the battery mounting and overall frame rigidity. "
        "This is rarely filterable; fix the physical cause. "
        "Ensure FC mounting is secure and uses vibration dampening."
    ),
    "high_freq_noise": (
        "Likely electrical — add capacitors to the power bus. "
        "Check for gyro aliasing — ensure gyro_main_lpf_hz is below Nyquist. "
        "Route signal wires away from power/motor wires."
    ),
    "unknown": (
        "Unable to classify this noise with high confidence. "
        "Try: tighten all hardware, soft-mount FC, check prop balance, "
        "add power bus capacitors. Re-analyze after each change to isolate the cause."
    ),
}


def _noise_remedy(source, freq_hz, power_db, n_axes):
    """Get prescriptive remedy for a noise source, with severity-specific advice."""
    base = _NOISE_REMEDIES.get(source, _NOISE_REMEDIES["unknown"])

    # Add severity context
    if power_db > -5:
        severity_prefix = "CRITICAL: This noise is severe and likely causing visible oscillation. "
    elif power_db > -15:
        severity_prefix = "This is moderate noise that should be addressed. "
    else:
        severity_prefix = ""

    return severity_prefix + base


def compute_filter_recommendations(noise_results, config, profile=None):
    """Compute comprehensive filter settings from noise analysis.

    Returns a dict with:
      - gyro_lpf_hz: recommended gyro LPF cutoff
      - dterm_lpf_hz: recommended D-term LPF cutoff
      - dyn_notch_min_hz: recommended dynamic notch minimum
      - dyn_notch_max_hz: recommended dynamic notch maximum
      - dyn_notch_count: recommended number of dynamic notch filters
      - notch_peaks: list of specific peaks that would benefit from notch filtering
      - crossover_hz: the signal-to-noise crossover frequency per axis
      - reasoning: human-readable explanation of the recommendations
    """
    valid = [nr for nr in noise_results if nr is not None]
    if not valid:
        return None
    if profile is None:
        profile = get_frame_profile(5)

    n_motors = config.get("_n_motors", 4)

    # Compute LPF recommendations
    gyro_lpf = compute_recommended_filter(noise_results, 100, "gyro", profile)
    dterm_lpf = compute_recommended_filter(noise_results, 65, "dterm", profile)

    # Collect all strong peaks for notch recommendations
    all_peaks = []
    crossovers = {}
    for nr in valid:
        axis = nr["axis"]
        for p in nr["peaks"]:
            if p["power_db"] > -20 and p["prominence"] > 8:
                all_peaks.append({"axis": axis, **p})

        # Record signal-to-noise crossover per axis
        crossovers[axis] = nr.get("noise_start_freq", 250.0)

    # Sort peaks by prominence (most significant first)
    all_peaks.sort(key=lambda p: p["prominence"], reverse=True)

    # Dynamic notch range: should cover the strongest motor/prop peaks
    notch_peaks = []
    notch_freqs = []
    for p in all_peaks[:6]:  # Top 6 peaks
        freq = p["freq_hz"]
        if 50 <= freq <= 400:  # Notch-filterable range
            notch_peaks.append(p)
            notch_freqs.append(freq)

    dyn_notch_min = None
    dyn_notch_max = None
    dyn_notch_count = 1

    if notch_freqs:
        # Set range to cover all significant peaks with margin
        dyn_notch_min = max(50, int(min(notch_freqs) * 0.7))
        dyn_notch_max = min(500, int(max(notch_freqs) * 1.3))

        # Number of notches based on number of distinct frequency clusters
        # Cluster peaks within 30Hz of each other
        clusters = []
        for f in sorted(notch_freqs):
            added = False
            for c in clusters:
                if abs(f - c[-1]) < 30:
                    c.append(f)
                    added = True
                    break
            if not added:
                clusters.append([f])

        dyn_notch_count = min(len(clusters), 3)  # INAV supports up to ~3

        # Round to nearest 5
        dyn_notch_min = round(dyn_notch_min / 5) * 5
        dyn_notch_max = round(dyn_notch_max / 5) * 5

    # Build reasoning
    reasoning = []
    if gyro_lpf:
        reasoning.append(f"Gyro LPF → {gyro_lpf}Hz (signal-to-noise crossover "
                         f"at {min(crossovers.values()):.0f}Hz with safety margin)")
    if dterm_lpf:
        reasoning.append(f"D-term LPF → {dterm_lpf}Hz")
    if notch_peaks:
        peak_desc = ", ".join(f"{p['freq_hz']:.0f}Hz" for p in notch_peaks[:3])
        reasoning.append(f"Dynamic notch should cover peaks at {peak_desc}")
        if dyn_notch_count > 1:
            reasoning.append(f"{dyn_notch_count} notch filters recommended for distinct frequency clusters")

    result = {
        "gyro_lpf_hz": gyro_lpf,
        "dterm_lpf_hz": dterm_lpf,
        "notch_peaks": notch_peaks,
        "crossover_hz": crossovers,
        "reasoning": reasoning,
    }

    if dyn_notch_min is not None:
        result["dyn_notch_min_hz"] = dyn_notch_min
        result["dyn_notch_max_hz"] = dyn_notch_max
        result["dyn_notch_count"] = dyn_notch_count

    return result

def fingerprint_noise(noise_results, config, prop_harmonics=None):
    """Identify the likely source of each noise peak across all axes.

    Classifies peaks as: prop_harmonics, motor_imbalance, structural,
    electrical, bearing_wear, propwash, or unknown.

    Returns a dict with:
      - peaks: list of classified peaks (freq, power, source, confidence, detail)
      - dominant_source: the primary noise issue
      - summary: human-readable diagnosis
    """
    if not any(nr for nr in noise_results if nr is not None):
        return {"peaks": [], "dominant_source": "none", "summary": "No noise data available."}

    n_motors = config.get("_n_motors", 4)

    # Collect all significant peaks across axes with cross-axis correlation
    axis_peaks = {}  # freq_bucket -> list of (axis, peak_dict)
    for nr in noise_results:
        if nr is None:
            continue
        for p in nr["peaks"]:
            if p["power_db"] < -30:
                continue
            # Bucket to nearest 10Hz for cross-axis matching
            bucket = int(round(p["freq_hz"] / 10) * 10)
            axis_peaks.setdefault(bucket, []).append((nr["axis"], p))

    classified = []

    for bucket, axis_hits in sorted(axis_peaks.items()):
        freq = np.mean([p["freq_hz"] for _, p in axis_hits])
        power = max(p["power_db"] for _, p in axis_hits)
        prominence = max(p["prominence"] for _, p in axis_hits)
        axes_affected = list(set(a for a, _ in axis_hits))
        n_axes = len(axes_affected)

        source = "unknown"
        confidence = "low"
        detail = ""

        # 1. Match against predicted prop harmonics (most reliable when available)
        if prop_harmonics:
            for h in prop_harmonics:
                if h["min_hz"] * 0.85 <= freq <= h["max_hz"] * 1.15:
                    source = "prop_harmonics"
                    confidence = "high"
                    detail = (f"{h['label']} (predicted {h['min_hz']:.0f}-{h['max_hz']:.0f}Hz "
                              f"from motor KV and blade count)")
                    break

        # 2. Motor imbalance: low frequency, scales with motor count
        #    One-per-rev vibration at motor RPM/60, shows as broad hump
        if source == "unknown" and 15 <= freq <= 80:
            if n_axes >= 2:
                source = "motor_imbalance"
                confidence = "medium"
                detail = (f"Low-frequency vibration ({freq:.0f}Hz) on {n_axes} axes - "
                          f"consistent with prop/motor imbalance or bent shaft")
            else:
                source = "propwash"
                confidence = "medium"
                detail = (f"Low-frequency energy ({freq:.0f}Hz) on {axes_affected[0]} axis - "
                          f"likely propwash turbulence during throttle changes")

        # 3. Structural resonance: 40-150Hz, equal amplitude across all 3 axes
        if source == "unknown" and 40 <= freq <= 150 and n_axes >= 3:
            source = "structural"
            confidence = "medium"
            detail = (f"Resonance at {freq:.0f}Hz present on all axes equally - "
                      f"frame, standoff, or stack vibration")

        # 4. Electrical noise: sharp narrow peaks, often at fixed frequencies
        #    High prominence relative to power = very narrow spike
        if source == "unknown" and freq > 150 and prominence > 12:
            source = "electrical"
            confidence = "medium"
            detail = (f"Sharp spike at {freq:.0f}Hz (prominence={prominence:.0f}dB) - "
                      f"likely ESC switching noise, gyro sampling alias, or power supply ripple")

        # 5. Bearing wear: broad energy hump, moderate prominence, 100-500Hz
        if source == "unknown" and 100 <= freq <= 500 and prominence < 8 and power > -20:
            if n_axes >= 2:
                source = "bearing_wear"
                confidence = "low"
                detail = (f"Broad noise around {freq:.0f}Hz on {n_axes} axes - "
                          f"possible motor bearing wear or loose hardware")

        # 6. Frequency-range fallback classification
        if source == "unknown":
            if freq < 20:
                source = "vibration"
                detail = f"Very low frequency ({freq:.0f}Hz) - frame flex or mounting vibration"
            elif freq < 100:
                source = "mechanical"
                detail = f"Mid-low frequency ({freq:.0f}Hz) - mechanical vibration source"
            elif freq < 300:
                source = "motor_noise"
                detail = f"Motor frequency range ({freq:.0f}Hz) - motor or prop related noise"
            else:
                source = "high_freq_noise"
                detail = f"High frequency ({freq:.0f}Hz) - electrical or resonance artifact"
            confidence = "low"

        classified.append({
            "freq_hz": float(freq),
            "power_db": float(power),
            "prominence": float(prominence),
            "axes": axes_affected,
            "n_axes": n_axes,
            "source": source,
            "confidence": confidence,
            "detail": detail,
            "remedy": _noise_remedy(source, freq, power, n_axes),
        })

    # Determine dominant noise source (highest power classified peak)
    dominant = "clean"
    summary_parts = []

    if classified:
        by_power = sorted(classified, key=lambda p: p["power_db"], reverse=True)
        dominant = by_power[0]["source"]

        # Group by source for summary
        sources = {}
        for p in classified:
            sources.setdefault(p["source"], []).append(p)

        source_labels = {
            "prop_harmonics": "Propeller harmonics",
            "motor_imbalance": "Motor/prop imbalance",
            "structural": "Frame resonance",
            "electrical": "Electrical interference",
            "bearing_wear": "Motor bearing wear",
            "propwash": "Propwash",
            "motor_noise": "Motor noise",
            "mechanical": "Mechanical vibration",
            "vibration": "Low-frequency vibration",
            "high_freq_noise": "High-frequency noise",
            "unknown": "Unidentified noise",
        }

        for src, peaks in sources.items():
            label = source_labels.get(src, src)
            freqs_str = ", ".join(f"{p['freq_hz']:.0f}Hz" for p in peaks[:3])
            severity = "strong" if any(p["power_db"] > -10 for p in peaks) else \
                       "moderate" if any(p["power_db"] > -20 for p in peaks) else "mild"
            summary_parts.append(f"{label} ({severity}) at {freqs_str}")

    summary = "; ".join(summary_parts) if summary_parts else "Noise floor is clean - no dominant noise sources detected."

    return {
        "peaks": classified,
        "dominant_source": dominant,
        "summary": summary,
    }


def format_noise_fingerprint_terminal(fp, colors=None, max_show=3):
    """Format noise fingerprint results for terminal display.

    Groups peaks by source type, shows the top `max_show` most significant,
    and collapses the rest into a summary line. Deduplicates remedies.
    """
    if not fp["peaks"]:
        return ""
    R, B, C, G, Y, RED, DIM = colors or _colors()

    source_icons = {
        "prop_harmonics": "⚙", "motor_imbalance": "⚖", "structural": "🔧",
        "electrical": "⚡", "bearing_wear": "⊚", "propwash": "↻",
        "motor_noise": "⚙", "mechanical": "~", "vibration": "~",
        "high_freq_noise": "⚡", "unknown": "?",
    }

    # Group peaks by source type
    by_source = {}
    for p in fp["peaks"]:
        src = p["source"]
        by_source.setdefault(src, []).append(p)

    # Build display items: one line per source group, sorted by worst peak power
    groups = []
    for src, peaks in by_source.items():
        peaks_sorted = sorted(peaks, key=lambda x: x["power_db"], reverse=True)
        worst = peaks_sorted[0]
        freqs = sorted(set(int(p["freq_hz"]) for p in peaks_sorted))
        all_axes = sorted(set(ax for p in peaks_sorted for ax in p["axes"]))
        groups.append({
            "source": src,
            "icon": source_icons.get(src, "?"),
            "power_db": worst["power_db"],
            "freqs": freqs,
            "axes": all_axes,
            "confidence": worst["confidence"],
            "remedy": worst.get("remedy", ""),
            "count": len(peaks_sorted),
        })

    groups.sort(key=lambda x: x["power_db"], reverse=True)

    lines = []
    lines.append(f"\n  {B}NOISE SOURCES:{R}")

    shown = groups[:max_show]
    hidden = groups[max_show:]

    for g in shown:
        power_color = RED if g["power_db"] > -10 else Y if g["power_db"] > -20 else DIM
        conf = f" [{g['confidence']}]" if g["confidence"] != "high" else ""
        axes_str = "/".join(g["axes"])
        label = g["source"].replace("_", " ")

        # Show frequency list compactly
        if len(g["freqs"]) <= 3:
            freq_str = ", ".join(f"{f}Hz" for f in g["freqs"])
        else:
            freq_str = f"{g['freqs'][0]}-{g['freqs'][-1]}Hz ({g['count']} peaks)"

        lines.append(
            f"    {g['icon']} {power_color}{freq_str}{R} {DIM}{label} on {axes_str}{conf}{R}")
        if g["remedy"]:
            # Truncate long remedies to one line
            remedy = g["remedy"]
            if len(remedy) > 90:
                remedy = remedy[:87] + "..."
            lines.append(f"      {G}→ {remedy}{R}")

    if hidden:
        n_hidden = sum(g["count"] for g in hidden)
        hidden_sources = set(g["source"].replace("_", " ") for g in hidden)
        lines.append(f"    {DIM}  + {n_hidden} more peak{'s' if n_hidden > 1 else ''} "
                     f"({', '.join(hidden_sources)}){R}")

    return "\n".join(lines)


def measure_tracking_delay_xcorr(sp, gy, sr, max_delay_ms=200):
    """Measure average tracking delay using cross-correlation.
    This is robust against residual tracking error, overshoot, and noise.
    Returns delay in ms, or None if insufficient signal."""
    if len(sp) < 500:
        return None

    # High-pass filter to remove DC/drift - we only care about dynamics
    # Simple differencing acts as a high-pass filter
    sp_hp = np.diff(sp.astype(np.float64))
    gy_hp = np.diff(gy.astype(np.float64))

    # Only analyze segments with actual stick activity
    # (ignore hover/cruise where setpoint is constant)
    activity = np.abs(sp_hp)
    threshold = np.percentile(activity[activity > 0], 50) if np.any(activity > 0) else 0
    if threshold == 0:
        return None

    active_mask = activity > threshold
    if np.sum(active_mask) < 200:
        return None

    # Normalize for correlation
    sp_sig = sp_hp.copy()
    gy_sig = gy_hp.copy()
    sp_sig -= np.mean(sp_sig)
    gy_sig -= np.mean(gy_sig)
    sp_std = np.std(sp_sig)
    gy_std = np.std(gy_sig)
    if sp_std < 1e-6 or gy_std < 1e-6:
        return None

    # Cross-correlation: find lag where gyro best matches setpoint
    max_lag = int(sr * max_delay_ms / 1000)
    max_lag = min(max_lag, len(sp_sig) // 4)

    # Use segment-based approach: split into overlapping windows,
    # compute cross-correlation for each, take median
    win_size = min(int(sr * 0.5), len(sp_sig) // 4)  # 500ms windows
    if win_size < 100:
        win_size = len(sp_sig)

    step = win_size // 2
    delays = []

    for start in range(0, len(sp_sig) - win_size, step):
        sp_win = sp_sig[start:start + win_size]
        gy_win = gy_sig[start:start + win_size]

        # Skip quiet windows
        if np.std(sp_win) < sp_std * 0.3:
            continue

        # Cross-correlate: positive lag = gyro lags behind setpoint
        corr = np.correlate(gy_win, sp_win, mode='full')
        mid = len(sp_win) - 1
        # Only look at positive lags (gyro should lag behind setpoint)
        search_region = corr[mid:mid + max_lag]
        if len(search_region) < 2:
            continue

        peak_idx = np.argmax(search_region)
        delay_ms = float(peak_idx / sr * 1000)

        # Sanity: delay should be positive and reasonable
        if 0 < delay_ms < max_delay_ms:
            delays.append(delay_ms)

    if len(delays) < 3:
        return None

    return float(np.median(delays))


def detect_hover_oscillation(data, sr, profile=None):
    """Detect oscillation during hover (no/minimal stick input).

    Finds segments where setpoint is near zero, then measures gyro
    oscillation amplitude and dominant frequency. This catches the most
    dangerous tuning problem: a quad that can't hold still.

    Frame-size aware: larger frames naturally have more gyro activity,
    and low-frequency wobble (1-5Hz) outdoors is often wind buffeting,
    not P oscillation. The cause classification accounts for this.

    Returns list of per-axis dicts:
        axis, gyro_rms, gyro_p2p, dominant_freq_hz, severity, cause, hover_seconds
    Or empty list if no hover segments found.
    """
    if profile is None:
        profile = get_frame_profile(5)

    # Scale severity thresholds by frame size.
    # Larger frames have more inertia and naturally higher gyro readings.
    frame_inches = profile.get("frame_inches", 5)
    size_factor = max(1.0, frame_inches / 7.0)  # 1.0 for 7", 1.43 for 10", 2.14 for 15"

    # Severity thresholds (RMS deg/s) — scaled by frame size
    sev_none = 2.0 * size_factor     # 2.0 for 7", 2.9 for 10"
    sev_mild = 5.0 * size_factor     # 5.0 for 7", 7.1 for 10"
    sev_moderate = 15.0 * size_factor # 15.0 for 7", 21.4 for 10"

    results = []
    min_hover_samples = int(sr * 0.5)  # At least 0.5s of hover

    for axis_idx, axis in enumerate(AXIS_NAMES):
        sp_key = f"setpoint_{axis.lower()}"
        gy_key = f"gyro_{axis.lower()}"
        if sp_key not in data or gy_key not in data:
            continue

        sp = data[sp_key].copy()
        gy = data[gy_key].copy()
        mask = ~(np.isnan(sp) | np.isnan(gy))
        sp, gy = sp[mask], gy[mask]
        if len(sp) < min_hover_samples:
            continue

        # Find hover segments: setpoint near zero (stick centered)
        # Use a threshold relative to the setpoint range
        sp_range = np.percentile(np.abs(sp), 99) if len(sp) > 0 else 1
        hover_threshold = max(5.0, sp_range * 0.05)  # 5 deg/s minimum or 5% of range
        is_hover = np.abs(sp) < hover_threshold

        # Find contiguous hover segments of at least min_hover_samples
        hover_segments = []
        in_hover = False
        seg_start = 0
        for i in range(len(is_hover)):
            if is_hover[i] and not in_hover:
                seg_start = i
                in_hover = True
            elif not is_hover[i] and in_hover:
                if i - seg_start >= min_hover_samples:
                    hover_segments.append((seg_start, i))
                in_hover = False
        if in_hover and len(is_hover) - seg_start >= min_hover_samples:
            hover_segments.append((seg_start, len(is_hover)))

        if not hover_segments:
            continue

        # Concatenate all hover gyro data
        hover_gyro = np.concatenate([gy[s:e] for s, e in hover_segments])
        total_hover_seconds = len(hover_gyro) / sr

        if len(hover_gyro) < min_hover_samples:
            continue

        # Remove DC offset (mean) - we care about oscillation, not steady drift
        hover_gyro = hover_gyro - np.mean(hover_gyro)

        # Amplitude metrics
        gyro_rms = float(np.sqrt(np.mean(hover_gyro ** 2)))
        gyro_p2p = float(np.max(hover_gyro) - np.min(hover_gyro))

        # Dominant frequency via FFT
        dominant_freq = None
        peak_prominence = 0  # How much the peak stands out from noise floor
        if len(hover_gyro) >= int(sr * 0.25):  # Need at least 0.25s for FFT
            freqs = rfftfreq(len(hover_gyro), 1.0 / sr)
            spectrum = np.abs(rfft(hover_gyro))
            # Only look at 1-100 Hz (ignore DC and above Nyquist/2)
            freq_mask = (freqs >= 1) & (freqs <= 100)
            if np.any(freq_mask):
                masked_spectrum = spectrum[freq_mask]
                masked_freqs = freqs[freq_mask]
                peak_idx = np.argmax(masked_spectrum)
                dominant_freq = float(masked_freqs[peak_idx])
                peak_power = masked_spectrum[peak_idx]
                mean_power = np.mean(masked_spectrum)
                peak_prominence = peak_power / mean_power if mean_power > 0 else 0
                # Only report if peak is significantly above noise floor
                if peak_prominence < 3:
                    dominant_freq = None  # No clear dominant frequency

        # Classify severity (frame-size-scaled thresholds)
        if gyro_rms < sev_none:
            severity = "none"
        elif gyro_rms < sev_mild:
            severity = "mild"
        elif gyro_rms < sev_moderate:
            severity = "moderate"
        else:
            severity = "severe"

        # Classify cause from dominant frequency — frame-size aware
        cause = None
        if severity != "none" and dominant_freq is not None:
            if dominant_freq < 10:
                # Low-frequency wobble. On large frames (10"+), this is often
                # wind buffeting or GPS position hold corrections, NOT P oscillation.
                # True P oscillation has a sharp spectral peak (high prominence).
                # Wind buffeting is broadband (low prominence).
                if frame_inches >= 10 and peak_prominence < 6:
                    cause = "wind_buffeting"  # Likely environmental, not tuning
                elif frame_inches >= 10 and dominant_freq < 3:
                    cause = "wind_buffeting"  # <3Hz on 10"+ is almost certainly wind
                else:
                    cause = "P_too_high"
            elif dominant_freq < 25:
                cause = "PD_interaction"     # Mid-freq: P/D fighting
            elif dominant_freq < 50:
                cause = "D_noise"            # D-term amplifying noise
            else:
                cause = "filter_gap"         # High-freq noise leaking through
        elif severity != "none":
            # No dominant frequency — broadband noise, likely environmental
            if frame_inches >= 10:
                cause = "wind_buffeting"
            else:
                cause = "unknown"

        results.append({
            "axis": axis,
            "gyro_rms": gyro_rms,
            "gyro_p2p": gyro_p2p,
            "dominant_freq_hz": dominant_freq,
            "severity": severity,
            "cause": cause,
            "hover_seconds": total_hover_seconds,
            "peak_prominence": peak_prominence,
        })

    return results


def analyze_pid_response(data, axis_idx, sr):
    axis = AXIS_NAMES[axis_idx]
    sp_key, gyro_key = f"setpoint_{axis.lower()}", f"gyro_{axis.lower()}"
    if sp_key not in data or gyro_key not in data:
        return None
    sp, gy = data[sp_key].copy(), data[gyro_key].copy()
    mask = ~(np.isnan(sp) | np.isnan(gy))
    sp, gy = sp[mask], gy[mask]
    if len(sp) < 100:
        return None

    error = sp - gy
    rms_error = float(np.sqrt(np.mean(error**2)))

    # ── Delay: cross-correlation (robust, uses entire signal) ──
    avg_delay = measure_tracking_delay_xcorr(sp, gy, sr)

    # ── Overshoot: step detection ──
    sp_diff = np.abs(np.diff(sp))
    threshold = np.percentile(sp_diff[sp_diff > 0], 90) if np.any(sp_diff > 0) else 1
    steps = np.where(sp_diff > threshold)[0]
    window = int(sr * 0.1)  # 100ms window
    settle_window = int(sr * 0.01)  # 10ms pre-step settling check

    overshoots = []
    n_steps_analyzed = 0
    for idx in steps[::max(1, len(steps)//30)]:
        end = min(idx + window, len(gy) - 1)
        if end - idx < 10:
            continue
        sp_target, sp_start = sp[end], sp[idx]
        delta = sp_target - sp_start
        if abs(delta) < 1:
            continue

        # Pre-step sanity: reject if gyro is already way past setpoint in the step direction
        # (residual overshoot from prior step would contaminate this measurement)
        if idx >= settle_window:
            pre_gyro = np.mean(gy[idx-settle_window:idx])
            pre_offset = pre_gyro - sp[idx]
            # If offset is >50% of delta AND in the same direction as the step,
            # the gyro is already flying past where we need to measure
            if abs(pre_offset) > abs(delta) * 0.5 and np.sign(pre_offset) == np.sign(delta):
                continue  # skip: gyro already headed past target

        n_steps_analyzed += 1
        response = gy[idx:end]

        # ── Overshoot: peak beyond target ──
        peak = np.max(response) if delta > 0 else np.min(response)
        os_pct = ((peak - sp_target) / delta * 100) if delta > 0 else ((sp_target - peak) / (-delta) * 100)
        if 0 < os_pct < 200:
            overshoots.append(os_pct)

    avg_overshoot = float(np.median(overshoots)) if overshoots else None

    pid_stats = {}
    for name, suffix in [("P", "P"), ("I", "I"), ("D", "D")]:
        key = f"axis{suffix}_{axis.lower()}"
        if key in data:
            arr = data[key][mask]
            arr = arr[~np.isnan(arr)]
            if len(arr) > 0:
                pid_stats[name] = {"rms": float(np.sqrt(np.mean(arr**2))), "max": float(np.max(np.abs(arr)))}

    return {"axis": axis, "rms_error": rms_error, "tracking_delay_ms": avg_delay,
            "avg_overshoot_pct": avg_overshoot, "n_steps": n_steps_analyzed,
            "setpoint": sp, "gyro": gy, "pid_stats": pid_stats}


def analyze_motors(data, sr, config=None):
    motors = []
    for i in range(4):
        key = f"motor{i}"
        if key in data:
            motors.append(data[key])
    if not motors:
        return None

    global_max = np.nanmax(np.array(motors))
    global_min = np.nanmin(np.array(motors))

    # Use header-provided motor output range if available (most accurate)
    motor_lo = config.get("motor_output_low") if config else None
    motor_hi = config.get("motor_output_high") if config else None

    if motor_lo is not None and motor_hi is not None and motor_hi > motor_lo:
        motor_min, motor_max = motor_lo, motor_hi
    elif global_max > 1500:
        motor_min, motor_max = 1000, 2000
    elif global_max > 100:
        # Ambiguous range - try to infer from minthrottle
        minthrottle = config.get("minthrottle") if config else None
        if minthrottle and minthrottle > 900:
            motor_min, motor_max = minthrottle, 2000
        else:
            motor_min, motor_max = 0, 1000
    else:
        motor_min, motor_max = 0, 100
    span = max(1, motor_max - motor_min)

    results = []
    for i, m in enumerate(motors):
        clean = m[~np.isnan(m)]
        if len(clean) == 0:
            continue
        pct = np.clip((clean - motor_min) / span * 100, 0, 100)
        std_raw = float(np.std(clean))
        results.append({
            "motor": i + 1, "avg_pct": float(np.mean(pct)), "std_pct": float(np.std(pct)),
            "saturation_pct": float(np.sum(pct > 95) / len(pct) * 100),
            "min_pct": float(np.min(pct)), "normalized": pct, "raw": clean,
        })

    # Detect idle/ground: motors at or near minthrottle with no meaningful variation
    # This happens during arm-on-ground, pre-takeoff, or failed arm attempts
    all_stds = [r["std_pct"] for r in results]
    all_avgs = [r["avg_pct"] for r in results]
    idle_detected = (len(all_stds) > 0 and max(all_stds) < 3.0 and
                     len(all_avgs) > 0 and max(all_avgs) < 15.0)

    avgs = [r["avg_pct"] for r in results]
    spread = max(avgs) - min(avgs) if len(avgs) >= 2 else 0
    worst_motor, worst_direction = None, None
    if spread > MOTOR_IMBAL_OK and len(avgs) >= 4:
        mean_avg = np.mean(avgs)
        deviations = [abs(a - mean_avg) for a in avgs]
        worst_motor = int(np.argmax(deviations)) + 1
        worst_direction = "high" if avgs[worst_motor - 1] > mean_avg else "low"

    return {"n_motors": len(results), "motors": results, "balance_spread_pct": spread,
            "motor_range": (motor_min, motor_max),
            "worst_motor": worst_motor, "worst_direction": worst_direction,
            "idle_detected": idle_detected}


def analyze_dterm_noise(data, sr):
    results = []
    for axis in AXIS_NAMES:
        d_key = f"axisD_{axis.lower()}"
        if d_key not in data:
            continue
        clean = data[d_key][~np.isnan(data[d_key])]
        if len(clean) < 256:
            continue
        freqs, psd_db = compute_psd(clean, sr)
        peaks = find_noise_peaks(freqs, psd_db, min_height_db=-40)
        results.append({"axis": axis, "freqs": freqs, "psd_db": psd_db,
                         "peaks": peaks, "rms": float(np.sqrt(np.mean(clean**2)))})
    return results


# ─── Accelerometer Vibration Analysis ─────────────────────────────────────────

_ACCEL_AXIS_MAP = {"X": "acc_x", "Y": "acc_y", "Z": "acc_z"}

def analyze_accel_vibration(data, sr, prop_harmonics=None):
    """Analyze accelerometer data for structural vibration signatures.

    Accelerometer FFT catches frame resonances, loose hardware, and
    prop/motor issues that gyro filtering may mask. The accel sees
    physical vibration directly, while gyro sees rotational effects.

    Args:
        data: Decoded flight data dict
        sr: Sample rate in Hz
        prop_harmonics: Optional prop harmonic predictions from RPM estimate

    Returns dict with:
        axes (list): Per-axis results [{axis, freqs, psd_db, peaks, rms_g, findings}]
        overall_rms_g (float): Combined RMS across all axes
        score (int): 0-100 vibration health score
        findings (list): [{level, text, detail}]
    """
    results = {
        "axes": [],
        "overall_rms_g": None,
        "score": 100,
        "findings": [],
    }

    has_accel = all(k in data for k in ["acc_x", "acc_y", "acc_z"])
    if not has_accel:
        return results

    all_rms = []
    score = 100

    for axis_name, key in _ACCEL_AXIS_MAP.items():
        raw = data[key]
        clean = raw[~np.isnan(raw)]
        if len(clean) < 512:
            continue

        # Convert to g (INAV accel is in cm/s², 1g = 981 cm/s²)
        # Remove DC offset (gravity) — we only care about vibration
        clean_g = clean / 981.0
        clean_g = clean_g - np.mean(clean_g)

        rms_g = float(np.sqrt(np.mean(clean_g**2)))
        all_rms.append(rms_g)

        # FFT
        freqs, psd_db = compute_psd(clean_g, sr)
        peaks = find_noise_peaks(freqs, psd_db, min_height_db=-30)
        # Filter out DC/sub-5Hz (gravity residual, not real vibration)
        peaks = [pk for pk in peaks if pk["freq_hz"] >= 5]

        axis_findings = []

        # Classify peaks
        for pk in peaks:
            freq = pk["freq_hz"]
            power = pk["power_db"]

            # Match against prop harmonics if available
            matched_harmonic = None
            if prop_harmonics:
                for h in prop_harmonics:
                    if h["min_hz"] <= freq <= h["max_hz"]:
                        matched_harmonic = h["label"]
                        break

            if matched_harmonic:
                axis_findings.append({
                    "level": "INFO",
                    "text": f"{axis_name}: {freq:.0f}Hz matches {matched_harmonic}",
                    "detail": "Prop/motor vibration at expected harmonic. "
                              "Check prop balance and tightness.",
                    "source": "prop_harmonic",
                })
            elif freq < 50:
                axis_findings.append({
                    "level": "WARNING",
                    "text": f"{axis_name}: Low-frequency vibration at {freq:.0f}Hz ({power:.0f}dB)",
                    "detail": "Sub-50Hz accel vibration suggests loose mounting, "
                              "frame flex, or a failing bearing.",
                    "source": "structural_low",
                })
            elif 50 <= freq <= 200:
                axis_findings.append({
                    "level": "INFO",
                    "text": f"{axis_name}: Mid-frequency vibration at {freq:.0f}Hz ({power:.0f}dB)",
                    "detail": "Likely motor/prop related. Check prop balance, "
                              "motor shaft straightness, and bell bearing play.",
                    "source": "motor_prop",
                })
            elif freq > 200:
                axis_findings.append({
                    "level": "INFO",
                    "text": f"{axis_name}: High-frequency vibration at {freq:.0f}Hz ({power:.0f}dB)",
                    "detail": "Electrical noise or high-order resonance. "
                              "Check motor wire routing and ESC shielding.",
                    "source": "electrical_hf",
                })

        # RMS severity per axis
        if rms_g > 0.5:
            score -= 15
            axis_findings.append({
                "level": "WARNING",
                "text": f"{axis_name}: High vibration ({rms_g:.2f}g RMS)",
                "detail": "Excessive vibration — check props, motors, frame hardware.",
                "source": "rms_high",
            })
        elif rms_g > 0.2:
            score -= 5
            axis_findings.append({
                "level": "INFO",
                "text": f"{axis_name}: Moderate vibration ({rms_g:.2f}g RMS)",
                "detail": "Some vibration present. Monitor for degradation.",
                "source": "rms_moderate",
            })

        results["axes"].append({
            "axis": axis_name,
            "freqs": freqs,
            "psd_db": psd_db,
            "peaks": peaks,
            "rms_g": rms_g,
            "findings": axis_findings,
        })

    if all_rms:
        results["overall_rms_g"] = float(np.sqrt(np.mean(np.array(all_rms)**2)))

    # Cross-axis comparison: Z should be highest (gravity axis), X/Y similar
    axis_rms = {a["axis"]: a["rms_g"] for a in results["axes"]}
    if "X" in axis_rms and "Y" in axis_rms:
        xy_ratio = max(axis_rms["X"], axis_rms["Y"]) / max(min(axis_rms["X"], axis_rms["Y"]), 0.001)
        if xy_ratio > 3.0:
            worse_axis = "X" if axis_rms["X"] > axis_rms["Y"] else "Y"
            results["findings"].append({
                "level": "WARNING",
                "text": f"{worse_axis}-axis vibration {xy_ratio:.1f}× higher than the other — "
                        f"asymmetric issue (prop, motor, or arm on that side)",
                "detail": f"X={axis_rms.get('X', 0):.2f}g, Y={axis_rms.get('Y', 0):.2f}g. "
                          f"Swap props between {worse_axis}-axis motors to isolate.",
            })

    # Aggregate axis findings into top-level
    for ax in results["axes"]:
        for f in ax["findings"]:
            if f["level"] == "WARNING":
                results["findings"].append(f)

    results["score"] = max(0, score)
    return results


def _print_section_vibration(vib_results):
    """Print accelerometer vibration section to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}STRUCTURAL VIBRATION (Accelerometer):{R}")

    if not vib_results or not vib_results.get("axes"):
        print(f"  {DIM}  No accelerometer data available.{R}")
        return

    score = vib_results.get("score", 100)
    sc = G if score >= 85 else Y if score >= 60 else RED
    overall = vib_results.get("overall_rms_g", 0)
    print(f"  {sc}  Vibration score: {score}/100  (overall {overall:.2f}g RMS){R}")

    for ax in vib_results["axes"]:
        rms = ax["rms_g"]
        rc = G if rms < 0.1 else Y if rms < 0.3 else RED
        n_peaks = len(ax["peaks"])
        print(f"    {ax['axis']}-axis: {rc}{rms:.3f}g{R} RMS  ({n_peaks} peaks)")

    for f in vib_results.get("findings", []):
        if f["level"] == "WARNING":
            print(f"\n  {Y}⚠ {f['text']}{R}")
            if f.get("detail"):
                print(f"    {DIM}{f['detail']}{R}")
        elif f["level"] == "INFO":
            print(f"\n  {DIM}ℹ {f['text']}{R}")


def _print_section_recipe(recipe):
    """Print tuning recipe section to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}TUNING RECIPE: {recipe['recipe_name']}{R}")
    print(f"  {DIM}{recipe['description']}{R}")

    print(f"\n  {B}Filter stack:{R}")
    for param, val in recipe["stack"].items():
        print(f"    {G}set {param} = {val}{R}")

    print(f"\n  {B}Reasoning:{R}")
    for r in recipe["reasoning"]:
        print(f"    {DIM}• {r}{R}")


def _print_section_power(power):
    """Print power/efficiency section to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}POWER & BATTERY:{R}")

    if not power or power.get("avg_cell_v") is None:
        print(f"  {DIM}  No battery voltage data available.{R}")
        return

    vc = G if power["min_cell_v"] > 3.5 else Y if power["min_cell_v"] > 3.3 else RED
    print(f"    Cell voltage: avg {power['avg_cell_v']:.2f}V  min {vc}{power['min_cell_v']:.2f}V{R}  sag {power['sag_v']:.2f}V")

    if power.get("avg_current_a") is not None:
        print(f"    Current: avg {power['avg_current_a']:.1f}A  Power: {power['avg_power_w']:.0f}W")
        print(f"    Consumed: {power['total_mah']:.0f}mAh ({power['total_wh']:.1f}Wh)")

    for f in power.get("findings", []):
        icon = f"{Y}⚠" if f["level"] == "WARNING" else f"{DIM}ℹ"
        print(f"\n  {icon} {f['text']}{R}")
        if f.get("detail"):
            print(f"    {DIM}{f['detail']}{R}")


def _print_section_propwash(propwash):
    """Print propwash scoring section to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}PROPWASH:{R}")

    if not propwash or not propwash.get("events"):
        for f in propwash.get("findings", []) if propwash else []:
            print(f"  {G}✓ {f['text']}{R}")
        return

    score = propwash.get("score", 100)
    sc = G if score >= 85 else Y if score >= 60 else RED
    worst = propwash.get("worst_axis", "?")
    n_events = len(propwash["events"])
    print(f"  {sc}  Propwash score: {score}/100  ({n_events} events, worst on {worst}){R}")

    # Event summary by severity
    for sev in ["severe", "moderate", "mild"]:
        evts = [e for e in propwash["events"] if e["severity"] == sev]
        if evts:
            avg_rms = np.mean([e["rms"] for e in evts])
            avg_freq = np.mean([e["freq_hz"] for e in evts])
            print(f"    {sev.capitalize()}: {len(evts)} events, {avg_rms:.1f}°/s RMS at {avg_freq:.0f}Hz")

    for f in propwash.get("findings", []):
        icon = f"{Y}⚠" if f["level"] == "WARNING" else f"{DIM}ℹ" if f["level"] == "INFO" else f"{G}✓"
        print(f"\n  {icon} {f['text']}{R}")
        if f.get("detail"):
            print(f"    {DIM}{f['detail']}{R}")


# ─── Prescriptive Recommendation Engine ──────────────────────────────────────

def compute_recommended_filter(noise_results, current_hz, filter_type="gyro", profile=None):
    """Compute optimal LPF cutoff from actual noise spectrum.

    Strategy:
    1. Find the signal-to-noise crossover: where the smoothed PSD rises above
       the "noise floor threshold" (signal band ends, noise band begins).
    2. Apply a safety margin below that crossover so the filter rolls off
       before noise starts.
    3. Avoid placing the cutoff on or near a noise peak (would amplify it).
    4. Clamp to the frame profile's min/max range.
    5. CHECK PHASE LAG: if the recommended cutoff would add excessive phase lag
       compared to current, don't recommend lowering. Large quads (10"+) are
       especially sensitive to phase lag — lowering LPF from 65 to 30Hz can
       make oscillations worse even though noise numbers improve.
    6. If noise is well above the current LPF (all peaks >2x current cutoff),
       the LPF can't meaningfully help — recommend notch/RPM filters instead.

    Returns the recommended Hz value (int, rounded to 5), or None.
    """
    valid = [nr for nr in noise_results if nr is not None]
    if not valid:
        return None
    if profile is None:
        profile = get_frame_profile(5)

    if filter_type == "gyro":
        min_cutoff, max_cutoff = profile["gyro_lpf_range"]
        noise_threshold_db = -30  # dB above which we consider it "noise"
    else:
        min_cutoff, max_cutoff = profile["dterm_lpf_range"]
        noise_threshold_db = -35  # D-term is more sensitive

    safety = profile["filter_safety"]
    crossover_freqs = []

    for nr in valid:
        freqs = nr["freqs"]
        psd_db = nr["psd_db"]

        # Smooth the spectrum to find the trend (not individual peaks)
        window = max(1, min(30, len(psd_db) // 10))
        smoothed = np.convolve(psd_db, np.ones(window) / window, mode="same")

        # Find where smoothed PSD first rises above threshold in the noise band
        # Start searching above 20Hz (below is just DC/drift)
        search_mask = freqs >= 20
        search_freqs = freqs[search_mask]
        search_psd = smoothed[search_mask]

        # Find the crossover: first frequency where PSD > threshold
        above = search_psd > noise_threshold_db
        if np.any(above):
            crossover_idx = np.argmax(above)
            crossover_freqs.append(float(search_freqs[crossover_idx]))
        else:
            # Spectrum is clean — noise never rises above threshold
            crossover_freqs.append(float(freqs[-1]))

    if not crossover_freqs:
        return None

    # Use the worst (lowest) crossover across all axes
    worst_crossover = min(crossover_freqs)

    # Apply safety margin: place cutoff below the crossover
    ideal = worst_crossover * safety

    # Avoid placing cutoff near a noise peak (within +/-15Hz of any strong peak)
    all_peaks = []
    for nr in valid:
        for p in nr["peaks"]:
            if p["power_db"] > -20:  # Only avoid strong peaks
                all_peaks.append(p["freq_hz"])

    # If ideal lands near a peak, push it lower
    for peak_freq in sorted(all_peaks):
        if abs(ideal - peak_freq) < 15:
            ideal = min(ideal, peak_freq - 20)

    # ── Phase lag guard ──
    # If lowering the filter would significantly increase phase lag, don't.
    # This is critical for large frames where phase lag causes more problems
    # than the noise the filter would remove.
    if current_hz is not None and ideal < current_hz:
        signal_freq = profile.get("phase_lag_freq", 50.0)  # typical control freq

        current_lag = estimate_filter_phase_lag(current_hz, signal_freq)
        proposed_lag = estimate_filter_phase_lag(ideal, signal_freq)

        if current_lag and proposed_lag:
            added_lag_ms = proposed_lag["ms"] - current_lag["ms"]
            added_lag_deg = proposed_lag["degrees"] - current_lag["degrees"]

            # For 10"+ frames, even 5ms extra lag is significant
            frame_size = profile.get("frame_inches", 5)
            max_added_lag_ms = 8.0 if frame_size <= 7 else 5.0 if frame_size <= 10 else 3.0

            if added_lag_ms > max_added_lag_ms:
                # Lowering would add too much phase lag — don't recommend
                return None

    # ── Relative change guard ──
    # On large frames, aggressive LPF cuts cause more harm than good.
    # The added phase lag and signal attenuation destabilize the PID loop.
    # Don't recommend cutting the LPF by more than 30% on 10"+.
    if current_hz is not None and ideal < current_hz:
        frame_size = profile.get("frame_inches", 5)
        if frame_size >= 10:
            max_reduction = 0.30  # 30% max cut for 10"+
        elif frame_size >= 7:
            max_reduction = 0.40  # 40% for 7"
        else:
            max_reduction = 0.50  # 50% for 5"

        min_allowed = current_hz * (1.0 - max_reduction)
        if ideal < min_allowed:
            ideal = min_allowed  # Cap the reduction

    # ── Propwash guard ──
    # If the lowest significant noise is below ~80Hz, it's likely propwash
    # (aerodynamic turbulence). LPF changes don't effectively address propwash
    # because it's broadband — you'd have to crush the filter so low that
    # the phase lag makes everything worse. Propwash is fixed through D-term
    # tuning and flight technique, not gyro LPF.
    if current_hz is not None and all_peaks and filter_type == "gyro":
        lowest_peak = min(all_peaks)
        frame_size = profile.get("frame_inches", 5)
        # For large frames, propwash typically sits at 30-80Hz
        propwash_ceiling = 80 if frame_size >= 10 else 100
        if lowest_peak <= propwash_ceiling and ideal < current_hz:
            # The noise driving the recommendation is in the propwash band.
            # Don't lower the LPF — it won't help and will add lag.
            # Only recommend if there are also peaks well above propwash.
            peaks_above_propwash = [p for p in all_peaks if p > propwash_ceiling * 1.5]
            if not peaks_above_propwash:
                return None
            # There are higher peaks too — but don't let propwash drive the cutoff down
            ideal = max(ideal, current_hz * 0.85)  # at most 15% cut

    # ── High-frequency noise guard ──
    # If all significant noise is well above the current LPF (>2x cutoff),
    # lowering the LPF won't meaningfully help — the noise needs notch/RPM
    # filters, not a lower LPF that would crush phase margin.
    if current_hz is not None and all_peaks:
        lowest_significant_peak = min(all_peaks)
        if lowest_significant_peak > current_hz * 2.0:
            # All noise is far above current LPF — LPF change won't help
            return None

    # Clamp and round
    result = round(int(np.clip(ideal, min_cutoff, max_cutoff)) / 5) * 5
    return result


def compute_recommended_pid(pid_result, current_p, current_i, current_d, profile=None,
                            current_ff=None):
    if pid_result is None:
        return None
    if profile is None:
        profile = get_frame_profile(5)
    axis = pid_result["axis"]
    changes, reasons = {}, []
    max_change = profile["pid_adjust_factor"]  # max fraction change per iteration
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]

    os_pct = pid_result.get("avg_overshoot_pct") or 0.0
    delay = pid_result.get("tracking_delay_ms") or 0.0
    os_high = os_pct > ok_os
    os_bad = os_pct > bad_os
    dl_high = delay > ok_dl
    dl_bad = delay > bad_dl

    # ── FF-aware overshoot attribution ──
    # FF drives initial response proportional to stick speed. High FF + high P
    # = double overshoot source. When FF is significant (>40), attribute some
    # overshoot to FF and reduce it before cutting P aggressively.
    ff_is_high = current_ff is not None and current_ff > 40
    ff_contributes_overshoot = ff_is_high and os_high

    if ff_contributes_overshoot:
        # Split the blame: FF handles stick-driven overshoot, P handles error-driven.
        # With high FF, reduce FF first, smaller P cut.
        ff_severity = current_ff / 60.0  # 1.0 at FF=60, higher = more blame on FF
        ff_share = min(0.6, ff_severity * 0.3)  # FF takes up to 60% of the blame

        new_ff = max(15, int(current_ff * (1 - min(0.25, ff_share * 0.4))))
        if new_ff != current_ff:
            changes["FF"] = {"current": current_ff, "new": new_ff}
            reasons.append(
                f"FeedForward is {current_ff} - contributes to overshoot on stick inputs. "
                f"Reducing FF from {current_ff} to {new_ff} before cutting P.")

    # ── Severity-proportional P adjustment ──
    # How far off from target determines size of correction.
    # Profile's max_change caps the adjustment for safety.
    if current_p is not None:
        new_p = current_p

        if os_high:
            # severity: 1.0 = at ok threshold, 2.0 = 2x the ok threshold, etc.
            severity = os_pct / ok_os
            if severity > 3.0:
                raw_cut = 0.35
            elif severity > 2.0:
                raw_cut = 0.25
            elif severity > 1.5:
                raw_cut = 0.18
            else:
                raw_cut = 0.10

            # If FF is taking some blame, reduce the P cut proportionally
            if ff_contributes_overshoot:
                ff_share_actual = min(0.6, (current_ff / 60.0) * 0.3)
                raw_cut *= (1.0 - ff_share_actual)

            has_d = current_d is not None and current_d > 0
            d_hint = " and raise D" if has_d else ""
            if os_high and dl_high:
                # Conflict: overshoot says lower P, delay says raise P.
                # Overshoot wins, but be a bit less aggressive since delay is also a concern.
                raw_cut *= 0.7
                settle = f", higher D helps it settle" if has_d else ""
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%) and delay is {delay:.0f}ms. "
                    f"Fixing overshoot first - lower P reduces the bounce{settle}.")
            elif os_bad:
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%). "
                    f"The quad swings past the target angle. Reduce P{d_hint} to fix this.")
            else:
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%). "
                    f"A small P reduction{d_hint} should tighten this up.")

            # max_change limits P increases (delay fix), but overshoot reduction
            # can exceed it - the quad is ALREADY oscillating, we need meaningful correction.
            p_cut = min(raw_cut, max(max_change, 0.20))  # at least 20% reduction allowed
            new_p = int(current_p * (1 - p_cut))

        elif dl_high:
            severity = delay / ok_dl
            if severity > 2.0:
                raw_bump = 0.25
            elif severity > 1.5:
                raw_bump = 0.15
            else:
                raw_bump = 0.08

            if dl_bad:
                reasons.append(f"Response is sluggish ({delay:.0f}ms delay, target: <{ok_dl}ms). Needs more P.")
            else:
                reasons.append(f"Delay is {delay:.0f}ms (target: <{ok_dl}ms). A small P bump will help.")

            p_bump = min(raw_bump, max_change)
            new_p = int(current_p * (1 + p_bump))

        new_p = max(15, min(200, new_p))
        if new_p != current_p:
            changes["P"] = {"current": current_p, "new": new_p}

    # ── D adjustment: proportional to overshoot severity ──
    if current_d is not None and current_d > 0:
        new_d = current_d
        if os_high:
            severity = os_pct / ok_os
            if severity > 2.5:
                raw_bump = 0.30
            elif severity > 1.5:
                raw_bump = 0.20
            else:
                raw_bump = 0.12
            d_bump = min(raw_bump, max_change * 1.5)  # D can move a bit more than P
            new_d = int(current_d * (1 + d_bump))
        new_d = max(0, min(150, new_d))
        if new_d != current_d:
            changes["D"] = {"current": current_d, "new": new_d}

    # ── I adjustment ──
    if current_i is not None:
        new_i = current_i
        if "I" in pid_result["pid_stats"] and pid_result["pid_stats"]["I"]["rms"] > 80:
            new_i = int(current_i * 0.85)
            reasons.append(f"I-term working very hard (possible CG offset)")
        new_i = max(5, min(200, new_i))
        if new_i != current_i:
            changes["I"] = {"current": current_i, "new": new_i}

    return {"axis": axis, "changes": changes, "reasons": reasons}


# ─── Navigation Analysis ────────────────────────────────────────────────────

# INAV flight mode bitmask values (from runtime_config.h)
FMODE_ANGLE    = 1 << 0
FMODE_HORIZON  = 1 << 1
FMODE_HEADING  = 1 << 2
FMODE_ALTHOLD  = 1 << 3
FMODE_RTH      = 1 << 4
FMODE_POSHOLD  = 1 << 5
FMODE_HEADFREE = 1 << 6
FMODE_LAUNCH   = 1 << 7
FMODE_MANUAL   = 1 << 8
FMODE_FAILSAFE = 1 << 9
FMODE_WP       = 1 << 10
FMODE_CRUISE   = 1 << 11


def detect_nav_fields(data):
    """Detect which navigation fields are available in the data."""
    avail = {}
    avail["has_pos"] = "nav_pos_n" in data and "nav_pos_e" in data and "nav_pos_u" in data
    avail["has_vel"] = "nav_vel_n" in data and "nav_vel_e" in data and "nav_vel_u" in data
    avail["has_tgt"] = "nav_tgt_n" in data and "nav_tgt_e" in data and "nav_tgt_u" in data
    avail["has_att"] = "att_roll" in data and "att_pitch" in data and "att_heading" in data
    avail["has_baro"] = "baro_alt" in data
    avail["has_acc"] = "acc_x" in data and "acc_y" in data and "acc_z" in data
    avail["has_nav_state"] = "nav_state" in data
    avail["has_eph"] = "nav_eph" in data
    avail["has_throttle"] = "throttle" in data or "motor0" in data
    avail["has_gps_frames"] = len(data.get("_gps_frames", [])) > 0
    avail["has_slow_frames"] = len(data.get("_slow_frames", [])) > 0
    avail["has_any"] = any(v for k, v in avail.items() if k.startswith("has_"))
    return avail


def segment_flight_phases(data, sr):
    """Segment flight into phases using navState from I/P frames.

    Returns list of (start_idx, end_idx, phase_name) tuples.
    Minimum phase duration: 2 seconds.
    """
    if "nav_state" not in data:
        return []

    ns = data["nav_state"]
    valid = ~np.isnan(ns)
    if np.sum(valid) < 100:
        return []

    # navState values - group into broad categories
    # The exact values vary by INAV version, but the pattern is consistent:
    # 0-1: IDLE, 2-9: various ALTHOLD states, 10-19: POSHOLD states,
    # 20-29: RTH states, 30-39: WP states, 40+: LANDING/EMERGENCY
    # Rather than hard-code all values, detect transitions and label by context.
    # Simple approach: use the raw value and detect contiguous regions.

    phases = []
    min_samples = int(sr * 2)  # 2 second minimum
    state_arr = ns.copy()
    state_arr[~valid] = -1

    # Detect contiguous regions of same navState
    i = 0
    n = len(state_arr)
    while i < n:
        if state_arr[i] < 0:
            i += 1
            continue
        current_state = int(state_arr[i])
        start = i
        while i < n and (state_arr[i] == current_state or state_arr[i] < 0):
            i += 1
        end = i
        if end - start >= min_samples and current_state > 0:
            phases.append((start, end, current_state))

    return phases


def analyze_compass_health(data, sr):
    """Analyze compass/magnetometer health from heading data.

    Detects:
    - Heading noise (jitter in steady flight)
    - Motor/throttle EMI correlation
    - Heading drift rate

    Works on ANY flight mode - doesn't need poshold.
    """
    results = {
        "score": None,
        "heading_jitter_deg": None,
        "throttle_correlation": None,
        "heading_drift_dps": None,
        "findings": []
    }

    if "att_heading" not in data:
        return results

    heading = data["att_heading"]
    valid = ~np.isnan(heading)
    if np.sum(valid) < sr * 5:  # need at least 5 seconds
        return results

    hdg = heading[valid] / 10.0  # decidegrees to degrees

    # ─── Heading jitter: std dev of heading derivative ───
    # Compass updates at ~75Hz but log rate can be 1000Hz.
    # Downsample to ~50Hz to avoid quantization noise from identical samples.
    ds_factor = max(1, int(sr / 50))
    hdg_ds = hdg[::ds_factor]
    sr_ds = sr / ds_factor

    # Unwrap heading to handle 0/360 wraparound
    hdg_unwrap = np.unwrap(np.deg2rad(hdg_ds))
    hdg_rate = np.diff(hdg_unwrap) * sr_ds  # rad/s
    hdg_rate_deg = np.rad2deg(hdg_rate)

    # Filter out large intentional turns (>30 deg/s)
    steady_mask = np.abs(hdg_rate_deg) < 30
    if np.sum(steady_mask) < sr_ds * 2:
        return results

    jitter = float(np.std(hdg_rate_deg[steady_mask]))
    results["heading_jitter_deg"] = round(jitter, 2)

    # ─── Motor correlation: does heading jump when throttle changes? ───
    throttle = None
    if "throttle" in data:
        throttle = data["throttle"][valid][::ds_factor]
    elif "motor0" in data:
        throttle = data["motor0"][valid][::ds_factor]

    if throttle is not None and len(throttle) > 100:
        # Compute throttle rate of change
        thr_rate = np.diff(throttle)
        # Align lengths
        min_len = min(len(hdg_rate_deg), len(thr_rate))
        if min_len > 100:
            h = hdg_rate_deg[:min_len]
            t = thr_rate[:min_len]
            # Absolute correlation - we care about magnitude, not direction
            # Use sliding windows to catch delayed correlation
            try:
                corr = abs(float(np.corrcoef(np.abs(h), np.abs(t))[0, 1]))
                if not np.isnan(corr):
                    results["throttle_correlation"] = round(corr, 3)
            except (ValueError, FloatingPointError):
                pass

    # ─── Heading drift: average rate over the whole flight ───
    duration_s = len(hdg_ds) / sr_ds
    if duration_s > 10:
        total_drift = hdg_unwrap[-1] - hdg_unwrap[0]
        drift_dps = float(np.rad2deg(total_drift) / duration_s)
        results["heading_drift_dps"] = round(drift_dps, 3)

    # ─── Scoring ───
    score = 100
    findings = []

    if jitter > 5.0:
        score -= 40
        findings.append(("WARNING", f"High heading jitter: {jitter:.1f} deg/s RMS "
                         "(expect <2 deg/s in calm hover, check compass mounting)"))
    elif jitter > 2.0:
        score -= 15
        findings.append(("INFO", f"Moderate heading jitter: {jitter:.1f} deg/s RMS"))

    corr = results["throttle_correlation"]
    if corr is not None and corr > 0.4:
        score -= 30
        findings.append(("WARNING", f"Heading correlates with throttle (r={corr:.2f}) - "
                         "likely motor EMI on compass. Twist power leads, "
                         "increase compass distance from motors/PDB"))
    elif corr is not None and corr > 0.2:
        score -= 10
        findings.append(("INFO", f"Mild throttle-heading correlation (r={corr:.2f})"))

    drift = results["heading_drift_dps"]
    if drift is not None and abs(drift) > 1.0:
        score -= 15
        findings.append(("INFO", f"Heading drift: {drift:.2f} deg/s "
                         "(may indicate compass calibration needed)"))

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


def analyze_gps_quality(data, sr):
    """Analyze GPS quality from nav estimation data and GPS frames.

    Detects:
    - EPH/EPV quality (position error estimates)
    - Position jumps (sudden GPS glitches)
    - GPS satellite count (from G-frames)
    - GPS update rate

    Works on ANY flight mode.
    """
    results = {
        "score": None,
        "avg_eph": None,
        "max_eph": None,
        "avg_sats": None,
        "min_sats": None,
        "position_jumps": 0,
        "gps_rate_hz": None,
        "findings": []
    }

    findings = []
    score = 100
    has_data = False

    # ─── EPH/EPV from I-frame nav data ───
    if "nav_eph" in data:
        eph = data["nav_eph"]
        valid_eph = eph[~np.isnan(eph)]
        valid_eph = valid_eph[valid_eph > 0]  # filter zeros (no fix)
        if len(valid_eph) > 10:
            has_data = True
            avg_eph = float(np.mean(valid_eph))
            max_eph = float(np.max(valid_eph))
            results["avg_eph"] = round(avg_eph, 1)
            results["max_eph"] = round(max_eph, 1)

            if avg_eph > 500:
                score -= 40
                findings.append(("WARNING", f"Poor GPS accuracy: EPH avg {avg_eph:.0f}cm "
                                 "(need <300cm for reliable nav)"))
            elif avg_eph > 300:
                score -= 15
                findings.append(("INFO", f"Marginal GPS accuracy: EPH avg {avg_eph:.0f}cm"))

    # ─── Position jumps from navPos ───
    if "nav_pos_n" in data and "nav_pos_e" in data:
        pos_n = data["nav_pos_n"]
        pos_e = data["nav_pos_e"]
        valid = ~(np.isnan(pos_n) | np.isnan(pos_e))
        if np.sum(valid) > 100:
            has_data = True
            pn = pos_n[valid]
            pe = pos_e[valid]
            # Detect jumps: >500cm (5m) in a single sample
            dn = np.abs(np.diff(pn))
            de = np.abs(np.diff(pe))
            dist_jump = np.sqrt(dn**2 + de**2)
            # Scale threshold by sample rate (at 500Hz, 5m/sample = 2500m/s which is impossible)
            max_speed_cms = 3000  # 30 m/s max realistic speed
            threshold = max_speed_cms / sr * 3  # 3x max speed per sample
            threshold = max(threshold, 200)  # at least 2m jump
            jumps = int(np.sum(dist_jump > threshold))
            results["position_jumps"] = jumps
            if jumps > 5:
                score -= 25
                findings.append(("WARNING", f"{jumps} GPS position jumps detected (>2m) - "
                                 "check antenna placement and sky view"))
            elif jumps > 0:
                score -= 5
                findings.append(("INFO", f"{jumps} minor GPS position jump(s) detected"))

    # ─── GPS frames: satellite count and update rate ───
    gps_frames = data.get("_gps_frames", [])
    if gps_frames:
        has_data = True
        # Extract sat count (field name: GPS_numSat)
        sats = []
        for idx, fields in gps_frames:
            for key in ("GPS_numSat", "GPS_numsat", "gps_numsat"):
                if key in fields:
                    try:
                        s = int(fields[key])
                        if 0 < s < 50:  # sanity check
                            sats.append(s)
                    except (ValueError, TypeError):
                        pass
                    break

        if sats:
            avg_sats = float(np.mean(sats))
            min_sats = int(np.min(sats))
            results["avg_sats"] = round(avg_sats, 1)
            results["min_sats"] = min_sats

            if min_sats < 6:
                score -= 20
                findings.append(("WARNING", f"GPS sat count dropped to {min_sats} "
                                 f"(avg {avg_sats:.0f}) - need 8+ for reliable nav"))
            elif avg_sats < 8:
                score -= 10
                findings.append(("INFO", f"Low average GPS sats: {avg_sats:.0f} "
                                 "(8+ recommended for nav modes)"))

        # GPS update rate
        if len(gps_frames) > 2:
            indices = [idx for idx, _ in gps_frames]
            sample_span = indices[-1] - indices[0]
            if sample_span > 0:
                gps_rate = len(gps_frames) / (sample_span / sr)
                results["gps_rate_hz"] = round(gps_rate, 1)

    if not has_data:
        return results

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


def analyze_baro_quality(data, sr):
    """Analyze barometer noise and quality.

    Detects:
    - Baro noise level (std dev during steady flight)
    - Propwash-induced baro noise (correlation with throttle)
    - Baro spikes/outliers

    Works on ANY flight mode.
    """
    results = {
        "score": None,
        "noise_cm": None,
        "throttle_correlation": None,
        "spikes": 0,
        "findings": []
    }

    if "baro_alt" not in data:
        return results

    baro = data["baro_alt"]
    valid = ~np.isnan(baro)
    if np.sum(valid) < sr * 5:
        return results

    alt = baro[valid]
    findings = []
    score = 100

    # ─── Detrend altitude (remove intentional climbs/descents) ───
    # Use a very low-pass filter to get the trend, then subtract.
    # sosfiltfilt is zero-phase (no startup transient).
    from scipy.signal import butter, sosfiltfilt
    try:
        if sr > 2:
            sos = butter(2, 0.5 / (sr / 2), btype='low', output='sos')
            trend = sosfiltfilt(sos, alt)
            residual = alt - trend
        else:
            residual = alt - np.mean(alt)
    except Exception:
        residual = alt - np.mean(alt)

    noise_cm = float(np.std(residual))
    results["noise_cm"] = round(noise_cm, 1)

    # ─── Spike detection ───
    threshold = max(noise_cm * 5, 100)  # 5 sigma or 1m, whichever is larger
    spikes = int(np.sum(np.abs(residual) > threshold))
    results["spikes"] = spikes

    # ─── Throttle correlation ───
    throttle = None
    if "throttle" in data:
        throttle = data["throttle"][valid]
    elif "motor0" in data:
        throttle = data["motor0"][valid]

    if throttle is not None and len(throttle) > 100:
        # Low-pass both signals to compare trends
        try:
            if sr > 4:
                sos2 = butter(2, 1.0 / (sr / 2), btype='low', output='sos')
                baro_lp = sosfilt(sos2, residual)
                thr_lp = sosfilt(sos2, throttle - np.mean(throttle))
                corr = abs(float(np.corrcoef(baro_lp, thr_lp)[0, 1]))
                if not np.isnan(corr):
                    results["throttle_correlation"] = round(corr, 3)
        except Exception:
            pass

    # ─── Scoring ───
    if noise_cm > 100:
        score -= 40
        findings.append(("WARNING", f"High baro noise: {noise_cm:.0f}cm RMS - "
                         "check baro foam coverage, seal from prop wash"))
    elif noise_cm > 30:
        score -= 15
        findings.append(("INFO", f"Moderate baro noise: {noise_cm:.0f}cm RMS "
                         "(normal range 5-20cm)"))

    if spikes > 3:
        score -= 15
        findings.append(("WARNING", f"{spikes} baro spikes detected - "
                         "may cause altitude jumps in althold"))

    corr = results["throttle_correlation"]
    if corr is not None and corr > 0.5:
        score -= 20
        findings.append(("WARNING", f"Baro noise correlates with throttle (r={corr:.2f}) - "
                         "propwash affecting barometer, improve foam isolation"))

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


def analyze_altitude_hold(data, sr, phase_start=None, phase_end=None):
    """Analyze altitude hold performance in a specific flight phase.

    Requires navPos[2] (altitude) and navTgtPos[2] (target altitude).
    Only meaningful during ALTHOLD or POSHOLD phases.
    """
    results = {
        "score": None,
        "oscillation_cm": None,
        "overshoot_cm": None,
        "z_vel_noise": None,
        "findings": []
    }

    if "nav_pos_u" not in data or "nav_tgt_u" not in data:
        return results

    s = phase_start or 0
    e = phase_end or len(data["nav_pos_u"])
    pos_z = data["nav_pos_u"][s:e]
    tgt_z = data["nav_tgt_u"][s:e]
    valid = ~(np.isnan(pos_z) | np.isnan(tgt_z))
    if np.sum(valid) < sr * 3:
        return results

    pz = pos_z[valid]
    tz = tgt_z[valid]
    error = pz - tz  # altitude error in cm

    findings = []
    score = 100

    # ─── Oscillation: peak-to-peak of error ───
    osc = float(np.max(error) - np.min(error))
    results["oscillation_cm"] = round(osc, 1)

    if osc > 200:
        score -= 40
        findings.append(("WARNING", f"Large altitude oscillation: {osc:.0f}cm peak-to-peak "
                         "(reduce nav_mc_vel_z_p or nav_mc_pos_z_p)"))
    elif osc > 50:
        score -= 15
        findings.append(("INFO", f"Altitude oscillation: {osc:.0f}cm peak-to-peak "
                         "(acceptable, <50cm is ideal)"))

    # ─── Z velocity noise ───
    if "nav_vel_u" in data:
        vz = data["nav_vel_u"][s:e][valid]
        if len(vz) > 10:
            vz_noise = float(np.std(vz))
            results["z_vel_noise"] = round(vz_noise, 1)
            if vz_noise > 50:
                score -= 15
                findings.append(("INFO", f"Z velocity noise: {vz_noise:.0f}cm/s RMS"))

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


def analyze_position_hold(data, sr, phase_start=None, phase_end=None):
    """Analyze position hold performance.

    Detects:
    - Drift radius (CEP - Circular Error Probable)
    - Toilet bowl / salad bowl (circular oscillation from compass issues)
    - Correction effort

    Only meaningful during POSHOLD phases.
    """
    results = {
        "score": None,
        "cep_cm": None,
        "max_drift_cm": None,
        "toilet_bowl": False,
        "tb_amplitude_cm": None,
        "tb_period_s": None,
        "findings": []
    }

    if not ("nav_pos_n" in data and "nav_pos_e" in data):
        return results
    if not ("nav_tgt_n" in data and "nav_tgt_e" in data):
        # No target - use mean position as reference
        pass

    s = phase_start or 0
    e = phase_end or len(data["nav_pos_n"])
    pn = data["nav_pos_n"][s:e]
    pe = data["nav_pos_e"][s:e]

    if "nav_tgt_n" in data and "nav_tgt_e" in data:
        tn = data["nav_tgt_n"][s:e]
        te = data["nav_tgt_e"][s:e]
    else:
        tn = np.full_like(pn, np.nanmean(pn))
        te = np.full_like(pe, np.nanmean(pe))

    valid = ~(np.isnan(pn) | np.isnan(pe) | np.isnan(tn) | np.isnan(te))
    if np.sum(valid) < sr * 3:
        return results

    err_n = pn[valid] - tn[valid]
    err_e = pe[valid] - te[valid]
    dist = np.sqrt(err_n**2 + err_e**2)

    findings = []
    score = 100

    # ─── CEP (Circular Error Probable) - 50th percentile ───
    cep = float(np.percentile(dist, 50))
    max_drift = float(np.max(dist))
    results["cep_cm"] = round(cep, 1)
    results["max_drift_cm"] = round(max_drift, 1)

    if cep > 500:
        score -= 40
        findings.append(("WARNING", f"Large position drift: CEP {cep:.0f}cm, max {max_drift:.0f}cm "
                         "(check GPS quality, nav_mc_vel_xy PIDs)"))
    elif cep > 200:
        score -= 15
        findings.append(("INFO", f"Position drift: CEP {cep:.0f}cm, max {max_drift:.0f}cm"))

    # ─── Toilet Bowl Detection ───
    # The "salad bowl" pattern: position traces circles because compass heading
    # is offset from true heading. The nav controller corrects in the wrong
    # direction, creating circular drift.
    # Detection: check if position error has a dominant oscillation frequency
    # in the 0.05-0.5 Hz range (2-20 second period).
    if len(err_n) > sr * 10:  # need at least 10 seconds
        try:
            from scipy.signal import welch
            # Compute angle of position error over time
            angle = np.arctan2(err_e, err_n)
            angle_unwrap = np.unwrap(angle)

            # If angle is steadily increasing/decreasing, it's a toilet bowl
            # Rate of angle change
            angle_rate = np.diff(angle_unwrap) * sr  # rad/s
            mean_rate = float(np.mean(angle_rate))
            rate_std = float(np.std(angle_rate))

            # Toilet bowl: consistent rotation rate with amplitude
            if abs(mean_rate) > 0.1 and rate_std < abs(mean_rate) * 3:
                # It's rotating! Check amplitude
                amplitude = float(np.mean(dist))
                if amplitude > 50:  # more than 50cm radius
                    period = abs(2 * np.pi / mean_rate)
                    results["toilet_bowl"] = True
                    results["tb_amplitude_cm"] = round(amplitude, 0)
                    results["tb_period_s"] = round(period, 1)
                    score -= 35
                    findings.append(("WARNING",
                                     f"TOILET BOWL detected: {amplitude:.0f}cm radius, "
                                     f"{period:.1f}s period - compass interference or "
                                     "miscalibration. Recalibrate compass away from motors, "
                                     "twist power leads, check compass orientation"))

            # Also check using PSD for oscillation in position
            if not results["toilet_bowl"]:
                f, psd_n = welch(err_n, fs=sr, nperseg=min(len(err_n), int(sr * 20)))
                f, psd_e = welch(err_e, fs=sr, nperseg=min(len(err_e), int(sr * 20)))
                psd_total = psd_n + psd_e
                # Look in toilet bowl frequency range (0.05-0.5 Hz)
                mask = (f >= 0.05) & (f <= 0.5)
                if np.any(mask):
                    peak_idx = np.argmax(psd_total[mask])
                    peak_freq = f[mask][peak_idx]
                    peak_power = psd_total[mask][peak_idx]
                    total_power = np.sum(psd_total[mask])
                    # If one frequency dominates, it's oscillatory
                    if peak_power > total_power * 0.4 and cep > 100:
                        period = 1.0 / peak_freq
                        results["toilet_bowl"] = True
                        results["tb_amplitude_cm"] = round(cep, 0)
                        results["tb_period_s"] = round(period, 1)
                        score -= 25
                        findings.append(("WARNING",
                                         f"Oscillatory position drift detected at {peak_freq:.2f}Hz "
                                         f"({period:.1f}s) - possible toilet bowl, check compass"))
        except Exception:
            pass

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


def analyze_estimator_health(data, sr):
    """Check if the navigation position estimator is healthy.

    Compares navPos[2] (estimated altitude) vs BaroAlt to detect
    estimator divergence - a dangerous condition where the FC's
    internal model departs from reality.

    Works on ANY flight mode.
    """
    results = {
        "score": None,
        "baro_vs_nav_corr": None,
        "max_divergence_cm": None,
        "findings": []
    }

    if "nav_pos_u" not in data or "baro_alt" not in data:
        return results

    nav_z = data["nav_pos_u"]
    baro = data["baro_alt"]
    valid = ~(np.isnan(nav_z) | np.isnan(baro))
    if np.sum(valid) < sr * 5:
        return results

    nz = nav_z[valid]
    ba = baro[valid]

    findings = []
    score = 100

    # ─── Correlation between estimated and baro altitude ───
    try:
        corr = float(np.corrcoef(nz, ba)[0, 1])
        if not np.isnan(corr):
            results["baro_vs_nav_corr"] = round(corr, 3)
            if corr < 0.8:
                score -= 40
                findings.append(("WARNING",
                                 f"Nav estimator diverges from barometer (r={corr:.2f}) - "
                                 "altitude estimates may be unreliable"))
            elif corr < 0.95:
                score -= 10
                findings.append(("INFO",
                                 f"Mild estimator-baro disagreement (r={corr:.2f})"))
    except (ValueError, FloatingPointError):
        pass

    # ─── Max divergence ───
    # Normalize both to start at 0 and compare
    nz_norm = nz - nz[0]
    ba_norm = ba - ba[0]
    divergence = np.abs(nz_norm - ba_norm)
    max_div = float(np.max(divergence))
    results["max_divergence_cm"] = round(max_div, 1)

    if max_div > 1000:  # >10m divergence
        score -= 30
        findings.append(("WARNING",
                         f"Estimator diverged {max_div:.0f}cm from baro - "
                         "IMU/accel issue or baro failure"))

    results["score"] = max(0, score)
    results["findings"] = findings
    return results


# ─── Nav Controller Performance Analysis ─────────────────────────────────────

def _get_nav_mode_mask(data, sr, mode_bit):
    """Build a boolean mask at I-frame rate for a specific nav mode.

    Uses slow frame flight mode flags to determine when a mode was active.
    Returns np.ndarray of bools with same length as data arrays.
    """
    slow_frames = data.get("_slow_frames", [])
    n_rows = data.get("n_rows", len(data.get("time_s", [])))
    mask = np.zeros(n_rows, dtype=bool)

    if not slow_frames:
        return mask

    # Build transitions from slow frames
    transitions = []
    for frame_idx, fields in slow_frames:
        mode_flags = fields.get("flightModeFlags", None)
        if mode_flags is None:
            for k in fields:
                if "flight" in k.lower() and "mode" in k.lower():
                    mode_flags = fields[k]
                    break
        if mode_flags is not None:
            try:
                flags = int(mode_flags)
                active = bool(flags & (1 << mode_bit))
                transitions.append((min(frame_idx, n_rows - 1), active))
            except (ValueError, TypeError):
                pass

    # Fill mask between transitions
    for i, (idx, active) in enumerate(transitions):
        next_idx = transitions[i + 1][0] if i + 1 < len(transitions) else n_rows
        if active:
            mask[idx:next_idx] = True

    return mask


# INAV flight mode bits (from src/main/fc/runtime_config.h)
NAV_MODE_ALTHOLD = 3
NAV_MODE_RTH = 7
NAV_MODE_POSHOLD = 8
NAV_MODE_CRUISE = 28


def analyze_nav_performance(data, sr, config=None, profile=None):
    """Analyze navigation controller performance from blackbox data.

    Examines the nav PID stack internals to detect:
    - Deceleration overshoot (velocity controller too aggressive)
    - Position hold quality (position controller tuning)
    - Altitude hold quality (vertical controller tuning)
    - Controller saturation (nav demanding more than platform can deliver)
    - Wind vs tuning separation (I-term vs environmental drift)

    Returns dict with findings, metrics, and specific nav PID recommendations.
    """
    if profile is None:
        profile = get_frame_profile(5)
    frame_inches = profile.get("frame_inches", 5)

    results = {
        "deceleration": [],
        "poshold": None,
        "althold": None,
        "saturation": None,
        "wind_correlation": None,
        "findings": [],
        "nav_actions": [],
        "score": None,
    }

    has_pos = "nav_pos_n" in data and "nav_pos_e" in data
    has_vel = "nav_vel_n" in data and "nav_vel_e" in data
    has_tgt = "nav_tgt_n" in data and "nav_tgt_e" in data
    has_tgt_vel = "nav_tgt_vel_n" in data and "nav_tgt_vel_e" in data
    has_vel_ctrl = "mc_vel_out_0" in data

    if not (has_pos and has_vel):
        return results

    # Get nav mode masks
    poshold_mask = _get_nav_mode_mask(data, sr, NAV_MODE_POSHOLD)
    althold_mask = _get_nav_mode_mask(data, sr, NAV_MODE_ALTHOLD) | poshold_mask
    rth_mask = _get_nav_mode_mask(data, sr, NAV_MODE_RTH)
    any_nav_mask = poshold_mask | althold_mask | rth_mask

    score = 100
    findings = []
    nav_actions = []

    # ═══ 1. DECELERATION OVERSHOOT ═══
    if has_tgt_vel and has_tgt:
        tgt_vel_n = data["nav_tgt_vel_n"].copy()
        tgt_vel_e = data["nav_tgt_vel_e"].copy()
        pos_n = data["nav_pos_n"].copy()
        pos_e = data["nav_pos_e"].copy()
        tgt_n = data["nav_tgt_n"].copy()
        tgt_e = data["nav_tgt_e"].copy()

        # Replace NaN with 0 for velocity
        tgt_vel_n = np.nan_to_num(tgt_vel_n, 0)
        tgt_vel_e = np.nan_to_num(tgt_vel_e, 0)

        tgt_speed = np.sqrt(tgt_vel_n**2 + tgt_vel_e**2)  # cm/s

        # Find deceleration events: target speed drops from >100cm/s to <30cm/s
        min_speed = 100  # cm/s = 1 m/s
        stop_speed = 30  # cm/s = 0.3 m/s

        decel_events = []
        in_fast = False
        fast_start = 0

        for i in range(len(tgt_speed)):
            if not in_fast and tgt_speed[i] > min_speed:
                in_fast = True
                fast_start = i
            elif in_fast and tgt_speed[i] < stop_speed:
                in_fast = False
                stop_idx = i
                # Analyze the post-stop settling
                # Look at position error for 5 seconds after stop
                settle_window = min(int(sr * 5), len(pos_n) - stop_idx)
                if settle_window < int(sr * 0.5):
                    continue  # too short to analyze

                window_end = stop_idx + settle_window
                err_n = pos_n[stop_idx:window_end] - tgt_n[stop_idx:window_end]
                err_e = pos_e[stop_idx:window_end] - tgt_e[stop_idx:window_end]

                # Filter NaN
                valid = ~(np.isnan(err_n) | np.isnan(err_e))
                if np.sum(valid) < int(sr * 0.3):
                    continue

                err_n = err_n[valid]
                err_e = err_e[valid]
                err_dist = np.sqrt(err_n**2 + err_e**2)

                # Metrics
                peak_error = float(np.max(err_dist))  # cm
                mean_error = float(np.mean(err_dist))

                # Count oscillations (zero-crossings in err_n or err_e)
                # Use the axis with more variation
                err_axis = err_n if np.std(err_n) > np.std(err_e) else err_e
                err_centered = err_axis - np.mean(err_axis)
                zero_crossings = int(np.sum(np.abs(np.diff(np.sign(err_centered))) > 0))
                oscillation_count = zero_crossings // 2

                # Settling time: when error stays within 100cm (1m)
                settle_threshold = 100  # cm
                settled_idx = None
                for j in range(len(err_dist) - 1, -1, -1):
                    if err_dist[j] > settle_threshold:
                        settled_idx = j
                        break
                settling_time = (settled_idx + 1) / sr if settled_idx is not None else 0

                time_s = float(data["time_s"][stop_idx]) if "time_s" in data else stop_idx / sr

                decel_events.append({
                    "time_s": time_s,
                    "peak_error_cm": peak_error,
                    "mean_error_cm": mean_error,
                    "oscillation_count": oscillation_count,
                    "settling_time_s": settling_time,
                    "duration_fast_s": (stop_idx - fast_start) / sr,
                })

        results["deceleration"] = decel_events

        if decel_events:
            worst = max(decel_events, key=lambda e: e["peak_error_cm"])
            avg_peak = np.mean([e["peak_error_cm"] for e in decel_events])
            avg_osc = np.mean([e["oscillation_count"] for e in decel_events])
            avg_settle = np.mean([e["settling_time_s"] for e in decel_events])

            # Frame-scaled thresholds
            ok_overshoot = 150 * (frame_inches / 7)  # cm
            bad_overshoot = 300 * (frame_inches / 7)
            ok_oscillations = 2
            bad_oscillations = 4

            if avg_peak > bad_overshoot or avg_osc > bad_oscillations:
                score -= 30
                findings.append({
                    "level": "WARNING",
                    "text": f"Deceleration overshoot: {avg_peak:.0f}cm avg peak error, "
                            f"{avg_osc:.1f} oscillations avg, {avg_settle:.1f}s to settle "
                            f"({len(decel_events)} events)",
                    "detail": f"Worst: {worst['peak_error_cm']:.0f}cm at {worst['time_s']:.1f}s. "
                              f"The quad overshoots its target position when stopping."
                })

                vel_p = config.get("nav_vel_xy_p") if config else None
                decel_time = config.get("nav_decel_time") if config else None

                if vel_p and vel_p > 20:
                    new_vel_p = max(15, int(vel_p * 0.7))
                    nav_actions.append({
                        "action": f"Reduce nav velocity P: {vel_p} -> {new_vel_p}",
                        "reason": f"Deceleration overshoot {avg_peak:.0f}cm with {avg_osc:.0f} oscillations. "
                                  f"Velocity controller brakes too hard for {frame_inches}-inch momentum.",
                        "cli": f"set nav_mc_vel_xy_p = {new_vel_p}",
                    })
                if decel_time and decel_time < 200:
                    new_decel = min(400, int(decel_time * 1.5))
                    nav_actions.append({
                        "action": f"Increase deceleration time: {decel_time} -> {new_decel} ({new_decel/100:.1f}s)",
                        "reason": f"Quad needs more distance to bleed speed on {frame_inches}-inch frame.",
                        "cli": f"set nav_mc_pos_deceleration_time = {new_decel}",
                    })

            elif avg_peak > ok_overshoot:
                score -= 10
                findings.append({
                    "level": "INFO",
                    "text": f"Mild deceleration overshoot: {avg_peak:.0f}cm avg, "
                            f"{avg_settle:.1f}s settling ({len(decel_events)} events)",
                    "detail": "Acceptable but could be tighter with lower velocity P."
                })

    # ═══ 2. POSITION HOLD QUALITY ═══
    if has_tgt and np.any(poshold_mask):
        pos_n = data["nav_pos_n"].copy()
        pos_e = data["nav_pos_e"].copy()
        tgt_n = data["nav_tgt_n"].copy()
        tgt_e = data["nav_tgt_e"].copy()

        # Isolate poshold segments
        ph_pos_n = pos_n[poshold_mask]
        ph_pos_e = pos_e[poshold_mask]
        ph_tgt_n = tgt_n[poshold_mask]
        ph_tgt_e = tgt_e[poshold_mask]

        valid = ~(np.isnan(ph_pos_n) | np.isnan(ph_pos_e) |
                  np.isnan(ph_tgt_n) | np.isnan(ph_tgt_e))

        if np.sum(valid) > sr * 2:  # at least 2 seconds of poshold
            err_n = ph_pos_n[valid] - ph_tgt_n[valid]
            err_e = ph_pos_e[valid] - ph_tgt_e[valid]
            err_dist = np.sqrt(err_n**2 + err_e**2)

            cep = float(np.percentile(err_dist, 50))
            max_drift = float(np.max(err_dist))
            rms_error = float(np.sqrt(np.mean(err_dist**2)))
            hold_duration = np.sum(valid) / sr

            # Toilet bowl detection (circular oscillation from compass issues)
            toilet_bowl = False
            tb_period = None
            if len(err_n) > sr * 3:
                from scipy.fft import rfft, rfftfreq
                freqs_n = rfftfreq(len(err_n), 1.0 / sr)
                spec_n = np.abs(rfft(err_n - np.mean(err_n)))
                spec_e = np.abs(rfft(err_e - np.mean(err_e)))

                # Look for matching peaks in both axes (0.1-2Hz range)
                band = (freqs_n >= 0.1) & (freqs_n <= 2.0)
                if np.any(band):
                    peak_n = freqs_n[band][np.argmax(spec_n[band])]
                    peak_e = freqs_n[band][np.argmax(spec_e[band])]
                    # Peaks within 20% of each other = likely toilet bowl
                    if abs(peak_n - peak_e) < max(peak_n, peak_e) * 0.2:
                        # Check if both axes have significant oscillation
                        n_power = spec_n[band][np.argmax(spec_n[band])]
                        e_power = spec_e[band][np.argmax(spec_e[band])]
                        n_mean = np.mean(spec_n[band])
                        e_mean = np.mean(spec_e[band])
                        if n_power > n_mean * 4 and e_power > e_mean * 4:
                            toilet_bowl = True
                            tb_period = 1.0 / ((peak_n + peak_e) / 2)

            results["poshold"] = {
                "cep_cm": round(cep, 1),
                "max_drift_cm": round(max_drift, 1),
                "rms_error_cm": round(rms_error, 1),
                "hold_duration_s": round(hold_duration, 1),
                "toilet_bowl": toilet_bowl,
                "tb_period_s": round(tb_period, 1) if tb_period else None,
            }

            # Thresholds scaled by frame (larger frames drift more)
            ok_cep = 150 * (frame_inches / 7)
            bad_cep = 400 * (frame_inches / 7)

            if toilet_bowl:
                score -= 25
                findings.append({
                    "level": "WARNING",
                    "text": f"Toilet bowl pattern detected (period {tb_period:.1f}s, CEP {cep:.0f}cm)",
                    "detail": "Circular drift pattern indicates compass heading error. "
                              "Check compass calibration, reduce magnetic interference, "
                              "or raise the GPS mast."
                })
            elif cep > bad_cep:
                score -= 20
                findings.append({
                    "level": "WARNING",
                    "text": f"Large position hold drift: CEP {cep:.0f}cm, max {max_drift:.0f}cm "
                            f"({hold_duration:.0f}s hold)",
                    "detail": "Position hold is loose. May need higher pos_xy_p or vel_xy_i for wind."
                })
                pos_p_val = config.get("nav_pos_p") if config else None
                if pos_p_val and pos_p_val < 50:
                    new_pos_p = min(65, int(pos_p_val * 1.3))
                    nav_actions.append({
                        "action": f"Increase nav position P: {pos_p_val} -> {new_pos_p}",
                        "reason": f"Position hold CEP {cep:.0f}cm — too loose, needs stronger correction.",
                        "cli": f"set nav_mc_pos_xy_p = {new_pos_p}",
                    })
            elif cep > ok_cep:
                score -= 5
                findings.append({
                    "level": "INFO",
                    "text": f"Position hold: CEP {cep:.0f}cm, max {max_drift:.0f}cm "
                            f"({hold_duration:.0f}s hold)",
                    "detail": "Acceptable. Wind and GPS accuracy are likely the limiting factors."
                })
            else:
                findings.append({
                    "level": "OK",
                    "text": f"Position hold: CEP {cep:.0f}cm, max {max_drift:.0f}cm — good",
                    "detail": ""
                })

    # ═══ 3. ALTITUDE HOLD QUALITY ═══
    if "nav_pos_u" in data and "nav_tgt_u" in data and np.any(althold_mask):
        pos_z = data["nav_pos_u"].copy()
        tgt_z = data["nav_tgt_u"].copy()

        ah_pos_z = pos_z[althold_mask]
        ah_tgt_z = tgt_z[althold_mask]

        valid = ~(np.isnan(ah_pos_z) | np.isnan(ah_tgt_z))
        if np.sum(valid) > sr * 2:
            err_z = ah_pos_z[valid] - ah_tgt_z[valid]
            rms_z = float(np.sqrt(np.mean(err_z**2)))
            max_z = float(np.max(np.abs(err_z)))
            hold_dur = np.sum(valid) / sr

            # Oscillation detection in Z
            z_osc = False
            z_osc_freq = None
            if len(err_z) > sr * 2:
                from scipy.fft import rfft, rfftfreq
                freqs = rfftfreq(len(err_z), 1.0 / sr)
                spec = np.abs(rfft(err_z - np.mean(err_z)))
                band = (freqs >= 0.1) & (freqs <= 5.0)
                if np.any(band):
                    peak_idx = np.argmax(spec[band])
                    peak_power = spec[band][peak_idx]
                    mean_power = np.mean(spec[band])
                    if peak_power > mean_power * 5:
                        z_osc = True
                        z_osc_freq = float(freqs[band][peak_idx])

            results["althold"] = {
                "rms_error_cm": round(rms_z, 1),
                "max_error_cm": round(max_z, 1),
                "hold_duration_s": round(hold_dur, 1),
                "oscillation": z_osc,
                "osc_freq_hz": round(z_osc_freq, 2) if z_osc_freq else None,
            }

            if z_osc:
                score -= 20
                findings.append({
                    "level": "WARNING",
                    "text": f"Altitude oscillation at {z_osc_freq:.1f}Hz "
                            f"(RMS {rms_z:.0f}cm, max {max_z:.0f}cm)",
                    "detail": "Altitude is bobbing. Reduce nav_mc_vel_z_p or nav_mc_pos_z_p."
                })
            elif max_z > 200:
                score -= 10
                findings.append({
                    "level": "INFO",
                    "text": f"Altitude hold: RMS {rms_z:.0f}cm, max {max_z:.0f}cm ({hold_dur:.0f}s)",
                    "detail": "Some altitude wander. Check baro seal and nav Z gains."
                })
            else:
                findings.append({
                    "level": "OK",
                    "text": f"Altitude hold: RMS {rms_z:.0f}cm, max {max_z:.0f}cm — good",
                    "detail": ""
                })

    # ═══ 4. VELOCITY CONTROLLER SATURATION ═══
    if has_vel_ctrl and np.any(any_nav_mask):
        vel_out_0 = data["mc_vel_out_0"][any_nav_mask]
        vel_out_1 = data["mc_vel_out_1"][any_nav_mask]
        valid = ~(np.isnan(vel_out_0) | np.isnan(vel_out_1))

        if np.sum(valid) > sr:
            v0 = vel_out_0[valid]
            v1 = vel_out_1[valid]

            # Saturation: output hitting limits (typically +/-500 for tilt angle)
            # Look for values near the extremes
            output_max = max(np.percentile(np.abs(v0), 99.5),
                           np.percentile(np.abs(v1), 99.5))
            sat_threshold = output_max * 0.95
            sat_pct_0 = float(np.sum(np.abs(v0) > sat_threshold) / len(v0) * 100)
            sat_pct_1 = float(np.sum(np.abs(v1) > sat_threshold) / len(v1) * 100)
            max_sat = max(sat_pct_0, sat_pct_1)

            results["saturation"] = {
                "sat_pct_n": round(sat_pct_0, 1),
                "sat_pct_e": round(sat_pct_1, 1),
                "output_max": round(output_max, 0),
            }

            if max_sat > 10:
                score -= 15
                findings.append({
                    "level": "WARNING",
                    "text": f"Nav velocity controller saturating {max_sat:.0f}% of the time",
                    "detail": "Nav is demanding more tilt than the platform can deliver. "
                              "Reduce nav_mc_vel_xy_p or increase nav_mc_max_bank_angle."
                })

    # ═══ 5. WIND VS TUNING (from S-frame wind estimates) ═══
    if has_pos and has_tgt:
        slow_frames = data.get("_slow_frames", [])
        wind_data = []
        for frame_idx, fields in slow_frames:
            w0 = fields.get("wind[0]")
            w1 = fields.get("wind[1]")
            if w0 is not None and w1 is not None:
                try:
                    wind_data.append((frame_idx, float(w0), float(w1)))
                except (ValueError, TypeError):
                    pass

        if len(wind_data) > 10 and np.any(poshold_mask):
            # Interpolate wind to I-frame rate during poshold
            wind_idxs = np.array([w[0] for w in wind_data])
            wind_n = np.array([w[1] for w in wind_data])
            wind_e = np.array([w[2] for w in wind_data])

            pos_n = data["nav_pos_n"].copy()
            pos_e = data["nav_pos_e"].copy()
            tgt_n = data["nav_tgt_n"].copy()
            tgt_e = data["nav_tgt_e"].copy()

            # Simple: correlate wind speed with position error magnitude during poshold
            ph_indices = np.where(poshold_mask)[0]
            if len(ph_indices) > sr * 2:
                err_n = pos_n[ph_indices] - tgt_n[ph_indices]
                err_e = pos_e[ph_indices] - tgt_e[ph_indices]
                err_mag = np.sqrt(err_n**2 + err_e**2)

                # Interpolate wind to poshold indices
                wind_interp_n = np.interp(ph_indices, wind_idxs, wind_n)
                wind_interp_e = np.interp(ph_indices, wind_idxs, wind_e)
                wind_mag = np.sqrt(wind_interp_n**2 + wind_interp_e**2)

                valid = ~(np.isnan(err_mag) | np.isnan(wind_mag))
                if np.sum(valid) > 100 and np.std(wind_mag[valid]) > 1.0:
                    correlation = float(np.corrcoef(err_mag[valid], wind_mag[valid])[0, 1])
                    if np.isnan(correlation):
                        correlation = 0.0
                    avg_wind = float(np.mean(wind_mag[valid]))

                    results["wind_correlation"] = {
                        "correlation": round(correlation, 2),
                        "avg_wind_cms": round(avg_wind, 0),
                    }

                    if correlation > 0.5 and avg_wind > 50:
                        findings.append({
                            "level": "INFO",
                            "text": f"Position error correlates with wind ({correlation:.0%}, "
                                    f"avg wind {avg_wind/100:.1f}m/s)",
                            "detail": "Drift is wind-driven, not a PID tuning problem. "
                                      "Increase nav_mc_vel_xy_i for better wind holding."
                        })
                        vel_i = config.get("nav_vel_xy_i") if config else None
                        if vel_i and vel_i < 30:
                            new_vel_i = min(50, int(vel_i * 1.5))
                            nav_actions.append({
                                "action": f"Increase nav velocity I: {vel_i} -> {new_vel_i}",
                                "reason": f"Position error is {correlation:.0%} correlated with wind. "
                                          f"Higher I-term compensates for sustained wind forces.",
                                "cli": f"set nav_mc_vel_xy_i = {new_vel_i}",
                            })

    # ═══ SCORING ═══
    results["score"] = max(0, score)
    results["findings"] = findings
    results["nav_actions"] = nav_actions

    return results


def run_nav_analysis(data, sr, config=None):
    """Main entry point for navigation analysis.

    Runs all applicable nav health checks based on available data.
    Returns a dict with results from each sub-analyzer and an overall nav score.
    """
    avail = detect_nav_fields(data)
    if not avail["has_any"]:
        return None

    results = {
        "available_fields": avail,
        "compass": analyze_compass_health(data, sr),
        "gps": analyze_gps_quality(data, sr),
        "baro": analyze_baro_quality(data, sr),
        "estimator": analyze_estimator_health(data, sr),
        "althold": None,
        "poshold": None,
        "phases": [],
    }

    # ─── Phase-specific analysis ───
    phases = segment_flight_phases(data, sr)
    results["phases"] = phases

    # Only run althold/poshold analysis during actual nav-controlled phases.
    # Don't analyze the full flight - gives false drift/oscillation readings
    # when the pilot is flying manually.
    has_nav_phase = False
    if phases:
        for start, end, state_val in phases:
            duration_s = (end - start) / sr
            if state_val > 1 and duration_s > 5:
                has_nav_phase = True
                break

    if has_nav_phase and avail["has_pos"] and avail["has_tgt"]:
        best = max(phases, key=lambda p: p[1] - p[0])
        results["althold"] = analyze_altitude_hold(data, sr, best[0], best[1])
        results["poshold"] = analyze_position_hold(data, sr, best[0], best[1])

    # ─── Overall nav score ───
    scores = []
    weights = []
    for key, weight in [("compass", 30), ("gps", 25), ("baro", 25), ("estimator", 20)]:
        s = results[key].get("score") if results[key] else None
        if s is not None:
            scores.append(s)
            weights.append(weight)

    if scores:
        total_weight = sum(weights)
        results["nav_score"] = round(sum(s * w for s, w in zip(scores, weights)) / total_weight)
    else:
        results["nav_score"] = None

    # ─── Cross-reference with FC config (when diff is available) ───
    if config and config.get("_diff_merged"):
        _cross_reference_nav_config(results, config)

    return results


def _cross_reference_nav_config(nav_results, config):
    """Cross-reference nav sensor readings with FC configuration.

    Adds prescriptive findings when sensor data + config settings create
    dangerous combinations. Only runs when CLI diff data is available.
    """

    # ─── Baro noise + altitude PID gains ───
    baro = nav_results.get("baro")
    if baro and baro.get("noise_cm") is not None:
        noise = baro["noise_cm"]
        alt_p = config.get("nav_alt_p")
        vel_z_p = config.get("nav_vel_z_p")

        if noise > 50 and alt_p is not None:
            try:
                alt_p = int(alt_p)
                if alt_p > 40:
                    baro["findings"].append((
                        "WARNING",
                        f"Baro noise {noise:.0f}cm + aggressive nav_mc_pos_z_p={alt_p} "
                        f"will cause altitude oscillation in althold. "
                        f"Fix baro foam first, or reduce: set nav_mc_pos_z_p = 30"))
            except (ValueError, TypeError):
                pass

        if noise > 50 and vel_z_p is not None:
            try:
                vel_z_p = int(vel_z_p)
                if vel_z_p > 100:
                    baro["findings"].append((
                        "INFO",
                        f"High baro noise + nav_mc_vel_z_p={vel_z_p} amplifies noise "
                        f"into throttle commands. Consider reducing after fixing baro."))
            except (ValueError, TypeError):
                pass

    # ─── Compass jitter + mag calibration quality ───
    compass = nav_results.get("compass")
    if compass and compass.get("heading_jitter_deg") is not None:
        jitter = compass["heading_jitter_deg"]

        # Check mag gain spread
        cal_x = config.get("mag_cal_x")
        cal_y = config.get("mag_cal_y")
        cal_z = config.get("mag_cal_z")
        if cal_x is not None and cal_y is not None and cal_z is not None:
            try:
                gains = [abs(int(cal_x)), abs(int(cal_y)), abs(int(cal_z))]
                if max(gains) > 0:
                    spread = (max(gains) - min(gains)) / max(gains) * 100
                    if spread > 30 and jitter > 2.0:
                        compass["findings"].append((
                            "WARNING",
                            f"Compass jitter {jitter:.1f} deg/s + mag gain spread "
                            f"{spread:.0f}% (X={gains[0]} Y={gains[1]} Z={gains[2]}). "
                            f"Recalibrate compass outdoors away from metal and motors."))
            except (ValueError, TypeError):
                pass

        # Check heading P gain amplifying jitter
        heading_p = config.get("nav_heading_p")
        if heading_p is not None and jitter > 3.0:
            try:
                heading_p = int(heading_p)
                if heading_p > 30:
                    compass["findings"].append((
                        "INFO",
                        f"Heading jitter {jitter:.1f} deg/s is amplified by "
                        f"nav_mc_heading_p={heading_p}. Fix compass first, "
                        f"or reduce: set nav_mc_heading_p = 30"))
            except (ValueError, TypeError):
                pass

        # No compass configured
        mag_hw = config.get("mag_hardware")
        if mag_hw and str(mag_hw).upper() in ("NONE", "0", "FALSE"):
            compass["findings"].append((
                "WARNING",
                "No compass configured (mag_hardware=NONE). "
                "Poshold and RTH will use GPS course only - unreliable at low speed."))

    # ─── GPS quality + constellation config ───
    gps = nav_results.get("gps")
    if gps and gps.get("avg_sats") is not None:
        avg_sats = gps["avg_sats"]

        galileo = config.get("gps_use_galileo")
        beidou = config.get("gps_use_beidou")
        glonass = config.get("gps_use_glonass")

        disabled = []
        if galileo is not None and not galileo:
            disabled.append("Galileo")
        if beidou is not None and not beidou:
            disabled.append("Beidou")
        if glonass is not None and not glonass:
            disabled.append("GLONASS")

        if disabled and avg_sats < 15:
            fixes = [f"set gps_ublox_use_{d.lower()} = ON" for d in disabled]
            gps["findings"].append((
                "INFO",
                f"{avg_sats:.0f} sats avg with {', '.join(disabled)} disabled. "
                f"Enable for more satellites: {'; '.join(fixes)}"))

    # ─── Estimator + weight tuning ───
    est = nav_results.get("estimator")
    if est and est.get("baro_vs_nav_corr") is not None:
        corr = est["baro_vs_nav_corr"]
        w_baro = config.get("inav_w_z_baro_p")
        if w_baro is not None and corr < 0.9:
            try:
                w_baro = float(w_baro)
                if w_baro < 0.5:
                    est["findings"].append((
                        "INFO",
                        f"Estimator-baro correlation {corr:.2f} with low baro weight "
                        f"inav_w_z_baro_p={w_baro:.1f}. If altitude drifts in althold, "
                        f"increase: set inav_w_z_baro_p = 1.0"))
            except (ValueError, TypeError):
                pass


def format_nav_report(nav_results, use_color=True):
    """Format navigation analysis results for terminal output."""
    if nav_results is None:
        return ""

    if use_color and _ANSI_ENABLED:
        R, BOLD, _C, G, Y, RED, DIM = _colors()
    else:
        G = Y = RED = DIM = R = BOLD = ""

    def sc(v):
        if v is None:
            return f"{DIM}-{R}"
        if v >= 80:
            return f"{G}{v}/100{R}"
        if v >= 60:
            return f"{Y}{v}/100{R}"
        return f"{RED}{v}/100{R}"

    lines = []
    lines.append(f"\n  {BOLD}NAV HEALTH{R}")

    if nav_results.get("_tune_warning"):
        lines.append(f"  {DIM}(readings affected by PID oscillation - fix tuning first){R}")

    # Sub-scores
    for key, label in [("compass", "Compass"), ("gps", "GPS"),
                       ("baro", "Baro"), ("estimator", "Estimator")]:
        r = nav_results.get(key)
        if r and r.get("score") is not None:
            detail_parts = []
            if key == "compass":
                if r.get("heading_jitter_deg") is not None:
                    detail_parts.append(f"jitter {r['heading_jitter_deg']:.1f} deg/s")
                if r.get("throttle_correlation") is not None:
                    detail_parts.append(f"motor corr {r['throttle_correlation']:.2f}")
            elif key == "gps":
                if r.get("avg_sats") is not None:
                    detail_parts.append(f"{r['avg_sats']:.0f} sats avg")
                if r.get("avg_eph") is not None:
                    detail_parts.append(f"EPH {r['avg_eph']:.0f}cm")
                if r.get("position_jumps", 0) > 0:
                    detail_parts.append(f"{r['position_jumps']} jumps")
            elif key == "baro":
                if r.get("noise_cm") is not None:
                    detail_parts.append(f"noise {r['noise_cm']:.0f}cm RMS")
            elif key == "estimator":
                if r.get("baro_vs_nav_corr") is not None:
                    detail_parts.append(f"baro correlation {r['baro_vs_nav_corr']:.3f}")

            detail = ", ".join(detail_parts) if detail_parts else ""
            lines.append(f"    {label:12s} {sc(r['score']):>12s}  {DIM}{detail}{R}")

    # Overall
    nav_score = nav_results.get("nav_score")
    if nav_score is not None:
        lines.append(f"    {'Nav Score':12s} {sc(nav_score):>12s}")

    # Findings
    all_findings = []
    for key in ("compass", "gps", "baro", "estimator", "althold", "poshold"):
        r = nav_results.get(key)
        if r and r.get("findings"):
            all_findings.extend(r["findings"])

    if all_findings:
        lines.append("")
        for severity, msg in all_findings:
            icon = f"{RED}!" if severity == "WARNING" else f"{Y}*"
            lines.append(f"    {icon}{R} {msg}")

    return "\n".join(lines)


def generate_nav_html_section(nav_results):
    """Generate HTML for the navigation section of the report."""
    if nav_results is None:
        return ""

    html_parts = ['<div class="nav-section"><h3>Navigation Health</h3>']

    # Score table
    html_parts.append('<table class="nav-scores"><tr><th>Check</th><th>Score</th><th>Details</th></tr>')
    for key, label in [("compass", "Compass"), ("gps", "GPS"),
                       ("baro", "Barometer"), ("estimator", "Estimator")]:
        r = nav_results.get(key)
        if r and r.get("score") is not None:
            s = r["score"]
            css = "good" if s >= 80 else "warn" if s >= 60 else "bad"
            details = []
            if key == "compass":
                if r.get("heading_jitter_deg") is not None:
                    details.append(f"Jitter: {r['heading_jitter_deg']:.1f} deg/s")
                if r.get("throttle_correlation") is not None:
                    details.append(f"Motor correlation: {r['throttle_correlation']:.2f}")
            elif key == "gps":
                if r.get("avg_sats") is not None:
                    details.append(f"Avg sats: {r['avg_sats']:.0f}")
                if r.get("avg_eph") is not None:
                    details.append(f"EPH: {r['avg_eph']:.0f}cm")
            elif key == "baro":
                if r.get("noise_cm") is not None:
                    details.append(f"Noise: {r['noise_cm']:.0f}cm RMS")
            elif key == "estimator":
                if r.get("baro_vs_nav_corr") is not None:
                    details.append(f"Baro corr: {r['baro_vs_nav_corr']:.3f}")
            detail_str = ", ".join(details)
            html_parts.append(f'<tr><td>{label}</td><td class="{css}">{s}/100</td>'
                              f'<td>{detail_str}</td></tr>')
    html_parts.append('</table>')

    # Findings
    all_findings = []
    for key in ("compass", "gps", "baro", "estimator", "althold", "poshold"):
        r = nav_results.get(key)
        if r and r.get("findings"):
            all_findings.extend(r["findings"])

    if all_findings:
        html_parts.append('<div class="nav-findings">')
        for severity, msg in all_findings:
            css = "warning" if severity == "WARNING" else "info"
            html_parts.append(f'<div class="finding {css}">{msg}</div>')
        html_parts.append('</div>')

    html_parts.append('</div>')
    return "\n".join(html_parts)


def generate_action_plan(noise_results, pid_results, motor_analysis, dterm_results,
                          config, data, profile=None, phase_lag=None, motor_response=None,
                          rpm_range=None, prop_harmonics=None, hover_osc=None):
    actions = []
    if profile is None:
        profile = get_frame_profile(5)

    # ── Idle/ground detection: quad armed but never flew ──
    # All analysis is meaningless for ground segments - noise floor of a
    # stationary quad tells us nothing, filter/PID recommendations are bogus.
    is_idle = motor_analysis and motor_analysis.get("idle_detected", False)
    duration = data["time_s"][-1] if "time_s" in data and len(data["time_s"]) > 0 else 0

    if is_idle:
        return {
            "actions": [], "info": [],
            "scores": {
                "overall": None, "noise": None, "pid": None,
                "pid_measurable": False, "gyro_oscillation": None,
                "hover_osc": hover_osc or [], "motor": None,
            },
            "verdict": "GROUND_ONLY",
            "verdict_text": ("Motors were at idle throughout this log - the quad was armed "
                             "but never flew. No tuning analysis is possible. "
                             "Fly with throttle above hover, then re-analyze."),
        }

    has_config = config_has_pid(config)
    has_filters = config_has_filters(config)
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]
    ok_noise = profile["ok_noise_db"]
    bad_noise = profile["bad_noise_db"]

    info_items = []

    # ── Hover oscillation (highest priority - can't tune if quad won't hold still) ──
    if hover_osc:
        for osc in hover_osc:
            if osc["severity"] == "none":
                continue
            axis = osc["axis"]
            axis_l = axis.lower()
            rms = osc["gyro_rms"]
            p2p = osc["gyro_p2p"]
            freq = osc["dominant_freq_hz"]
            cause = osc["cause"]
            sev = osc["severity"]

            urg = "CRITICAL" if sev == "severe" else "IMPORTANT" if sev == "moderate" else "RECOMMENDED"
            prio = 0 if sev == "severe" else 1  # Priority 0 = above everything else

            freq_str = f" at ~{freq:.0f}Hz" if freq else ""
            amp_str = f"gyro RMS {rms:.1f}°/s, peak-to-peak {p2p:.0f}°/s{freq_str}"

            if cause == "wind_buffeting":
                # Low-frequency broadband wobble on large frame — likely wind, not P
                # Don't recommend aggressive P cuts — show as informational
                info_items.append({
                    "text": f"{axis}: Low-frequency gyro activity ({rms:.1f}°/s RMS at ~{freq:.0f}Hz)",
                    "detail": f"On a {profile.get('frame_inches', '?')}-inch frame, {freq:.0f}Hz wobble is "
                              f"typically wind buffeting or GPS position corrections, not P oscillation. "
                              f"If this happens indoors or in calm air, then P may be too high."})

            elif cause == "P_too_high":
                current_p = config.get(f"{axis_l}_p")
                if current_p:
                    new_p = max(10, int(current_p * 0.70))  # Aggressive 30% cut
                    actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                        "action": f"{axis}: Hover oscillation - reduce P from {current_p} to {new_p}",
                        "reason": f"Low-frequency oscillation detected during hover ({amp_str}). "
                                  f"P-term is driving the oscillation - reduce P aggressively to stabilize.",
                        "sub_actions": [{"param": f"{axis_l}_p", "new": new_p}]})
                else:
                    actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                        "action": f"{axis}: Hover oscillation - reduce P significantly",
                        "reason": f"Low-frequency oscillation detected during hover ({amp_str}). "
                                  f"P-term is driving the oscillation."})

            elif cause == "PD_interaction":
                current_p = config.get(f"{axis_l}_p")
                current_d = config.get(f"{axis_l}_d")
                sub = []
                parts = []
                if current_p:
                    new_p = max(10, int(current_p * 0.75))
                    sub.append({"param": f"{axis_l}_p", "new": new_p})
                    parts.append(f"P from {current_p} to {new_p}")
                if current_d:
                    new_d = max(0, int(current_d * 0.70))
                    sub.append({"param": f"{axis_l}_d", "new": new_d})
                    parts.append(f"D from {current_d} to {new_d}")
                act_str = f"{axis}: Hover oscillation - reduce {', '.join(parts)}" if parts else f"{axis}: Reduce P and D to stop hover oscillation"
                actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                    "action": act_str,
                    "reason": f"Mid-frequency oscillation detected during hover ({amp_str}). "
                              f"P and D terms are fighting each other - reduce both.",
                    "sub_actions": sub if sub else None})

            elif cause == "D_noise":
                current_dlpf = config.get("dterm_lpf_hz")
                current_d = config.get(f"{axis_l}_d")
                sub = []
                parts = []
                if current_dlpf and current_dlpf > 40:
                    new_dlpf = max(25, int(current_dlpf * 0.6))
                    sub.append({"param": "dterm_lpf_hz", "new": new_dlpf})
                    parts.append(f"D-term LPF from {current_dlpf}Hz to {new_dlpf}Hz")
                if current_d:
                    new_d = max(0, int(current_d * 0.70))
                    sub.append({"param": f"{axis_l}_d", "new": new_d})
                    parts.append(f"D from {current_d} to {new_d}")
                act_str = f"{axis}: Hover oscillation - reduce {', '.join(parts)}" if parts else f"{axis}: Lower D-term LPF and reduce D gain"
                actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                    "action": act_str,
                    "reason": f"D-term noise amplification causing oscillation during hover ({amp_str}). "
                              f"D-term is amplifying noise into the motors.",
                    "sub_actions": sub if sub else None})

            elif cause == "filter_gap":
                current_glpf = config.get("gyro_lpf_hz")
                sub = []
                parts = []
                if current_glpf and current_glpf > 30:
                    new_glpf = max(20, int(current_glpf * 0.6))
                    sub.append({"param": "gyro_lpf_hz", "new": new_glpf})
                    parts.append(f"Gyro LPF from {current_glpf}Hz to {new_glpf}Hz")
                act_str = f"{axis}: Hover oscillation - tighten {', '.join(parts)}" if parts else f"{axis}: Lower Gyro LPF to stop high-frequency oscillation"
                actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                    "action": act_str,
                    "reason": f"High-frequency noise leaking through filters causing oscillation during hover ({amp_str}). "
                              f"Filters need tightening.",
                    "sub_actions": sub if sub else None})

            else:
                # Unknown cause but oscillation is real
                current_p = config.get(f"{axis_l}_p")
                sub = []
                if current_p:
                    new_p = max(10, int(current_p * 0.75))
                    sub.append({"param": f"{axis_l}_p", "new": new_p})
                actions.append({"priority": prio, "urgency": urg, "category": "Oscillation",
                    "action": f"{axis}: Hover oscillation detected - reduce P by 25%",
                    "reason": f"Oscillation detected during hover ({amp_str}). "
                              f"Start by reducing P-term as the most common cause.",
                    "sub_actions": sub if sub else None})

    # ═══ 0. PHASE LAG WARNING (large quad critical) ═══
    if phase_lag and phase_lag["total_degrees"] > 45:
        severity = "CRITICAL" if phase_lag["total_degrees"] > 70 else "IMPORTANT"
        prio = 1 if severity == "CRITICAL" else 2
        chain_desc = ", ".join(f'{c["name"]}={c["degrees"]:.0f}°' for c in phase_lag["chain"])
        actions.append({"priority": prio, "urgency": severity, "category": "Phase Lag",
            "action": f"Total filter phase lag: {phase_lag['total_degrees']:.0f}° ({phase_lag['total_ms']:.1f}ms) at {phase_lag['signal_freq']:.0f}Hz",
            "param": "filter_chain", "current": f"{phase_lag['total_degrees']:.0f}°", "new": "<45°",
            "reason": f"Filter chain: {chain_desc}. "
                       f"Excessive phase lag makes the PID controller fight yesterday's problem. "
                       f"Consider raising lowpass cutoffs or switching BIQUAD filters to PT1."})

    # ═══ 0b. MOTOR RESPONSE INFO (not an action - informational only) ═══
    if motor_response and motor_response["motor_response_ms"] > profile["ok_delay_ms"]:
        mr_ms = motor_response["motor_response_ms"]
        info_items.append({
            "text": f"Motor response time: {mr_ms:.0f}ms",
            "detail": f"Motors physically cannot respond faster than {mr_ms:.0f}ms with these props. "
                      f"This sets a hard floor - tuning for tighter delay than this is futile."})

    # ═══ 1. FILTERS ═══
    current_gyro_lp = config.get("gyro_lowpass_hz") if has_filters else None
    rec_gyro_lp = compute_recommended_filter(noise_results, current_gyro_lp, "gyro", profile)

    if rec_gyro_lp is not None:
        worst_noise = max((nr["rms_high"] for nr in noise_results if nr), default=-80)
        if worst_noise > bad_noise:
            prio, urg = 1, "CRITICAL"
        elif worst_noise > ok_noise:
            prio, urg = 2, "IMPORTANT"
        else:
            prio, urg = 5, None

        if current_gyro_lp is not None and abs(rec_gyro_lp - current_gyro_lp) > 10:
            direction = "Reduce" if rec_gyro_lp < current_gyro_lp else "Increase"
            if rec_gyro_lp < current_gyro_lp:
                reason = f"High-freq noise at {worst_noise:.0f} dB avg - lower cutoff will reduce noise reaching the PID controller"
            else:
                reason = f"Noise floor is clean ({worst_noise:.0f} dB avg) - raising cutoff reduces filter delay with no noise penalty"
            actions.append({"priority": prio, "urgency": urg, "category": "Filter",
                "action": f"Gyro lowpass filter: {direction} from {current_gyro_lp}Hz to {rec_gyro_lp}Hz",
                "param": "gyro_lowpass_hz", "current": current_gyro_lp, "new": rec_gyro_lp,
                "reason": reason})
        elif current_gyro_lp is None and worst_noise > ok_noise:
            actions.append({"priority": prio, "urgency": urg, "category": "Filter",
                "action": f"Set gyro_lowpass_hz to {rec_gyro_lp}",
                "param": "gyro_lowpass_hz", "current": "unknown", "new": rec_gyro_lp,
                "reason": f"Significant noise - use .bbl file for current value reading"})

    # D-term lowpass
    current_dterm_lp = config.get("dterm_lpf_hz") if has_filters else None
    if dterm_results:
        max_dterm_rms = max(d["rms"] for d in dterm_results)
        dterm_noise_starts = []
        for dt in dterm_results:
            nm = dt["psd_db"] > -35
            if np.any(nm):
                dterm_noise_starts.append(float(dt["freqs"][np.argmax(nm)]))
        if dterm_noise_starts and max_dterm_rms > 30:
            dt_min, dt_max = profile["dterm_lpf_range"]
            dt_safety = profile["filter_safety"] * 0.875  # D-term uses tighter margin
            rec_dterm_lp = round(int(np.clip(min(dterm_noise_starts) * dt_safety, dt_min, dt_max)) / 5) * 5
            if current_dterm_lp is not None and abs(rec_dterm_lp - current_dterm_lp) > 10:
                actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                    "action": f"D-term lowpass filter: Reduce from {current_dterm_lp}Hz to {rec_dterm_lp}Hz",
                    "param": "dterm_lpf_hz", "current": current_dterm_lp, "new": rec_dterm_lp,
                    "reason": f"D-term amplifying noise (RMS={max_dterm_rms:.0f})"})
            elif current_dterm_lp is None:
                actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                    "action": f"Set dterm_lpf_hz to {rec_dterm_lp}",
                    "param": "dterm_lpf_hz", "current": "unknown", "new": rec_dterm_lp,
                    "reason": f"D-term noise high - use .bbl for current value"})

    # Dynamic notch / RPM filter recommendations
    all_peaks = []
    for nr in noise_results:
        if nr:
            for p in nr["peaks"]:
                if p["power_db"] > -20 and 50 < p["freq_hz"] < 500:
                    all_peaks.append(p)
    if all_peaks:
        dom_freqs = sorted(set(int(round(p["freq_hz"]/10)*10) for p in all_peaks))
        lowest_peak = min(p["freq_hz"] for p in all_peaks)

        dyn_en = config.get("dyn_notch_enabled")
        dyn_q = config.get("dyn_notch_q")
        dyn_min_hz = config.get("dyn_notch_min_hz")
        rpm_en = config.get("rpm_filter_enabled")

        if dyn_en in (None, "0", 0, "OFF"):
            # Dynamic notch is truly off - recommend enabling
            rec_min_hz = max(30, int(round(lowest_peak * 0.7 / 10) * 10))
            actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                "action": f"Enable dynamic notch filter (set dynamic_gyro_notch_enabled = ON, min_hz = {rec_min_hz})",
                "param": "dynamic_gyro_notch_enabled", "current": "OFF", "new": "ON",
                "reason": f"Noise peaks at: {', '.join(f'{f}Hz' for f in dom_freqs[:4])} - dynamic notch will track and attenuate these"})
        elif isinstance(dyn_min_hz, (int, float)) and dyn_min_hz > 0:
            # Dynamic notch is on - check if min_hz is too high to catch the lowest peaks
            if lowest_peak < dyn_min_hz * 1.1:
                rec_min_hz = max(30, int(round(lowest_peak * 0.7 / 10) * 10))
                actions.append({"priority": 3, "urgency": "RECOMMENDED", "category": "Filter",
                    "action": f"Lower dynamic_gyro_notch_min_hz: {int(dyn_min_hz)} → {rec_min_hz}",
                    "param": "dyn_notch_min_hz", "current": int(dyn_min_hz), "new": rec_min_hz,
                    "reason": f"Noise peak at {int(lowest_peak)}Hz is near/below current min_hz ({int(dyn_min_hz)}Hz) - notch can't track it"})

        # RPM filter recommendation (more effective than dynamic notch for motor noise)
        if rpm_en in (None, "0", 0, "OFF"):
            actions.append({"priority": 4, "urgency": "OPTIONAL", "category": "Filter",
                    "action": "Consider enabling RPM filter (set rpm_gyro_filter_enabled = ON)",
                    "param": "rpm_filter_enabled", "current": "OFF", "new": "ON",
                    "reason": "RPM filter tracks motor noise precisely - requires ESC telemetry wire connected to a UART"})

    # ═══ PID CHANGES (merged per axis) ═══
    for i, axis in enumerate(AXIS_NAMES):
        pid = pid_results[i] if i < len(pid_results) else None
        if pid is None:
            continue
        cur_p = config.get(f"{axis.lower()}_p")
        cur_i = config.get(f"{axis.lower()}_i")
        cur_d = config.get(f"{axis.lower()}_d")
        cur_ff = config.get(f"{axis.lower()}_ff")

        if has_config:
            rec = compute_recommended_pid(pid, cur_p, cur_i, cur_d, profile, current_ff=cur_ff)
            if rec and rec["changes"]:
                _os = pid["avg_overshoot_pct"] or 0
                _dl = pid["tracking_delay_ms"] or 0
                prio = 3 if (_os > bad_os or _dl > bad_dl) else 4
                urg = "IMPORTANT" if prio == 3 else None

                # Build a single merged description for all PID changes on this axis
                change_parts = []
                sub_actions = []  # for CLI generation
                for term in ["FF", "P", "D", "I"]:
                    if term in rec["changes"]:
                        ch = rec["changes"][term]
                        direction = "Reduce" if ch["new"] < ch["current"] else "Increase"
                        change_parts.append(f"{direction} {term} from {ch['current']} to {ch['new']}")
                        sub_actions.append({"param": f"{axis.lower()}_{term.lower()}",
                                           "current": ch["current"], "new": ch["new"]})

                action_text = f"{axis}: {', '.join(change_parts)}"

                # Use reasons from compute_recommended_pid (already has good descriptions)
                actions.append({"priority": prio, "urgency": urg, "category": "PID",
                    "action": action_text,
                    "param": sub_actions[0]["param"] if len(sub_actions) == 1 else f"{axis.lower()}_pid",
                    "current": sub_actions[0]["current"] if len(sub_actions) == 1 else "multiple",
                    "new": sub_actions[0]["new"] if len(sub_actions) == 1 else "multiple",
                    "sub_actions": sub_actions,
                    "reason": " ".join(rec["reasons"])})
        else:
            _os = pid["avg_overshoot_pct"] or 0
            _dl = pid["tracking_delay_ms"] or 0
            if _os > ok_os:
                actions.append({"priority": 3, "urgency": "IMPORTANT" if _os > bad_os else None,
                    "category": "PID",
                    "action": f"Reduce {axis} P by ~{int(_os/3)}% and increase D by ~10%",
                    "param": f"{axis.lower()}_p", "current": "unknown", "new": "see action",
                    "reason": f"Overshoot {_os:.0f}% - use .bbl file for exact values"})
            if _dl > ok_dl:
                actions.append({"priority": 3, "urgency": "IMPORTANT" if _dl > bad_dl else None,
                    "category": "PID",
                    "action": f"Increase {axis} P by ~{int(_dl/2)}%",
                    "param": f"{axis.lower()}_p", "current": "unknown", "new": "see action",
                    "reason": f"Delay {_dl:.0f}ms - use .bbl for exact values"})

    # ═══ MOTOR / MECHANICAL ═══
    if motor_analysis and not motor_analysis.get("idle_detected", False):
        for m in motor_analysis["motors"]:
            if m["saturation_pct"] > profile["motor_sat_warn"]:
                actions.append({"priority": 1 if m["saturation_pct"] > 15 else 3,
                    "urgency": "CRITICAL" if m["saturation_pct"] > 15 else "IMPORTANT",
                    "category": "Motor",
                    "action": f"Motor {m['motor']} saturating {m['saturation_pct']:.1f}% - reduce overall PID gains by 15%",
                    "param": "motor_saturation", "current": f"{m['saturation_pct']:.1f}%", "new": f"<{profile['motor_sat_ok']}%",
                    "reason": "Motor at 100% can't correct further - PID demands exceed physics"})

        if motor_analysis["worst_motor"] and motor_analysis["balance_spread_pct"] > profile["motor_imbal_warn"]:
            wm, wd = motor_analysis["worst_motor"], motor_analysis["worst_direction"]
            sp = motor_analysis["balance_spread_pct"]
            actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Mechanical",
                "action": f"Check Motor {wm} - running {wd} ({sp:.1f}% imbalance)",
                "param": "motor_balance", "current": f"{sp:.1f}%", "new": f"<{profile['motor_imbal_ok']}%",
                "reason": "Bent prop, bad motor, loose mount, or CG offset. Fix mechanics before tuning PIDs."})
        elif motor_analysis["worst_motor"] and motor_analysis["balance_spread_pct"] > profile["motor_imbal_ok"]:
            wm = motor_analysis["worst_motor"]
            sp = motor_analysis["balance_spread_pct"]
            actions.append({"priority": 5, "urgency": None, "category": "Mechanical",
                "action": f"Minor imbalance ({sp:.1f}%) - Motor {wm} is the outlier",
                "param": "motor_balance", "current": f"{sp:.1f}%", "new": f"<{profile['motor_imbal_ok']}%",
                "reason": "Check prop balance and CG"})

    actions.sort(key=lambda a: a["priority"])

    # ═══ OSCILLATION-FIRST ENFORCEMENT ═══
    # When hover oscillation is detected, the oscillation actions already include
    # the right P/D reductions. Defer regular PID actions to avoid duplicates.
    osc_actions = [a for a in actions if a.get("category") == "Oscillation"
                   and a.get("urgency") in ("CRITICAL", "IMPORTANT")]
    if osc_actions:
        # Get axes that have oscillation actions
        osc_axes = set()
        for a in osc_actions:
            for axis in AXIS_NAMES:
                if a["action"].startswith(axis + ":"):
                    osc_axes.add(axis.lower())
        # Defer PID actions on those axes
        for a in actions:
            if a.get("category") == "PID" and not a.get("deferred"):
                # Check if this PID action is for an oscillating axis
                action_axis = a["action"].split(":")[0].strip().lower() if ":" in a["action"] else ""
                if action_axis in osc_axes:
                    a["deferred"] = True
                    a["urgency"] = None
                    a["original_action"] = a["action"]
                    a["action"] = f"[DEFERRED] {a['action']}"
                    a["reason"] = (
                        "PID changes deferred - fix hover oscillation first. The oscillation "
                        "actions above address the same axis with more aggressive corrections.")

    # ═══ FILTER-FIRST ENFORCEMENT ═══
    # When filter changes are recommended, PID measurements are unreliable because
    # noise bleeds into the PID loop and inflates overshoot/delay readings.
    # Raising D with a wide-open dterm LPF amplifies noise and makes things worse.
    # Strategy: only output filter changes, tell user to re-fly for PID tuning.
    filter_actions = [a for a in actions if a.get("category") == "Filter"
                      and a.get("urgency") in ("CRITICAL", "IMPORTANT")]
    pid_actions = [a for a in actions if a.get("category") == "PID"]

    if filter_actions and pid_actions:
        # Mark PID actions as deferred - they'll show as informational, not actionable
        for a in pid_actions:
            a["deferred"] = True
            a["urgency"] = None
            a["original_action"] = a["action"]
            a["action"] = f"[DEFERRED] {a['action']}"
            a["reason"] = (
                "PID changes deferred until filters are fixed. Current overshoot/delay readings "
                "are inflated by noise passing through the filter stack. Fix filters first, "
                "re-fly, then the analyzer will give accurate PID recommendations.")

        # Add explicit guidance
        actions.append({
            "priority": 2, "urgency": "IMPORTANT", "category": "Workflow",
            "action": "Fix filters first, then re-fly for PID tuning",
            "param": "workflow", "current": "N/A", "new": "N/A",
            "reason": "Noise is corrupting PID measurements. Apply the filter changes above, "
                      "fly one pack, then run the analyzer again for accurate PID recommendations. "
                      "Do NOT change PIDs and filters at the same time."})
        actions.sort(key=lambda a: a["priority"])

    # ═══ QUALITY SCORE ═══
    good_noise = profile["good_noise_db"]
    good_os = profile["good_overshoot"]
    good_dl = profile["good_delay_ms"]

    noise_scores = []
    for nr in noise_results:
        if nr:
            s = np.clip((nr["rms_high"] - bad_noise) / (good_noise - bad_noise) * 100, 0, 100)
            noise_scores.append(s)
    pid_scores = []
    pid_measurable = False
    for pid in pid_results:
        if pid:
            _os = pid["avg_overshoot_pct"]
            _dl = pid["tracking_delay_ms"]
            components = []
            if _os is not None:
                os_s = 100 - np.clip((_os - good_os) / (bad_os - good_os) * 100, 0, 100)
                components.append(os_s)
            if _dl is not None:
                dl_s = 100 - np.clip((_dl - good_dl) / (bad_dl - good_dl) * 100, 0, 100)
                components.append(dl_s)
            if components:
                pid_scores.append(np.mean(components))
                pid_measurable = True

    # ── Oscillation score from hover detection ──
    # Uses the structured hover_osc results from detect_hover_oscillation()
    gyro_oscillation_score = None
    has_oscillation = False
    if hover_osc:
        osc_scores = []
        for osc in hover_osc:
            rms = osc["gyro_rms"]
            sev = osc["severity"]
            cause = osc.get("cause")

            # Wind buffeting gets a much softer penalty — it's environmental,
            # not a tuning problem. Don't tank the score for flying in wind.
            if cause == "wind_buffeting":
                osc_scores.append(max(60, 100 - rms * 2))  # Floor at 60
                continue

            if sev == "none":
                osc_scores.append(100)
            elif sev == "mild":
                osc_scores.append(100 - np.clip((rms - 2) / (5 - 2) * 40, 0, 40))  # 60-100
            elif sev == "moderate":
                osc_scores.append(100 - np.clip((rms - 5) / (15 - 5) * 50 + 40, 40, 90))  # 10-60
                has_oscillation = True
            else:  # severe
                osc_scores.append(max(0, 10 - rms))  # 0-10
                has_oscillation = True
        if osc_scores:
            gyro_oscillation_score = float(min(osc_scores))  # Worst axis drives the score

    motor_score = 100
    if motor_analysis and not motor_analysis.get("idle_detected", False):
        sat_s = [max(0, 100 - m["saturation_pct"] * 10) for m in motor_analysis["motors"]]
        bal_s = max(0, 100 - motor_analysis["balance_spread_pct"] * 8)
        motor_score = (np.mean(sat_s) + bal_s) / 2

    # Build overall score
    # Four components: noise, PID (or oscillation proxy), motors, and oscillation penalty
    noise_component = np.mean(noise_scores) if noise_scores else 50

    if pid_measurable:
        pid_component = np.mean(pid_scores)
    elif gyro_oscillation_score is not None:
        pid_component = gyro_oscillation_score
    else:
        pid_component = 50

    # Oscillation penalty: severe hover oscillation drags down overall score directly
    osc_penalty = 0
    if has_oscillation and gyro_oscillation_score is not None:
        osc_penalty = max(0, (50 - gyro_oscillation_score) * 0.5)  # Up to 25 point penalty

    overall = float(np.mean([noise_component, pid_component, motor_score])) - osc_penalty

    if not pid_measurable:
        overall = min(overall, 65)  # Can't score higher than 65 without PID data

    overall = max(0, min(100, overall))

    scores = {"overall": overall,
              "noise": float(np.mean(noise_scores)) if noise_scores else None,
              "pid": float(np.mean(pid_scores)) if pid_scores else None,
              "pid_measurable": pid_measurable,
              "gyro_oscillation": gyro_oscillation_score,
              "hover_osc": hover_osc,
              "motor": float(motor_score)}

    critical_actions = [a for a in actions if a["urgency"] in ("CRITICAL", "IMPORTANT")]
    if not pid_measurable:
        # Can't verify tune - need more stick data
        if overall >= 50:
            verdict, vtext = "NEED_DATA", t("verdict.need_data_good")
        else:
            verdict, vtext = "NEED_DATA", t("verdict.need_data_bad")
    elif overall >= 85 and len(critical_actions) == 0:
        verdict, vtext = "DIALED_IN", t("verdict.dialed_in")
    elif overall >= 75 and len(critical_actions) == 0:
        verdict, vtext = "NEARLY_THERE", t("verdict.nearly_there")
    elif overall >= 60:
        verdict, vtext = "GETTING_BETTER", t("verdict.getting_better")
    elif overall >= 40:
        verdict, vtext = "NEEDS_WORK", t("verdict.needs_work")
    else:
        verdict, vtext = "ROUGH", t("verdict.rough")

    return {"actions": actions, "info": info_items, "scores": scores, "verdict": verdict, "verdict_text": vtext}


# ─── Tuning Recipe Engine ─────────────────────────────────────────────────────

def generate_tuning_recipe(noise_results, noise_fp, config, profile, accel_vib=None):
    """Generate a holistic filter stack recommendation based on the overall noise picture.

    Instead of tweaking one filter at a time, analyzes the noise sources and
    recommends a complete filter configuration as a coherent "recipe."

    Returns dict with:
        recipe_name (str): Short label for the recipe type
        description (str): What this recipe does
        stack (dict): {param: value} — complete filter settings
        cli_commands (list): Ready-to-paste CLI commands
        reasoning (list): Why each choice was made
    """
    frame_inches = profile.get("frame_inches", 5)
    reasoning = []

    # Detect dominant noise characteristics
    has_prop_harmonics = False
    has_electrical = False
    has_propwash = False
    noise_floor_db = -60
    dominant_freq = None

    if noise_fp and noise_fp.get("peaks"):
        for pk in noise_fp["peaks"]:
            src = pk.get("source", "")
            if "prop" in src and "wash" not in src:
                has_prop_harmonics = True
            elif "electrical" in src:
                has_electrical = True
            elif "propwash" in src:
                has_propwash = True

    # Compute average noise floor from results
    for nr in noise_results:
        if nr:
            noise_floor_db = max(noise_floor_db, nr.get("rms_high", -60))

    # Current filter state
    current_gyro_lpf = config.get("gyro_lowpass_hz", 100)
    current_dterm_lpf = config.get("dterm_lpf_hz", 100)
    rpm_enabled = config.get("rpm_filter_enabled") not in (None, 0, "0", "OFF")
    dyn_notch_enabled = config.get("dyn_notch_enabled") not in (None, 0, "0", "OFF")

    # Has accel vibration issues?
    has_structural = False
    if accel_vib and accel_vib.get("score", 100) < 70:
        has_structural = True

    # ── Select recipe ──
    stack = {}

    if rpm_enabled and noise_floor_db < -45:
        # Clean with RPM filter — open up filters
        recipe_name = "RPM Clean"
        description = ("RPM filter is handling motor harmonics. Open up gyro and D-term LPF "
                       "for minimum delay. Keep dynamic notch as backup for non-harmonic noise.")
        target_gyro = min(profile["gyro_lpf_range"][1], max(100, int(current_gyro_lpf * 1.2)))
        target_dterm = max(60, int(target_gyro * 0.7))
        stack = {
            "gyro_main_lpf_hz": target_gyro,
            "dterm_lpf_hz": target_dterm,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_mode": "3D",
            "dynamic_gyro_notch_min_hz": max(50, profile["gyro_lpf_range"][0]),
        }
        reasoning.append(f"RPM filter active — gyro LPF can go higher ({target_gyro}Hz) for less delay")
        reasoning.append(f"D-term PT3 at {target_dterm}Hz — smooth D without excessive lag")

    elif has_prop_harmonics and not rpm_enabled:
        # Strong prop harmonics, no RPM filter — rely on dynamic notch + tight LPF
        recipe_name = "Harmonic Defense"
        description = ("Strong prop/motor harmonics without RPM filter. Dynamic notch tracks "
                       "the harmonics, tight LPF catches the rest. Consider wiring ESC telemetry "
                       "for RPM filter — it would allow much higher LPF and better response.")
        target_gyro = max(profile["gyro_lpf_range"][0], min(80, int(current_gyro_lpf * 0.85)))
        target_dterm = max(45, int(target_gyro * 0.65))
        stack = {
            "gyro_main_lpf_hz": target_gyro,
            "dterm_lpf_hz": target_dterm,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_mode": "3D",
            "dynamic_gyro_notch_min_hz": max(40, profile["gyro_lpf_range"][0] - 10),
            "dynamic_gyro_notch_q": 300,
        }
        reasoning.append(f"Prop harmonics detected — dynamic notch Q=300 for precise tracking")
        reasoning.append(f"Gyro LPF at {target_gyro}Hz — tight to catch residual noise")
        reasoning.append("Enable RPM filter for best results (requires ESC telemetry UART)")

    elif has_propwash and frame_inches >= 8:
        # Large frame propwash — can't filter, need to live with it
        recipe_name = "Large Frame Propwash"
        description = ("Propwash at sub-80Hz on a large frame. This is aerodynamic, not fixable "
                       "with filters. Keep LPF above the propwash frequency to avoid adding delay "
                       "that makes it worse. Fly smoothly and avoid aggressive descents.")
        target_gyro = max(70, current_gyro_lpf)
        target_dterm = max(55, int(target_gyro * 0.75))
        stack = {
            "gyro_main_lpf_hz": target_gyro,
            "dterm_lpf_hz": target_dterm,
            "dterm_lpf_type": "PT3",
        }
        reasoning.append(f"Propwash is aerodynamic on {frame_inches}\" — filtering it adds delay and makes it worse")
        reasoning.append(f"Gyro LPF at {target_gyro}Hz — above propwash band to preserve response")

    elif noise_floor_db > -35:
        # Very noisy — aggressive filtering needed
        recipe_name = "Noise Suppression"
        description = ("High overall noise floor. Aggressive filtering needed to protect motors. "
                       "Fix the noise source (vibration, wiring, prop balance) then re-analyze — "
                       "this recipe trades response for safety.")
        target_gyro = max(profile["gyro_lpf_range"][0], min(60, int(current_gyro_lpf * 0.7)))
        target_dterm = max(40, int(target_gyro * 0.6))
        stack = {
            "gyro_main_lpf_hz": target_gyro,
            "dterm_lpf_hz": target_dterm,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_mode": "3D",
            "dynamic_gyro_notch_min_hz": max(30, profile["gyro_lpf_range"][0] - 15),
            "dynamic_gyro_notch_q": 200,
        }
        reasoning.append(f"High noise floor ({noise_floor_db:.0f}dB) — aggressive LPF needed")
        reasoning.append("Fix noise source first: prop balance, motor wiring, soft mount")

    else:
        # Reasonably clean — balanced setup
        recipe_name = "Balanced"
        description = "Noise is manageable. Balanced filter setup for good response with adequate filtering."
        target_gyro = max(profile["gyro_lpf_range"][0], min(profile["gyro_lpf_range"][1],
                          int((current_gyro_lpf + profile["gyro_lpf_range"][1]) / 2)))
        target_dterm = max(50, int(target_gyro * 0.7))
        stack = {
            "gyro_main_lpf_hz": target_gyro,
            "dterm_lpf_hz": target_dterm,
            "dterm_lpf_type": "PT3",
        }
        reasoning.append(f"Clean noise profile — balanced LPF at {target_gyro}Hz")

    if has_structural:
        reasoning.append("Structural vibration detected — check frame hardware, soft-mount FC")

    # Generate CLI
    cli_commands = [f"set {k} = {v}" for k, v in stack.items()]
    cli_commands.append("save")

    return {
        "recipe_name": recipe_name,
        "description": description,
        "stack": stack,
        "cli_commands": cli_commands,
        "reasoning": reasoning,
    }


# ─── Power/Efficiency Analysis ───────────────────────────────────────────────

def analyze_power(data, sr, config=None):
    """Analyze battery voltage and current for efficiency metrics.

    Returns dict with:
        avg_current_a (float): Average current draw in amps
        avg_power_w (float): Average power in watts
        total_mah (float): Estimated mAh consumed
        total_wh (float): Estimated Wh consumed
        min_cell_v (float): Minimum cell voltage during flight
        avg_cell_v (float): Average cell voltage
        sag_v (float): Voltage sag (avg - min)
        efficiency (dict): Power vs throttle curve data
        findings (list): [{level, text, detail}]
    """
    results = {
        "avg_current_a": None, "avg_power_w": None,
        "total_mah": None, "total_wh": None,
        "min_cell_v": None, "avg_cell_v": None, "sag_v": None,
        "efficiency": None, "findings": [],
    }

    has_vbat = "vbat" in data
    has_amp = "amperage" in data

    if not has_vbat:
        return results

    vbat = data["vbat"]
    clean_v = vbat[~np.isnan(vbat)]
    if len(clean_v) < 100:
        return results

    # INAV vbat is in 0.01V units
    voltage = clean_v / 100.0

    # Detect cell count
    cells = config.get("_cell_count") if config else None
    if not cells:
        max_v = float(np.max(voltage))
        cells = round(max_v / 4.2)
        if cells < 1:
            cells = 1

    cell_voltage = voltage / cells
    min_cell = float(np.min(cell_voltage))
    avg_cell = float(np.mean(cell_voltage))
    sag = avg_cell - min_cell

    results["min_cell_v"] = round(min_cell, 2)
    results["avg_cell_v"] = round(avg_cell, 2)
    results["sag_v"] = round(sag, 2)

    if has_amp:
        amp = data["amperage"]
        clean_a = amp[~np.isnan(amp)]
        if len(clean_a) > 100:
            # INAV amperage is in 0.01A units
            current = clean_a / 100.0
            avg_current = float(np.mean(current))
            duration_s = len(clean_a) / sr

            # Match lengths for power calculation
            min_len = min(len(voltage), len(current))
            power = voltage[:min_len] * current[:min_len]
            avg_power = float(np.mean(power))

            total_mah = avg_current * (duration_s / 3600) * 1000
            total_wh = avg_power * (duration_s / 3600)

            results["avg_current_a"] = round(avg_current, 1)
            results["avg_power_w"] = round(avg_power, 1)
            results["total_mah"] = round(total_mah, 0)
            results["total_wh"] = round(total_wh, 2)

    # Findings
    if min_cell < 3.3:
        results["findings"].append({
            "level": "WARNING",
            "text": f"Cell voltage dropped to {min_cell:.2f}V — below safe LiPo minimum (3.3V)",
            "detail": "Landing voltage too low. Raise battery warning threshold or shorten flights.",
        })
    elif min_cell < 3.5:
        results["findings"].append({
            "level": "INFO",
            "text": f"Minimum cell voltage {min_cell:.2f}V — acceptable but not much margin",
            "detail": "Consider landing earlier for battery longevity.",
        })

    if sag > 0.5:
        results["findings"].append({
            "level": "WARNING",
            "text": f"Voltage sag {sag:.2f}V/cell — battery may be aging or undersized",
            "detail": "High sag means the pack can't deliver the current needed. "
                      "Check C-rating, consider a higher-capacity or higher-C pack.",
        })

    return results


# ─── Failsafe Event Reconstruction ───────────────────────────────────────────

def analyze_failsafe_events(data, sr):
    """Reconstruct failsafe events from slow frame flight mode flags.

    Detects transitions into and out of failsafe modes (RX_LOST,
    RTH triggered by failsafe) and builds a timeline.

    Returns dict with:
        events (list): [{start_s, end_s, duration_s, type, recovered}]
        total_failsafe_s (float): Total time in failsafe
        findings (list): [{level, text, detail}]
    """
    results = {"events": [], "total_failsafe_s": 0, "findings": []}

    slow_frames = data.get("_slow_frames", [])
    if not slow_frames:
        return results

    # INAV failsafe shows as specific flight mode bits or flags
    # flightModeFlags bit 7 = NAV_RTH, we also look for failsafe-specific flags
    MODE_RTH = 7
    MODE_ARM = 0

    # Track mode transitions to find failsafe patterns
    transitions = []
    for frame_idx, fields in slow_frames:
        mode_flags = fields.get("flightModeFlags")
        if mode_flags is None:
            for k in fields:
                if "flight" in k.lower() and "mode" in k.lower():
                    mode_flags = fields[k]
                    break
        if mode_flags is not None:
            try:
                flags = int(mode_flags)
                t_s = frame_idx / sr if sr > 0 else 0
                transitions.append({"time_s": t_s, "flags": flags, "idx": frame_idx})
            except (ValueError, TypeError):
                pass

    # Look for failsafe flags in event frames
    event_frames = data.get("_event_frames", [])
    for frame_idx, event_type, event_data in (event_frames or []):
        # Event type 15 = FLIGHT_MODE with failsafe flags
        if event_type == 15 and event_data:
            try:
                flags = int(event_data) if isinstance(event_data, (int, float)) else 0
                # Check for failsafe-related flags
                # Bit patterns vary by INAV version, but RX_LOSS typically
                # triggers an emergency RTH
            except (ValueError, TypeError):
                pass

    # Detect RTH that appears suddenly (not user-commanded)
    # Heuristic: RTH that starts without a preceding AUX channel change
    # is likely failsafe-triggered
    events = []
    in_rth = False
    rth_start = None

    for i, tr in enumerate(transitions):
        is_rth = bool(tr["flags"] & (1 << MODE_RTH))
        is_armed = bool(tr["flags"] & (1 << MODE_ARM))

        if is_rth and not in_rth:
            # RTH just started
            in_rth = True
            rth_start = tr["time_s"]

            # Check if this looks like failsafe (sudden, no user input pattern)
            # Simple heuristic: if there's no RC data change near this point
            # it's likely automatic
            is_failsafe = False  # would need RC data correlation for accurate detection

        elif not is_rth and in_rth:
            # RTH ended
            in_rth = False
            if rth_start is not None:
                duration = tr["time_s"] - rth_start
                events.append({
                    "start_s": rth_start,
                    "end_s": tr["time_s"],
                    "duration_s": duration,
                    "type": "RTH",
                    "recovered": True,
                })

        if not is_armed and in_rth:
            # Disarmed during RTH — landed
            in_rth = False
            if rth_start is not None:
                events.append({
                    "start_s": rth_start,
                    "end_s": tr["time_s"],
                    "duration_s": tr["time_s"] - rth_start,
                    "type": "RTH_LANDING",
                    "recovered": True,
                })

    results["events"] = events
    results["total_failsafe_s"] = sum(e["duration_s"] for e in events)

    if events:
        results["findings"].append({
            "level": "INFO",
            "text": f"{len(events)} RTH event(s) detected, total {results['total_failsafe_s']:.1f}s",
            "detail": "Check if these were intentional RTH commands or failsafe-triggered.",
        })

    return results


# ─── Propwash Scoring ─────────────────────────────────────────────────────────

def analyze_propwash(data, sr, profile=None):
    """Detect and score propwash oscillation during descents and direction reversals.

    Propwash happens when the quad descends through its own turbulent wake.
    It manifests as oscillation on pitch and roll during descents and throttle
    reversals. Separate from general noise analysis.

    Returns dict with:
        events (list): [{start_s, end_s, severity, axis, rms, freq_hz, vert_speed}]
        score (int): 0-100 propwash severity score
        worst_axis (str): Axis most affected
        findings (list): [{level, text, detail}]
    """
    results = {"events": [], "score": 100, "worst_axis": None, "findings": []}
    frame_inches = profile.get("frame_inches", 5) if profile else 5

    # Need throttle and gyro data
    if "throttle" not in data or "gyro_roll" not in data:
        return results

    throttle = data["throttle"]
    time_s = data.get("time_s")
    if time_s is None or len(time_s) < sr * 2:
        return results

    # Detect descent segments: throttle below hover (midpoint) for >0.3s
    # or vertical speed negative if nav data available
    thr_clean = throttle[~np.isnan(throttle)]
    if len(thr_clean) < sr:
        return results

    thr_mid = float(np.median(thr_clean))  # approximate hover throttle
    thr_low = thr_mid - 50  # threshold for "descending"

    # Window-based analysis: 0.5s windows
    window = int(sr * 0.5)
    step = int(sr * 0.25)
    events = []
    axis_rms_sum = {"Roll": 0, "Pitch": 0}
    axis_event_count = {"Roll": 0, "Pitch": 0}

    for start in range(0, len(throttle) - window, step):
        end = start + window
        thr_seg = throttle[start:end]

        # Check if this is a descent segment (low throttle)
        thr_mean = float(np.nanmean(thr_seg))
        if thr_mean > thr_low:
            continue

        # Check for preceding high throttle (throttle reversal = propwash trigger)
        lookback = max(0, start - int(sr * 0.5))
        thr_before = float(np.nanmean(throttle[lookback:start])) if lookback < start else thr_mid
        if thr_before < thr_mid:
            continue  # no reversal, sustained low throttle

        # Analyze gyro during this descent window
        for axis in ["Roll", "Pitch"]:
            key = f"gyro_{axis.lower()}"
            if key not in data:
                continue
            gyro_seg = data[key][start:end]
            clean = gyro_seg[~np.isnan(gyro_seg)]
            if len(clean) < window // 2:
                continue

            rms = float(np.sqrt(np.mean(clean**2)))

            # Frame-scaled threshold
            rms_threshold = 8.0 * (frame_inches / 7.0)
            if rms < rms_threshold:
                continue

            # FFT to find propwash frequency
            from scipy.fft import rfft, rfftfreq
            freqs = rfftfreq(len(clean), 1.0 / sr)
            spec = np.abs(rfft(clean - np.mean(clean)))
            # Propwash is typically 20-80Hz
            band = (freqs >= 15) & (freqs <= 100)
            if not np.any(band):
                continue
            peak_freq = float(freqs[band][np.argmax(spec[band])])

            severity = "severe" if rms > rms_threshold * 3 else "moderate" if rms > rms_threshold * 1.5 else "mild"
            t_start = float(time_s[start]) if start < len(time_s) else start / sr

            events.append({
                "start_s": t_start,
                "end_s": t_start + window / sr,
                "severity": severity,
                "axis": axis,
                "rms": rms,
                "freq_hz": peak_freq,
                "vert_speed": thr_mid - thr_mean,  # proxy for descent rate
            })

            axis_rms_sum[axis] += rms
            axis_event_count[axis] += 1

    results["events"] = events

    if not events:
        results["findings"].append({
            "level": "OK",
            "text": "No significant propwash detected during descents",
            "detail": "",
        })
        return results

    # Score
    n_severe = sum(1 for e in events if e["severity"] == "severe")
    n_moderate = sum(1 for e in events if e["severity"] == "moderate")
    score = max(0, 100 - n_severe * 20 - n_moderate * 8 - len(events) * 2)
    results["score"] = score

    # Worst axis
    worst = max(axis_rms_sum, key=lambda a: axis_rms_sum[a] / max(axis_event_count[a], 1))
    results["worst_axis"] = worst

    avg_freq = np.mean([e["freq_hz"] for e in events])
    avg_rms = np.mean([e["rms"] for e in events])

    if n_severe > 0:
        frame_note = f"On {frame_inches}-inch, this is mostly aerodynamic — avoid aggressive descents. " if frame_inches >= 8 else ""
        results["findings"].append({
            "level": "WARNING",
            "text": f"Propwash: {len(events)} events ({n_severe} severe), "
                    f"worst on {worst} at {avg_freq:.0f}Hz, {avg_rms:.1f}°/s RMS",
            "detail": f"Propwash oscillation during throttle reversals and descents. "
                      f"{frame_note}"
                      f"RPM filter and lower D-term LPF can help. Smoother flying style reduces it.",
        })
    else:
        results["findings"].append({
            "level": "INFO",
            "text": f"Mild propwash: {len(events)} events on {worst} at {avg_freq:.0f}Hz",
            "detail": "Some propwash present during descents. Normal for most frames.",
        })

    return results


# ─── Chart Generation ─────────────────────────────────────────────────────────

def fig_to_base64(fig, dpi=120):
    buf = BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")

def setup_dark_style():
    plt.rcParams.update({
        "figure.facecolor": "#1a1b26", "axes.facecolor": "#1a1b26",
        "axes.edgecolor": "#565f89", "axes.labelcolor": "#c0caf5",
        "text.color": "#c0caf5", "xtick.color": "#565f89", "ytick.color": "#565f89",
        "grid.color": "#24283b", "grid.alpha": 0.6, "font.size": 10,
        "axes.titlesize": 13, "axes.grid": True,
    })

def create_noise_chart(noise_results):
    setup_dark_style()
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), sharey=True)
    fig.suptitle("Gyro Noise Spectrum", fontsize=14, color="#c0caf5", fontweight="bold", y=1.02)
    for i, nr in enumerate(noise_results):
        if nr is None: continue
        ax = axes[i]
        ax.plot(nr["freqs"], nr["psd_db"], color=AXIS_COLORS[i], linewidth=1.2, alpha=0.9)
        ax.fill_between(nr["freqs"], nr["psd_db"], -80, alpha=0.15, color=AXIS_COLORS[i])
        for peak in nr["peaks"][:3]:
            ax.axvline(peak["freq_hz"], color="#ff9e64", alpha=0.6, linestyle="--", linewidth=0.8)
            ax.annotate(f'{peak["freq_hz"]:.0f}Hz', xy=(peak["freq_hz"], peak["power_db"]),
                        fontsize=8, color="#ff9e64", ha="center", va="bottom", xytext=(0,8), textcoords="offset points")
        ax.set_title(nr["axis"], color=AXIS_COLORS[i], fontweight="bold")
        ax.set_xlabel("Frequency (Hz)")
        if i == 0: ax.set_ylabel("Power (dB)")
        ax.set_xlim(0, min(1000, nr["freqs"][-1]))
    fig.tight_layout()
    return fig_to_base64(fig)

def create_pid_response_chart(pid_results, sr):
    setup_dark_style()
    valid = [p for p in pid_results if p is not None]
    if not valid: return None
    fig, axes = plt.subplots(len(valid), 1, figsize=(16, 3.5*len(valid)), sharex=True)
    if len(valid) == 1: axes = [axes]
    fig.suptitle("PID Response - Setpoint vs Gyro", fontsize=14, color="#c0caf5", fontweight="bold", y=1.01)
    ws = int(sr * 2)
    for i, pid in enumerate(valid):
        ax = axes[i]
        ai = AXIS_NAMES.index(pid["axis"])
        sp, gy = pid["setpoint"], pid["gyro"]
        bs, bv = 0, 0
        for s in range(0, len(sp)-ws, ws//4):
            v = np.var(sp[s:s+ws])
            if v > bv: bv, bs = v, s
        sl = slice(bs, bs+ws)
        t = np.arange(ws)/sr*1000
        sps = sp[sl] if sl.stop <= len(sp) else sp[:ws]
        gys = gy[sl] if sl.stop <= len(gy) else gy[:ws]
        n = min(len(sps), len(gys), len(t))
        ax.plot(t[:n], sps[:n], color="#565f89", linewidth=1.5, alpha=0.8, label="Setpoint")
        ax.plot(t[:n], gys[:n], color=AXIS_COLORS[ai], linewidth=1.2, alpha=0.9, label="Gyro")
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        dl_str = f"{_dl:.1f}ms" if _dl is not None else "N/A"
        os_str = f"{_os:.1f}%" if _os is not None else "N/A"
        ax.set_title(f'{pid["axis"]} - Delay:{dl_str} | OS:{os_str}',
                      color=AXIS_COLORS[ai], fontweight="bold", fontsize=11)
        ax.set_ylabel("deg/s")
        ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")
    axes[-1].set_xlabel("Time (ms)")
    fig.tight_layout()
    return fig_to_base64(fig)

def create_motor_chart(motor_analysis, time_s):
    setup_dark_style()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 6), gridspec_kw={"height_ratios": [2,1]})
    fig.suptitle("Motor Output", fontsize=14, color="#c0caf5", fontweight="bold", y=1.01)
    ns = len(motor_analysis["motors"][0]["raw"])
    w = min(ns, 5000)
    bs, bv = 0, 0
    for m in motor_analysis["motors"]:
        for s in range(0, len(m["raw"])-w, w//4):
            v = np.var(m["normalized"][s:s+w])
            if v > bv: bv, bs = v, s
    for i, m in enumerate(motor_analysis["motors"]):
        sl = slice(bs, bs+w)
        t = time_s[sl] if sl.stop <= len(time_s) else time_s[:w]
        vals = m["normalized"][sl] if sl.stop <= len(m["normalized"]) else m["normalized"][:w]
        ax1.plot(t[:len(vals)], vals[:len(t)], color=MOTOR_COLORS[i], linewidth=0.8, alpha=0.8, label=f'M{m["motor"]}')
    ax1.axhline(95, color="#f7768e", linestyle="--", alpha=0.5)
    ax1.set_ylabel("Output (%)"); ax1.legend(loc="upper right", fontsize=8, ncol=5, facecolor="#1a1b26", edgecolor="#565f89")
    ax1.set_ylim(0, 105)
    names = [f'M{m["motor"]}' for m in motor_analysis["motors"]]
    avgs = [m["avg_pct"] for m in motor_analysis["motors"]]
    bars = ax2.bar(names, avgs, color=MOTOR_COLORS[:len(names)], alpha=0.8, width=0.5)
    ax2.set_ylabel("Avg (%)"); ax2.set_title(f'Spread: {motor_analysis["balance_spread_pct"]:.1f}%', fontsize=10, color="#7aa2f7")
    for b, v in zip(bars, avgs): ax2.text(b.get_x()+b.get_width()/2, b.get_height()+1, f'{v:.1f}%', ha="center", fontsize=9, color="#c0caf5")
    fig.tight_layout()
    return fig_to_base64(fig)

def create_dterm_chart(dterm_results):
    setup_dark_style()
    if not dterm_results: return None
    fig, axes = plt.subplots(1, len(dterm_results), figsize=(16, 4), sharey=True)
    if len(dterm_results) == 1: axes = [axes]
    fig.suptitle("D-Term Noise", fontsize=14, color="#c0caf5", fontweight="bold", y=1.02)
    for i, dt in enumerate(dterm_results):
        ax = axes[i]
        ai = AXIS_NAMES.index(dt["axis"])
        ax.plot(dt["freqs"], dt["psd_db"], color=AXIS_COLORS[ai], linewidth=1.2, alpha=0.9)
        ax.fill_between(dt["freqs"], dt["psd_db"], -80, alpha=0.12, color=AXIS_COLORS[ai])
        ax.set_title(f'{dt["axis"]} (RMS:{dt["rms"]:.1f})', color=AXIS_COLORS[ai], fontweight="bold")
        ax.set_xlabel("Hz"); ax.set_xlim(0, min(500, dt["freqs"][-1]))
        if i == 0: ax.set_ylabel("dB")
    fig.tight_layout()
    return fig_to_base64(fig)


# ─── Narrative & CLI Commands ────────────────────────────────────────────────

# Map from action plan param names to INAV CLI setting names
INAV_CLI_MAP = {
    "roll_p": "mc_p_roll", "roll_i": "mc_i_roll", "roll_d": "mc_d_roll", "roll_ff": "mc_cd_roll",
    "pitch_p": "mc_p_pitch", "pitch_i": "mc_i_pitch", "pitch_d": "mc_d_pitch", "pitch_ff": "mc_cd_pitch",
    "yaw_p": "mc_p_yaw", "yaw_i": "mc_i_yaw", "yaw_d": "mc_d_yaw", "yaw_ff": "mc_cd_yaw",
    "gyro_lowpass_hz": "gyro_main_lpf_hz", "gyro_lpf_hz": "gyro_main_lpf_hz",
    "gyro_lowpass2_hz": "gyro_main_lpf2_hz",
    "dterm_lpf_hz": "dterm_lpf_hz", "dterm_lpf2_hz": "dterm_lpf2_hz",
    "dyn_notch_min_hz": "dynamic_gyro_notch_min_hz",
    "dyn_notch_q": "dynamic_gyro_notch_q",
}

# Map for GUI (INAV Configurator) tab references
INAV_GUI_MAP = {
    "roll_p": ("PID Tuning", "Roll → P"),
    "roll_i": ("PID Tuning", "Roll → I"),
    "roll_d": ("PID Tuning", "Roll → D"),
    "roll_ff": ("PID Tuning", "Roll → FF"),
    "pitch_p": ("PID Tuning", "Pitch → P"),
    "pitch_i": ("PID Tuning", "Pitch → I"),
    "pitch_d": ("PID Tuning", "Pitch → D"),
    "pitch_ff": ("PID Tuning", "Pitch → FF"),
    "yaw_p": ("PID Tuning", "Yaw → P"),
    "yaw_i": ("PID Tuning", "Yaw → I"),
    "yaw_d": ("PID Tuning", "Yaw → D"),
    "yaw_ff": ("PID Tuning", "Yaw → FF"),
    "gyro_lowpass_hz": ("Filtering", "Gyro LPF Hz"),
    "dterm_lpf_hz": ("Filtering", "D-term LPF Hz"),
}


def generate_cli_commands(actions):
    """Generate INAV CLI commands from action plan."""
    # Use ordered dict to deduplicate - last value wins per param
    cmd_map = {}

    for a in actions:
        # Skip deferred actions (e.g., PID changes when filters need fixing first)
        if a.get("deferred"):
            continue
        # Handle merged PID/oscillation actions with sub_actions
        if "sub_actions" in a and a["sub_actions"]:
            for sa in a["sub_actions"]:
                param = sa.get("param", "")
                new_val = sa.get("new")
                if param in INAV_CLI_MAP and new_val is not None:
                    try:
                        cli_name = INAV_CLI_MAP[param]
                        cmd_map[cli_name] = int(new_val)
                    except (ValueError, TypeError):
                        pass
        else:
            param = a.get("param", "")
            new_val = a.get("new")
            if param in INAV_CLI_MAP and new_val is not None and new_val not in ("see action",):
                try:
                    cli_name = INAV_CLI_MAP[param]
                    cmd_map[cli_name] = int(new_val)
                except (ValueError, TypeError):
                    pass

    cmds = [f"set {k} = {v}" for k, v in cmd_map.items()]
    if cmds:
        cmds.append("save")
    return cmds


def generate_narrative(plan, pid_results, motor_analysis, noise_results, config, data, profile):
    """Generate a human-readable narrative about the quad's current state."""
    craft = config.get("craft_name", "your quad")
    fw = config.get("firmware_version", "")
    duration = data["time_s"][-1]
    sr = data["sample_rate"]
    overall = plan["scores"]["overall"]
    actions = plan["actions"]

    parts = []

    # Opening
    if overall >= 85:
        parts.append(f"{craft} is flying well. The tune is close to dialed in with a quality score of {overall:.0f}/100.")
    elif overall >= 60:
        parts.append(f"{craft} is flyable but has room for improvement (quality score: {overall:.0f}/100).")
    else:
        parts.append(f"{craft} needs significant tuning work (quality score: {overall:.0f}/100).")

    parts.append(f"This analysis is based on {duration:.0f} seconds of flight data at {sr:.0f}Hz.")

    # Short flight warning
    if duration < 5:
        parts.append(
            "WARNING: This flight is very short (under 5 seconds). "
            "Results may not be representative - longer flights with stick inputs provide much better data.")

    # Idle/ground detection warning
    if motor_analysis and motor_analysis.get("idle_detected", False):
        parts.append(
            "Motors were at idle throughout this log (likely armed on the ground without flying). "
            "Motor analysis is not meaningful for this flight.")

    # Hover oscillation (most critical - mention first)
    hover_osc_data = plan["scores"].get("hover_osc", [])
    # Filter out wind_buffeting — it's environmental, not a tuning problem
    osc_axes = [o for o in hover_osc_data
                if o["severity"] in ("moderate", "severe") and o.get("cause") != "wind_buffeting"
                ] if hover_osc_data else []
    wind_axes = [o for o in hover_osc_data
                 if o["severity"] in ("moderate", "severe") and o.get("cause") == "wind_buffeting"
                 ] if hover_osc_data else []

    if osc_axes:
        axis_names = [o["axis"] for o in osc_axes]
        worst = max(osc_axes, key=lambda o: o["gyro_rms"])
        if worst["severity"] == "severe":
            parts.append(
                f"CRITICAL: The quad is oscillating during hover on {'/'.join(axis_names)} "
                f"(worst: {worst['axis']} at {worst['gyro_rms']:.1f}\u00b0/s RMS). "
                f"This must be fixed before any other tuning - the quad is fighting itself just to stay in the air.")
        else:
            parts.append(
                f"The quad shows oscillation during hover on {'/'.join(axis_names)} "
                f"(worst: {worst['axis']} at {worst['gyro_rms']:.1f}\u00b0/s RMS). "
                f"This should be addressed first as it affects all other measurements.")
        if worst.get("dominant_freq_hz"):
            freq = worst["dominant_freq_hz"]
            if freq < 10:
                parts.append(f"The oscillation frequency (~{freq:.0f}Hz) points to P-term being too high.")
            elif freq < 25:
                parts.append(f"The oscillation frequency (~{freq:.0f}Hz) suggests P and D terms are fighting each other.")
            elif freq < 50:
                parts.append(f"The oscillation frequency (~{freq:.0f}Hz) indicates D-term is amplifying noise into the motors.")
            else:
                parts.append(f"The oscillation frequency (~{freq:.0f}Hz) suggests noise is leaking through the filters.")

    if wind_axes and not osc_axes:
        # Only wind buffeting detected — inform but don't alarm
        worst_w = max(wind_axes, key=lambda o: o["gyro_rms"])
        parts.append(
            f"Some low-frequency gyro activity detected on hover ({worst_w['gyro_rms']:.1f}\u00b0/s RMS at "
            f"~{worst_w['dominant_freq_hz']:.0f}Hz). On this frame size, this is likely wind buffeting "
            f"or GPS position corrections rather than a PID problem.")

    # PID behavior per axis
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]

    for pid in pid_results:
        if pid is None:
            continue
        axis = pid["axis"]
        os_pct = pid["avg_overshoot_pct"]
        delay = pid["tracking_delay_ms"]
        n_steps = pid.get("n_steps", 0)

        if os_pct is None and delay is None:
            parts.append(f"{axis} had insufficient stick activity to measure response ({n_steps} steps detected).")
            continue

        _os = os_pct or 0
        _dl = delay or 0

        if _os > bad_os and _dl > ok_dl:
            parts.append(
                f"{axis} has high overshoot ({_os:.0f}%) AND slow response ({_dl:.0f}ms). "
                f"This means the quad overshoots its target angle then takes too long to settle. "
                f"Lowering P and raising D will reduce the overshoot; the delay should improve once oscillation stops fighting itself.")
        elif _os > bad_os:
            parts.append(
                f"{axis} overshoot is very high at {_os:.0f}%. When you move the stick, "
                f"the quad swings past the target and bounces back. This is typically too much P gain. "
                f"Reducing P and increasing D will help it settle faster.")
        elif _os > ok_os:
            parts.append(
                f"{axis} has moderate overshoot ({_os:.0f}%). A small P reduction and D increase should tighten this up.")
        elif _dl > bad_dl:
            parts.append(
                f"{axis} response is sluggish ({_dl:.0f}ms delay). The quad takes too long to reach the commanded angle. "
                f"Increasing P gain will make it more responsive, but watch for overshoot increasing.")
        else:
            detail_parts = []
            if os_pct is not None: detail_parts.append(f"overshoot {_os:.0f}%")
            if delay is not None: detail_parts.append(f"delay {_dl:.0f}ms")
            parts.append(f"{axis} is tracking well ({', '.join(detail_parts)}).")

    # Noise
    noise_score = plan["scores"].get("noise")
    noise_fp = plan.get("noise_fingerprint")
    if noise_score is not None:
        if noise_score >= 90:
            parts.append("Gyro noise levels are clean - no major vibration issues detected.")
        elif noise_score >= 60:
            if noise_fp and noise_fp["peaks"]:
                parts.append(f"There is moderate noise in the gyro signal. {noise_fp['summary']}.")
            else:
                parts.append("There is moderate noise in the gyro signal. Lowering the gyro lowpass filter or enabling RPM filtering would help.")
        else:
            if noise_fp and noise_fp["peaks"]:
                parts.append(f"The gyro signal is noisy. Primary sources identified: {noise_fp['summary']}. "
                            "Addressing noise should be the top priority before fine-tuning PIDs.")
            else:
                parts.append("The gyro signal is noisy - this often comes from propeller vibrations, loose mounting, or motor issues. "
                            "Addressing noise should be the top priority before fine-tuning PIDs.")

    # Motors
    if motor_analysis:
        spread = motor_analysis.get("balance_spread_pct", 0)
        if spread > profile["motor_imbal_warn"]:
            wm = motor_analysis["worst_motor"]
            parts.append(
                f"Motor {wm} is working significantly harder than the others ({spread:.0f}% imbalance). "
                f"This usually indicates a bent prop, bad motor bearing, loose mount, or CG offset. "
                f"Fix the mechanical issue before adjusting PIDs - no amount of software tuning can compensate for hardware problems.")
        elif spread > profile["motor_imbal_ok"]:
            wm = motor_analysis["worst_motor"]
            parts.append(f"There's a minor motor imbalance ({spread:.0f}%) on Motor {wm}. Worth checking prop balance and CG position.")

    return " ".join(parts)


# ─── Terminal Output ──────────────────────────────────────────────────────────

def print_terminal_report(plan, noise_results, pid_results, motor_analysis, config, data, show_narrative=True, profile=None, noise_fp=None):
    R, B, C, G, Y, RED, DIM = _colors()
    scores = plan["scores"]
    overall = scores["overall"]

    print(f"\n{B}{C}{'═'*70}{R}")
    print(f"  {DIM}{datetime.now().strftime('%Y-%m-%d %H:%M')} | {data['time_s'][-1]:.1f}s | {data['sample_rate']:.0f}Hz | {data['n_rows']:,} rows{R}")
    if config.get("craft_name"): print(f"  {DIM}Craft: {config['craft_name']}{R}")

    # ── Ground-only flights: brief message, no fake analysis ──
    if plan["verdict"] == "GROUND_ONLY":
        print(f"\n  {DIM}{'░' * 20} - /100{R}")
        print(f"  {DIM}  Noise:- | PID:- | Motors:-{R}")
        print(f"\n  {Y}▸ {plan['verdict_text']}{R}")
        print(f"\n{B}{C}{'═'*70}{R}")
        return

    sc = G if overall >= 85 else Y if overall >= 60 else RED
    print(f"\n  {B}TUNE QUALITY: {sc}{'█'*int(overall/5)}{'░'*(20-int(overall/5))} {overall:.0f}/100{R}")
    parts = []
    if scores["noise"] is not None: parts.append(f"Noise:{scores['noise']:.0f}")
    if scores["pid"] is not None:
        parts.append(f"PID:{scores['pid']:.0f}")
    elif not scores.get("pid_measurable", True):
        parts.append(f"PID:N/A")
    motor_str = "N/A" if motor_analysis and motor_analysis.get("idle_detected", False) else f"{scores['motor']:.0f}"
    parts.append(f"Motors:{motor_str}")
    print(f"  {DIM}  {' | '.join(parts)}{R}")

    vc = {"DIALED_IN": G, "NEARLY_THERE": G, "GETTING_BETTER": Y, "NEEDS_WORK": Y, "ROUGH": RED, "NEED_DATA": Y, "GROUND_ONLY": DIM}
    print(f"\n  {B}{vc.get(plan['verdict'],C)}▸ {plan['verdict_text']}{R}")

    # ── Config mismatch warning (show early if significant) ──
    mismatches = config.get("_diff_mismatches", [])
    if len(mismatches) >= 3:
        print(f"\n  {RED}{B}⚠ STALE DATA:{R} {Y}FC config has changed since this flight "
              f"({len(mismatches)} parameters differ).{R}")
        print(f"  {DIM}  Analysis reflects what was flying, not the current FC config.{R}")
        print(f"  {DIM}  Fly again to get analysis of your current tune.{R}")

    # ── Current config + mismatches (compact) ──
    if config_has_pid(config):
        print(f"\n  {B}CONFIG:{R}", end="")
        if mismatches:
            print(f"  {DIM}(from blackbox headers — what was actually flying){R}")
        else:
            print()
        for ax in ["roll","pitch","yaw"]:
            pid_line = f"    {ax.capitalize():6s} P={config.get(f'{ax}_p','?'):>3}  I={config.get(f'{ax}_i','?'):>3}  D={config.get(f'{ax}_d','?'):>3}"
            # Show FC-current values inline if different
            changed = []
            for param in ['p', 'i', 'd']:
                key = f"{ax}_{param}"
                for mk, bb_val, diff_val in mismatches:
                    if mk == key:
                        changed.append(f"{param.upper()}→{diff_val}")
            if changed:
                pid_line += f"  {Y}(FC now: {', '.join(changed)}){R}"
            print(pid_line)
        if config_has_filters(config):
            filt_line = f"    Gyro LPF: {config.get('gyro_lowpass_hz','?')}Hz  D-term LPF: {config.get('dterm_lpf_hz','?')}Hz"
            filt_changes = []
            for mk, bb_val, diff_val in mismatches:
                if mk in ('gyro_lowpass_hz', 'dterm_lpf_hz'):
                    label = 'Gyro' if 'gyro' in mk else 'D-term'
                    filt_changes.append(f"{label}→{diff_val}Hz")
            if filt_changes:
                filt_line += f"  {Y}(FC now: {', '.join(filt_changes)}){R}"
            print(filt_line)

    # Show remaining mismatches not already shown inline (motor protocol, etc.)
    if mismatches:
        shown_inline = set()
        for ax in ["roll","pitch","yaw"]:
            for param in ['p', 'i', 'd']:
                shown_inline.add(f"{ax}_{param}")
        shown_inline.update(['gyro_lowpass_hz', 'dterm_lpf_hz'])
        remaining = [(k, bb, fc) for k, bb, fc in mismatches if k not in shown_inline]
        if remaining:
            for key, bb_val, diff_val in remaining:
                label = key.replace("_", " ").title()
                print(f"    {DIM}{label}: {bb_val} → {diff_val} (on FC){R}")

    # Narrative (skip if config is stale — it would just confuse)
    if show_narrative and profile and len(mismatches) < 3:
        narrative = generate_narrative(plan, pid_results, motor_analysis, noise_results, config, data, profile)
        # Only show if it adds something beyond the verdict line
        if len(narrative) > len(plan.get('verdict_text', '')) + 20:
            print(f"\n  {DIM}{'─'*68}{R}")
            import textwrap
            for line in textwrap.wrap(narrative, width=66):
                print(f"  {DIM}{line}{R}")

    # Informational items (not actions)
    info_items = plan.get("info", [])
    if info_items:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}NOTE:{R}")
        for item in info_items:
            print(f"  {DIM}ℹ {item['text']}{R}")
            print(f"    {DIM}{item['detail']}{R}")

    # Noise source fingerprinting
    if noise_fp and noise_fp["peaks"]:
        fp_text = format_noise_fingerprint_terminal(noise_fp, (R, B, C, G, Y, RED, DIM))
        print(fp_text)

    actions = plan["actions"]
    active_actions = [a for a in actions if not a.get("deferred")]
    deferred_actions = [a for a in actions if a.get("deferred")]

    if active_actions:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}DO THIS - {len(active_actions)} change{'s' if len(active_actions)!=1 else ''}:{R}")
        print(f"{B}{C}{'─'*70}{R}")
        for i, a in enumerate(active_actions, 1):
            us = ""
            if a["urgency"] == "CRITICAL": us = f" {RED}{B}[!!]{R}"
            elif a["urgency"] == "IMPORTANT": us = f" {Y}{B}[!]{R}"
            print(f"\n  {B}{C}{i}.{R} {B}{a['action']}{R}{us}")
            print(f"     {DIM}{a['reason']}{R}")

        # CLI commands (only from active actions)
        cli_cmds = generate_cli_commands(active_actions)
        if cli_cmds:
            print(f"\n{B}{C}{'─'*70}{R}")
            print(f"  {B}INAV CLI - paste into Configurator CLI tab:{R}")
            print()
            for cmd in cli_cmds:
                print(f"    {G}{cmd}{R}")
            print()
            # GUI hints from active actions (including sub_actions)
            gui_hints = []
            for a in active_actions:
                if "sub_actions" in a:
                    for sa in a["sub_actions"]:
                        param = sa.get("param", "")
                        if param in INAV_GUI_MAP:
                            tab, field = INAV_GUI_MAP[param]
                            gui_hints.append(f"• {tab} tab → {field} → {sa['new']}")
                else:
                    param = a.get("param", "")
                    if param in INAV_GUI_MAP:
                        tab, field = INAV_GUI_MAP[param]
                        new_val = a.get("new")
                        if new_val is not None and new_val not in ("see action",):
                            gui_hints.append(f"• {tab} tab → {field} → {new_val}")
            if gui_hints:
                print(f"  {DIM}Or apply manually in Configurator:{R}")
                for h in gui_hints:
                    print(f"    {DIM}{h}{R}")

    if deferred_actions:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}{Y}DEFERRED (apply after filter changes + re-fly):{R}")
        print(f"{B}{C}{'─'*70}{R}")
        for a in deferred_actions:
            orig = a.get("original_action", a["action"])
            print(f"    {DIM}⏸ {orig}{R}")
        print(f"\n  {DIM}These PID changes are on hold because noise is distorting the")
        print(f"  measurements. Fix filters, fly one pack, then re-analyze.{R}")

    if not active_actions and not deferred_actions:
        if plan["scores"].get("pid_measurable", True):
            print(f"\n  {G}{B}  ✓ {t('terminal.no_changes')}{R}")
        else:
            print(f"\n  {Y}{B}  ⚠ {t('terminal.need_stick_data')}{R}")
            print(f"  {DIM}  Fly with deliberate roll/pitch/yaw moves, then re-analyze.{R}")

    print(f"\n{B}{C}{'─'*70}{R}")
    print(f"  {B}MEASUREMENTS:{R}")

    # Hover oscillation (show first - most critical)
    hover_osc_data = plan["scores"].get("hover_osc", [])
    any_osc = any(o["severity"] != "none" for o in hover_osc_data) if hover_osc_data else False
    if hover_osc_data:
        for osc in hover_osc_data:
            sev = osc["severity"]
            rms = osc["gyro_rms"]
            p2p = osc["gyro_p2p"]
            freq = osc["dominant_freq_hz"]
            freq_str = f"  @{freq:.0f}Hz" if freq else ""
            if sev == "severe":
                sc2 = RED
                sev_str = f"{RED}{B}SEVERE{R}"
            elif sev == "moderate":
                sc2 = Y
                sev_str = f"{Y}MODERATE{R}"
            elif sev == "mild":
                sc2 = Y
                sev_str = f"{Y}mild{R}"
            else:
                sc2 = G
                sev_str = f"{G}stable{R}"
            print(f"    {osc['axis']:6s}  Hover: {sev_str}  RMS:{sc2}{rms:5.1f}°/s{R}  P2P:{sc2}{p2p:5.0f}°/s{R}{freq_str}")
        if any_osc:
            print(f"    {DIM}(hover oscillation measured during {hover_osc_data[0]['hover_seconds']:.1f}s of centered stick){R}")
        print()

    for pid in pid_results:
        if pid is None: continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        n_steps = pid.get("n_steps", 0)
        if _os is not None:
            oc = RED if _os>BAD_OVERSHOOT else Y if _os>OK_OVERSHOOT else G
            os_str = f"{oc}{_os:5.1f}%{R}"
        else:
            os_str = f"{DIM}  N/A{R}"
        if _dl is not None:
            dc = RED if _dl>BAD_DELAY_MS else Y if _dl>OK_DELAY_MS else G
            dl_str = f"{dc}{_dl:5.1f}ms{R}"
        else:
            dl_str = f"{DIM}  N/A{R}"
        step_hint = f"  {DIM}({n_steps} steps){R}" if n_steps < 5 and (_os is None or _dl is None) else ""
        print(f"    {pid['axis']:6s}  OS:{os_str}  Delay:{dl_str}  Err:{pid['rms_error']:.1f}{step_hint}")
    if motor_analysis:
        if motor_analysis.get("idle_detected", False):
            print(f"    Motors: {DIM}idle/ground (no throttle variation - skipping saturation analysis){R}")
        else:
            for m in motor_analysis["motors"]:
                sc2 = RED if m["saturation_pct"]>MOTOR_SAT_WARN else G
                print(f"    Motor {m['motor']}  Avg:{m['avg_pct']:5.1f}%  Sat:{sc2}{m['saturation_pct']:4.1f}%{R}")
    print(f"\n{B}{C}{'═'*70}{R}\n")


# ─── Interactive Report Menu ──────────────────────────────────────────────────

def _capture(fn, *args, **kwargs):
    """Capture stdout from a function call, return as string."""
    old_stdout = sys.stdout
    sys.stdout = buf = StringIO()
    try:
        fn(*args, **kwargs)
    finally:
        sys.stdout = old_stdout
    return buf.getvalue()


def _print_score_bar(plan, config, data):
    """Print the always-visible score bar and verdict."""
    R, B, C, G, Y, RED, DIM = _colors()
    scores = plan["scores"]
    overall = scores["overall"]

    print(f"  {DIM}{datetime.now().strftime('%Y-%m-%d %H:%M')} | {data['time_s'][-1]:.1f}s | "
          f"{data['sample_rate']:.0f}Hz | {data['n_rows']:,} rows{R}")
    if config.get("craft_name"):
        print(f"  {DIM}Craft: {config['craft_name']}{R}")

    if plan["verdict"] == "GROUND_ONLY":
        print(f"\n  {DIM}{'░' * 20} - /100{R}")
        print(f"  {DIM}  Noise:- | PID:- | Motors:-{R}")
        print(f"\n  {Y}▸ {plan['verdict_text']}{R}")
        return

    sc = G if overall >= 85 else Y if overall >= 60 else RED
    print(f"\n  {B}TUNE QUALITY: {sc}{'█'*int(overall/5)}{'░'*(20-int(overall/5))} {overall:.0f}/100{R}")
    parts = []
    if scores["noise"] is not None:
        parts.append(f"Noise:{scores['noise']:.0f}")
    if scores["pid"] is not None:
        parts.append(f"PID:{scores['pid']:.0f}")
    elif not scores.get("pid_measurable", True):
        parts.append(f"PID:N/A")
    motor_str = "N/A" if scores.get("motor") is None else f"{scores['motor']:.0f}"
    parts.append(f"Motors:{motor_str}")
    print(f"  {DIM}  {' | '.join(parts)}{R}")

    vc = {"DIALED_IN": G, "NEARLY_THERE": G, "GETTING_BETTER": Y,
          "NEEDS_WORK": Y, "ROUGH": RED, "NEED_DATA": Y, "GROUND_ONLY": DIM}
    print(f"\n  {vc.get(plan['verdict'], C)}▸ {plan['verdict_text']}{R}")


def _print_section_pid(plan, pid_results, config, profile):
    """Print PID tuning section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}PID RESPONSE:{R}")

    # Config display
    if config_has_pid(config):
        for ax in ["roll", "pitch", "yaw"]:
            ff = config.get(f"{ax}_ff")
            ff_str = f"  FF={ff:>3}" if ff is not None else ""
            print(f"    {ax.capitalize():6s} P={config.get(f'{ax}_p', '?'):>3}  "
                  f"I={config.get(f'{ax}_i', '?'):>3}  D={config.get(f'{ax}_d', '?'):>3}{ff_str}")

    # Step response measurements
    for pid in pid_results:
        if pid is None:
            continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        n_steps = pid.get("n_steps", 0)
        if _os is not None:
            oc = RED if _os > BAD_OVERSHOOT else Y if _os > OK_OVERSHOOT else G
            os_str = f"{oc}{_os:5.1f}%{R}"
        else:
            os_str = f"{DIM}  N/A{R}"
        if _dl is not None:
            dc = RED if _dl > BAD_DELAY_MS else Y if _dl > OK_DELAY_MS else G
            dl_str = f"{dc}{_dl:5.1f}ms{R}"
        else:
            dl_str = f"{DIM}  N/A{R}"
        step_hint = f"  {DIM}({n_steps} steps){R}" if n_steps < 5 and (_os is None or _dl is None) else ""
        print(f"    {pid['axis']:6s}  OS:{os_str}  Delay:{dl_str}  Err:{pid['rms_error']:.1f}{step_hint}")

    # PID-related actions
    actions = plan["actions"]
    pid_actions = [a for a in actions if a.get("category") == "PID" and not a.get("deferred")]
    osc_actions = [a for a in actions if a.get("category") == "Oscillation" and not a.get("deferred")]
    deferred = [a for a in actions if a.get("category") == "PID" and a.get("deferred")]

    if osc_actions:
        print(f"\n  {B}Oscillation actions:{R}")
        for a in osc_actions:
            us = f" {RED}[!!]{R}" if a["urgency"] == "CRITICAL" else f" {Y}[!]{R}" if a["urgency"] == "IMPORTANT" else ""
            print(f"    {a['action']}{us}")
    if pid_actions:
        print(f"\n  {B}PID actions:{R}")
        for a in pid_actions:
            print(f"    {a['action']}")
    if deferred:
        print(f"\n  {DIM}Deferred (fix filters first):{R}")
        for a in deferred:
            orig = a.get("original_action", a["action"])
            print(f"    {DIM}⏸ {orig}{R}")


def _print_section_noise(plan, noise_fp):
    """Print noise analysis section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}NOISE ANALYSIS:{R}")

    if noise_fp and noise_fp["peaks"]:
        fp_text = format_noise_fingerprint_terminal(noise_fp, (R, B, C, G, Y, RED, DIM))
        print(fp_text)
    else:
        print(f"  {G}  No significant noise sources detected.{R}")

    # Filter actions
    actions = plan["actions"]
    filter_actions = [a for a in actions if a.get("category") == "Filter" and not a.get("deferred")]
    if filter_actions:
        print(f"\n  {B}Filter actions:{R}")
        for a in filter_actions:
            us = f" {RED}[!!]{R}" if a["urgency"] == "CRITICAL" else f" {Y}[!]{R}" if a["urgency"] == "IMPORTANT" else ""
            print(f"    {a['action']}{us}")
            print(f"    {DIM}{a['reason']}{R}")


def _print_section_hover(plan, motor_analysis):
    """Print hover stability and motor section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}HOVER & STABILITY:{R}")

    # Hover oscillation
    hover_osc_data = plan["scores"].get("hover_osc", [])
    if hover_osc_data:
        for osc in hover_osc_data:
            sev = osc["severity"]
            rms = osc["gyro_rms"]
            p2p = osc["gyro_p2p"]
            freq = osc["dominant_freq_hz"]
            cause = osc.get("cause", "")
            freq_str = f"  @{freq:.0f}Hz" if freq else ""
            cause_str = f"  [{cause.replace('_', ' ')}]" if cause else ""
            if sev == "severe":
                sev_str = f"{RED}{B}SEVERE{R}"
            elif sev == "moderate":
                sev_str = f"{Y}MODERATE{R}"
            elif sev == "mild":
                sev_str = f"{Y}mild{R}"
            else:
                sev_str = f"{G}stable{R}"
            print(f"    {osc['axis']:6s}  {sev_str}  RMS:{rms:5.1f} deg/s  P2P:{p2p:5.0f} deg/s{freq_str}{DIM}{cause_str}{R}")
        any_osc = any(o["severity"] != "none" for o in hover_osc_data)
        if any_osc:
            print(f"    {DIM}(measured during {hover_osc_data[0]['hover_seconds']:.1f}s centered stick){R}")
    else:
        print(f"  {DIM}  No hover data available (need centered-stick segments).{R}")

    # Info items (wind buffeting notes)
    info_items = plan.get("info", [])
    if info_items:
        print()
        for item in info_items:
            print(f"  {DIM}ℹ {item['text']}{R}")
            print(f"    {DIM}{item['detail']}{R}")

    # Motors
    print()
    if motor_analysis:
        if motor_analysis.get("idle_detected", False):
            print(f"  {B}Motors:{R} {DIM}idle/ground{R}")
        else:
            print(f"  {B}Motor balance:{R}")
            for m in motor_analysis["motors"]:
                sc2 = RED if m["saturation_pct"] > MOTOR_SAT_WARN else G
                print(f"    Motor {m['motor']}  Avg:{m['avg_pct']:5.1f}%  Sat:{sc2}{m['saturation_pct']:4.1f}%{R}")
            sp = motor_analysis.get("balance_spread_pct", 0)
            if sp > 5:
                wm = motor_analysis.get("worst_motor")
                print(f"    {Y}Imbalance: {sp:.1f}% — Motor {wm} is the outlier{R}")

    # Mechanical actions
    mech_actions = [a for a in plan["actions"]
                    if a.get("category") in ("Mechanical", "Motor") and not a.get("deferred")]
    if mech_actions:
        print(f"\n  {B}Mechanical:{R}")
        for a in mech_actions:
            print(f"    {a['action']}")


def _print_section_nav(nav_perf):
    """Print nav controller performance section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}NAV PERFORMANCE:{R}")

    if not nav_perf or not nav_perf.get("findings"):
        print(f"  {DIM}  No nav performance data (need POSHOLD/RTH flight segments).{R}")
        return

    # Score
    nav_score = nav_perf.get("score")
    if nav_score is not None:
        sc = G if nav_score >= 85 else Y if nav_score >= 60 else RED
        print(f"  {sc}  Nav score: {nav_score:.0f}/100{R}")

    # Deceleration events
    decel = nav_perf.get("deceleration", [])
    if decel:
        avg_peak = np.mean([e["peak_error_cm"] for e in decel])
        avg_osc = np.mean([e["oscillation_count"] for e in decel])
        avg_settle = np.mean([e["settling_time_s"] for e in decel])
        print(f"\n  {B}Deceleration ({len(decel)} events):{R}")
        print(f"    Avg overshoot: {avg_peak:.0f}cm | Oscillations: {avg_osc:.1f} | Settling: {avg_settle:.1f}s")
        worst = max(decel, key=lambda e: e["peak_error_cm"])
        print(f"    Worst: {worst['peak_error_cm']:.0f}cm at {worst['time_s']:.1f}s")

    # Position hold
    ph = nav_perf.get("poshold")
    if ph:
        cep_c = G if ph["cep_cm"] < 150 else Y if ph["cep_cm"] < 400 else RED
        print(f"\n  {B}Position hold ({ph['hold_duration_s']:.0f}s):{R}")
        print(f"    CEP: {cep_c}{ph['cep_cm']:.0f}cm{R}  Max drift: {ph['max_drift_cm']:.0f}cm  "
              f"RMS: {ph['rms_error_cm']:.0f}cm")
        if ph.get("toilet_bowl"):
            print(f"    {RED}Toilet bowl detected (period {ph['tb_period_s']:.1f}s) — check compass{R}")

    # Altitude hold
    ah = nav_perf.get("althold")
    if ah:
        alt_c = G if ah["rms_error_cm"] < 50 else Y if ah["rms_error_cm"] < 150 else RED
        print(f"\n  {B}Altitude hold ({ah['hold_duration_s']:.0f}s):{R}")
        print(f"    RMS: {alt_c}{ah['rms_error_cm']:.0f}cm{R}  Max: {ah['max_error_cm']:.0f}cm")
        if ah.get("oscillation"):
            print(f"    {Y}Oscillation at {ah['osc_freq_hz']:.1f}Hz{R}")

    # Saturation
    sat = nav_perf.get("saturation")
    if sat and max(sat["sat_pct_n"], sat["sat_pct_e"]) > 5:
        print(f"\n  {Y}Nav controller saturating: N={sat['sat_pct_n']:.0f}% E={sat['sat_pct_e']:.0f}%{R}")

    # Wind correlation
    wind = nav_perf.get("wind_correlation")
    if wind and not np.isnan(wind.get("correlation", float("nan"))):
        print(f"\n  {DIM}Wind: avg {wind['avg_wind_cms']/100:.1f}m/s, "
              f"error correlation {wind['correlation']:.0%}{R}")

    # Findings
    for f in nav_perf.get("findings", []):
        if f["level"] == "WARNING":
            print(f"\n  {Y}⚠ {f['text']}{R}")
        elif f["level"] == "INFO":
            print(f"\n  {DIM}ℹ {f['text']}{R}")
        elif f["level"] == "OK":
            print(f"\n  {G}✓ {f['text']}{R}")
        if f.get("detail"):
            print(f"    {DIM}{f['detail']}{R}")

    # Nav actions
    nav_actions = nav_perf.get("nav_actions", [])
    if nav_actions:
        print(f"\n  {B}Nav PID recommendations:{R}")
        for a in nav_actions:
            print(f"    {a['action']}")
            print(f"    {DIM}{a['reason']}{R}")
            print(f"    {G}{a['cli']}{R}")


def _print_section_nav_sensors(nav_results):
    """Print nav sensor health section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}NAV SENSORS:{R}")

    if not nav_results:
        print(f"  {DIM}  No nav sensor data available.{R}")
        return

    for section_name in ["compass", "gps", "baro", "estimator"]:
        section = nav_results.get(section_name)
        if not section:
            continue
        score = section.get("score")
        sc = G if (score or 0) >= 80 else Y if (score or 0) >= 50 else RED
        print(f"\n  {B}{section_name.capitalize()}: {sc}{score}/100{R}")
        for f in section.get("findings", []):
            level, text = f if isinstance(f, tuple) else (f.get("level", "INFO"), f.get("text", ""))
            if level == "WARNING":
                print(f"    {Y}⚠ {text}{R}")
            elif level == "CRITICAL":
                print(f"    {RED}✗ {text}{R}")
            elif level == "OK":
                print(f"    {G}✓ {text}{R}")
            else:
                print(f"    {DIM}ℹ {text}{R}")


def _print_section_cli(plan):
    """Print CLI commands section."""
    R, B, C, G, Y, RED, DIM = _colors()
    active_actions = [a for a in plan["actions"] if not a.get("deferred")]
    cli_cmds = generate_cli_commands(active_actions)

    print(f"\n  {B}{'─'*66}{R}")
    if cli_cmds:
        print(f"  {B}INAV CLI — paste into Configurator CLI tab:{R}")
        print()
        for cmd in cli_cmds:
            print(f"    {G}{cmd}{R}")
        print()

        # GUI hints
        gui_hints = []
        for a in active_actions:
            if "sub_actions" in a and a["sub_actions"]:
                for sa in a["sub_actions"]:
                    param = sa.get("param", "")
                    if param in INAV_GUI_MAP:
                        tab, field = INAV_GUI_MAP[param]
                        gui_hints.append(f"  {tab} tab -> {field} -> {sa['new']}")
            else:
                param = a.get("param", "")
                if param in INAV_GUI_MAP:
                    tab, field = INAV_GUI_MAP[param]
                    new_val = a.get("new")
                    if new_val is not None and new_val not in ("see action",):
                        gui_hints.append(f"  {tab} tab -> {field} -> {new_val}")
        if gui_hints:
            print(f"  {DIM}Or apply in Configurator:{R}")
            for h in gui_hints:
                print(f"    {DIM}{h}{R}")
    else:
        print(f"  {G}  No changes needed — nothing to paste.{R}")


def _sparkline(values, width=30, min_val=0, max_val=100):
    """Render an ASCII sparkline from a list of values.

    Returns a string like: ▁▂▃▅▇█▇▅▃
    """
    blocks = " ▁▂▃▄▅▆▇█"
    if not values:
        return ""
    vmin = min_val if min_val is not None else min(values)
    vmax = max_val if max_val is not None else max(values)
    if vmax <= vmin:
        return blocks[4] * len(values)

    result = []
    for v in values:
        if v is None:
            result.append(" ")
            continue
        normalized = (v - vmin) / (vmax - vmin)
        idx = int(normalized * (len(blocks) - 1))
        idx = max(0, min(len(blocks) - 1, idx))
        result.append(blocks[idx])
    return "".join(result)


def _print_section_history(prog, flight_diff=None):
    """Print flight history with sparklines and trend."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}FLIGHT HISTORY:{R}")

    flights = prog.get("flights", [])
    if not flights:
        print(f"  {DIM}  No flight history available.{R}")
        return

    # Score sparkline
    scores = [f.get("score") for f in flights]
    valid_scores = [s for s in scores if s is not None]

    trend_icon = {"improving": f"{G}↗ Improving", "degrading": f"{RED}↘ Degrading",
                  "stable": f"{Y}→ Stable", "insufficient": f"{DIM}? First flight"}.get(prog.get("trend", ""), "")
    print(f"\n  Trend: {trend_icon}{R}")

    if len(valid_scores) >= 2:
        spark = _sparkline(valid_scores)
        print(f"\n  {B}Score:{R}  {spark}  {DIM}{valid_scores[0]:.0f} → {valid_scores[-1]:.0f}{R}")

        # Sub-score sparklines
        noise_scores = [f.get("noise") for f in flights]
        pid_scores = [f.get("pid") for f in flights]
        motor_scores = [f.get("motor") for f in flights]

        if any(s is not None for s in noise_scores):
            spark_n = _sparkline([s if s is not None else 0 for s in noise_scores])
            print(f"  {DIM}Noise:{R}  {spark_n}")
        if any(s is not None for s in pid_scores):
            spark_p = _sparkline([s if s is not None else 0 for s in pid_scores])
            print(f"  {DIM}PID:  {R}  {spark_p}")
        if any(s is not None for s in motor_scores):
            spark_m = _sparkline([s if s is not None else 0 for s in motor_scores])
            print(f"  {DIM}Motor:{R}  {spark_m}")

    # Flight table
    print(f"\n  {B}{'#':>3}  {'Score':>6}  {'Noise':>6}  {'PID':>6}  {'Duration':>8}  Verdict{R}")
    print(f"  {DIM}{'─'*60}{R}")
    for f in flights:
        fid = f.get("id", "?")
        score = f"{f['score']:.0f}" if f.get("score") is not None else "-"
        noise = f"{f['noise']:.0f}" if f.get("noise") is not None else "-"
        pid = f"{f['pid']:.0f}" if f.get("pid") is not None else "-"
        dur = f"{f['duration']:.0f}s" if f.get("duration") is not None else "-"
        verdict = f.get("verdict", "").replace("_", " ").lower()
        sc = G if (f.get("score") or 0) >= 85 else Y if (f.get("score") or 0) >= 60 else RED
        print(f"  {sc}{fid:>3}{R}  {sc}{score:>6}{R}  {noise:>6}  {pid:>6}  {dur:>8}  {DIM}{verdict}{R}")

    # Changes since last
    changes = prog.get("changes", [])
    if changes:
        print(f"\n  {B}Latest changes:{R}")
        for ch in changes:
            print(f"    {ch}")


def _print_section_diff(flight_diff):
    """Print flight-to-flight comparison section."""
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}COMPARED TO PREVIOUS FLIGHT:{R}")

    if not flight_diff or not flight_diff.get("has_previous"):
        print(f"  {DIM}  No previous flight to compare against.{R}")
        return

    prev = flight_diff["previous"]
    curr = flight_diff["current"]
    delta = flight_diff["score_delta"]

    # Timestamp of previous
    from datetime import datetime as _dt
    try:
        prev_ts = _dt.fromisoformat(prev["timestamp"])
        now = _dt.now()
        age = now - prev_ts
        if age.days > 0:
            age_str = f"{age.days}d ago"
        elif age.seconds > 3600:
            age_str = f"{age.seconds // 3600}h ago"
        else:
            age_str = f"{age.seconds // 60}m ago"
    except Exception:
        age_str = prev["timestamp"][:16]

    print(f"  {DIM}Flight #{prev['id']} ({age_str}){R}")

    # Score delta
    dc = G if delta > 0 else RED if delta < 0 else Y
    arrow = "↗" if delta > 0 else "↘" if delta < 0 else "→"
    print(f"\n  Score: {prev['score']:.0f} → {curr['score']:.0f}  {dc}{arrow} {delta:+.0f}{R}")

    # Metric changes
    metrics = flight_diff.get("metric_changes", [])
    if metrics:
        print()
        for m in metrics:
            mc = G if m["direction"] == "better" else RED
            arrow = "↗" if m["direction"] == "better" else "↘"
            ax_str = f"{m['axis']} " if m["axis"] else ""
            if isinstance(m["old"], float):
                print(f"    {ax_str}{m['metric']}: {m['old']:.1f}{m['unit']} → {mc}{m['new']:.1f}{m['unit']}  {arrow} {m['direction']}{R}")
            else:
                print(f"    {ax_str}{m['metric']}: {m['old']}{m['unit']} → {mc}{m['new']}{m['unit']}  {arrow} {m['direction']}{R}")

    # Config changes
    config_changes = flight_diff.get("config_changes", [])
    if config_changes:
        print(f"\n  {B}Config changes since last flight:{R}")
        for ch in config_changes:
            try:
                old_f = float(ch["old"])
                new_f = float(ch["new"])
                pct = ((new_f - old_f) / old_f * 100) if old_f != 0 else 0
                pct_str = f"  ({pct:+.0f}%)" if abs(pct) > 1 else ""
            except (ValueError, ZeroDivisionError):
                pct_str = ""
            print(f"    {ch['param']}: {ch['old']} → {ch['new']}{pct_str}")

    # Verdict
    verdict = flight_diff.get("verdict", "")
    if verdict:
        vc = G if delta > 0 else Y if abs(delta) < 3 else RED
        print(f"\n  {vc}{verdict}{R}")


def _build_section_statuses(plan, pid_results, noise_fp, motor_analysis,
                            config_review_text, progression_text,
                            nav_perf=None, nav_results=None, flight_diff=None,
                            prog_data=None):
    """Build one-line status per section for the menu display."""
    R, B, C, G, Y, RED, DIM = _colors()
    statuses = {}

    # PID status
    pid_parts = []
    for pid in pid_results:
        if pid is None:
            continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        ax = pid["axis"][:1]
        if _os is not None and _os > BAD_OVERSHOOT:
            pid_parts.append(f"{RED}{ax}:{_os:.0f}%OS{R}")
        elif _dl is not None and _dl > BAD_DELAY_MS:
            pid_parts.append(f"{Y}{ax}:{_dl:.0f}ms{R}")
        elif _os is not None or _dl is not None:
            pid_parts.append(f"{G}{ax}:ok{R}")
        else:
            pid_parts.append(f"{DIM}{ax}:N/A{R}")
    statuses["pid"] = "  ".join(pid_parts) if pid_parts else f"{DIM}no data{R}"

    # Noise status
    n_sources = len(noise_fp["peaks"]) if noise_fp and noise_fp.get("peaks") else 0
    n_filter_actions = len([a for a in plan["actions"]
                           if a.get("category") == "Filter" and not a.get("deferred")])
    noise_score = plan["scores"].get("noise")
    if noise_score is not None:
        nc = G if noise_score >= 85 else Y if noise_score >= 60 else RED
        statuses["noise"] = f"{nc}{noise_score:.0f}/100{R} — {n_sources} source{'s' if n_sources != 1 else ''}"
        if n_filter_actions:
            statuses["noise"] += f", {Y}{n_filter_actions} filter change{'s' if n_filter_actions != 1 else ''}{R}"
    else:
        statuses["noise"] = f"{DIM}no data{R}"

    # Hover status
    hover_osc = plan["scores"].get("hover_osc", [])
    hover_parts = []
    for osc in hover_osc:
        ax = osc["axis"][:1]
        sev = osc["severity"]
        cause = osc.get("cause", "")
        if sev == "none":
            hover_parts.append(f"{G}{ax}:stable{R}")
        elif cause == "wind_buffeting":
            hover_parts.append(f"{DIM}{ax}:wind{R}")
        elif sev == "severe":
            hover_parts.append(f"{RED}{ax}:severe{R}")
        elif sev == "moderate":
            hover_parts.append(f"{Y}{ax}:moderate{R}")
        else:
            hover_parts.append(f"{Y}{ax}:mild{R}")
    statuses["hover"] = "  ".join(hover_parts) if hover_parts else f"{DIM}no hover data{R}"

    # Config review status
    if config_review_text and config_review_text.strip():
        n_crit = config_review_text.count("[CRITICAL]") + config_review_text.count("✗") + config_review_text.count("✖")
        n_warn = config_review_text.count("[WARNING]") + config_review_text.count("⚠")
        if n_crit:
            statuses["config"] = f"{RED}{n_crit} critical{R}"
            if n_warn:
                statuses["config"] += f", {Y}{n_warn} warnings{R}"
        elif n_warn:
            statuses["config"] = f"{Y}{n_warn} warnings{R}"
        else:
            statuses["config"] = f"{G}all good{R}"
    else:
        statuses["config"] = f"{DIM}no config loaded{R}"

    # History status
    if prog_data and len(prog_data.get("flights", [])) >= 2:
        scores = [f.get("score") for f in prog_data["flights"] if f.get("score") is not None]
        spark = _sparkline(scores) if scores else ""
        n = len(prog_data["flights"])
        statuses["history"] = f"{n} flights {spark}"
    elif progression_text and progression_text.strip():
        statuses["history"] = f"available"
    else:
        statuses["history"] = f"{DIM}no history{R}"

    # CLI status
    n_actions = len([a for a in plan["actions"] if not a.get("deferred")])
    cli_cmds = generate_cli_commands([a for a in plan["actions"] if not a.get("deferred")])
    if cli_cmds:
        statuses["cli"] = f"{Y}{len(cli_cmds) - 1} changes{R}"  # -1 for 'save'
    else:
        statuses["cli"] = f"{G}no changes needed{R}"

    # Nav performance status
    if nav_perf and nav_perf.get("score") is not None:
        ns = nav_perf["score"]
        nc = G if ns >= 85 else Y if ns >= 60 else RED
        parts = []
        decel = nav_perf.get("deceleration", [])
        if decel:
            avg_peak = np.mean([e["peak_error_cm"] for e in decel])
            parts.append(f"{len(decel)} decel events ({avg_peak:.0f}cm)")
        ph = nav_perf.get("poshold")
        if ph:
            parts.append(f"CEP {ph['cep_cm']:.0f}cm")
        ah = nav_perf.get("althold")
        if ah:
            parts.append(f"alt RMS {ah['rms_error_cm']:.0f}cm")
        statuses["nav_perf"] = f"{nc}{ns}/100{R} — {', '.join(parts)}" if parts else f"{nc}{ns}/100{R}"
    else:
        statuses["nav_perf"] = f"{DIM}no nav data{R}"

    # Nav sensors status
    if nav_results:
        sensor_parts = []
        for name in ["compass", "gps", "baro"]:
            sec = nav_results.get(name)
            if sec and sec.get("score") is not None:
                s = sec["score"]
                c = G if s >= 80 else Y if s >= 50 else RED
                sensor_parts.append(f"{c}{name[:3].capitalize()}:{s}{R}")
        statuses["nav_sensors"] = "  ".join(sensor_parts) if sensor_parts else f"{DIM}no sensor data{R}"
    else:
        statuses["nav_sensors"] = f"{DIM}no sensor data{R}"

    # Flight diff status
    if flight_diff and flight_diff.get("has_previous"):
        delta = flight_diff["score_delta"]
        dc = G if delta > 0 else RED if delta < 0 else Y
        arrow = "↗" if delta > 0 else "↘" if delta < 0 else "→"
        n_changes = len(flight_diff.get("config_changes", []))
        statuses["diff"] = f"{dc}{arrow} {delta:+.0f}{R} ({n_changes} config change{'s' if n_changes != 1 else ''})"
    else:
        statuses["diff"] = f"{DIM}no previous flight{R}"

    return statuses


def _interactive_menu(plan, pid_results, noise_fp, motor_analysis, config, data,
                      profile, config_review_text, progression_text,
                      nav_perf=None, nav_results=None, flight_diff=None,
                      prog_data=None, accel_vib=None):
    """Run interactive menu loop for exploring analysis results."""
    R, B, C, G, Y, RED, DIM = _colors()

    statuses = _build_section_statuses(plan, pid_results, noise_fp, motor_analysis,
                                        config_review_text, progression_text,
                                        nav_perf=nav_perf, nav_results=nav_results,
                                        flight_diff=flight_diff, prog_data=prog_data)

    sections = {
        "1": ("PID Tuning", statuses["pid"]),
        "2": ("Noise Analysis", statuses["noise"]),
        "3": ("Hover & Stability", statuses["hover"]),
        "4": ("Nav Performance", statuses["nav_perf"]),
        "5": ("Nav Sensors", statuses["nav_sensors"]),
        "6": ("Config Review", statuses["config"]),
        "7": ("Flight History", statuses["history"]),
        "8": ("vs Previous", statuses["diff"]),
        "c": ("CLI Commands", statuses["cli"]),
        "a": ("Show All", ""),
        "q": ("Quit", ""),
    }

    while True:
        # Clear screen and show score bar
        print("\033[2J\033[H", end="")
        _print_score_bar(plan, config, data)

        # Menu
        print(f"\n  {B}{'─'*66}{R}")
        for key, (name, status) in sections.items():
            if key in ("a", "q"):
                print(f"  {B}[{key.upper()}]{R} {name}")
            else:
                print(f"  {B}[{key}]{R} {name:20s} {status}")
        print(f"  {B}{'─'*66}{R}")

        try:
            choice = input(f"\n  {B}Section (1-8, C, A, Q): {R}").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if choice == "q" or choice == "":
            break
        elif choice == "1":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_pid(plan, pid_results, config, profile)
            _wait_for_enter()
        elif choice == "2":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_noise(plan, noise_fp)
            if accel_vib and accel_vib.get("axes"):
                _print_section_vibration(accel_vib)
            _wait_for_enter()
        elif choice == "3":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_hover(plan, motor_analysis)
            _wait_for_enter()
        elif choice == "4":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_nav(nav_perf)
            _wait_for_enter()
        elif choice == "5":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_nav_sensors(nav_results)
            _wait_for_enter()
        elif choice == "6":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            if config_review_text and config_review_text.strip():
                print(config_review_text)
            else:
                print(f"\n  {DIM}No config review available (no dump/diff loaded).{R}")
            _wait_for_enter()
        elif choice == "7":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            if prog_data and prog_data.get("flights"):
                _print_section_history(prog_data, flight_diff)
            elif progression_text and progression_text.strip():
                print(progression_text)
            else:
                print(f"\n  {DIM}No flight history available.{R}")
            _wait_for_enter()
        elif choice == "8":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_diff(flight_diff)
            _wait_for_enter()
        elif choice == "c":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_cli(plan)
            # Also show nav CLI commands if any
            if nav_perf and nav_perf.get("nav_actions"):
                print(f"\n  {B}Nav PID commands:{R}")
                for a in nav_perf["nav_actions"]:
                    print(f"    {G}{a['cli']}{R}")
            _wait_for_enter()
        elif choice == "a":
            print("\033[2J\033[H", end="")
            _print_score_bar(plan, config, data)
            _print_section_pid(plan, pid_results, config, profile)
            _print_section_noise(plan, noise_fp)
            if accel_vib and accel_vib.get("axes"):
                _print_section_vibration(accel_vib)
            _print_section_hover(plan, motor_analysis)
            _print_section_nav(nav_perf)
            _print_section_nav_sensors(nav_results)
            if config_review_text and config_review_text.strip():
                print(config_review_text)
            _print_section_cli(plan)
            if nav_perf and nav_perf.get("nav_actions"):
                print(f"\n  {B}Nav PID commands:{R}")
                for a in nav_perf["nav_actions"]:
                    print(f"    {G}{a['cli']}{R}")
            _print_section_diff(flight_diff)
            if prog_data and prog_data.get("flights"):
                _print_section_history(prog_data, flight_diff)
            elif progression_text and progression_text.strip():
                print(progression_text)
            _wait_for_enter()


def _wait_for_enter():
    """Wait for Enter key to return to menu."""
    R, B, C, G, Y, RED, DIM = _colors()
    try:
        input(f"\n  {DIM}Press Enter to return to menu...{R}")
    except (EOFError, KeyboardInterrupt):
        pass


def _offer_auto_apply(cli_cmds, args):
    """Offer to push CLI commands directly to the FC.

    Reconnects to the FC, verifies a vault backup exists, sends commands
    via cli_batch, and handles the post-save USB reconnection.
    """
    R, B, C, G, Y, RED, DIM = _colors()

    # Filter out 'save' — cli_batch adds it automatically
    cmds_to_send = [c for c in cli_cmds if c.strip().lower() != "save"]
    if not cmds_to_send:
        return

    # Check vault has a backup
    vault_entries = vault_list(blackbox_dir=getattr(args, 'blackbox_dir', './blackbox'), limit=1)
    if not vault_entries:
        print(f"\n  {Y}⚠ No config backup in vault — run a download first to create a backup.{R}")
        print(f"  {DIM}  Auto-apply requires a vault backup for safety.{R}")
        return

    print(f"\n  {B}{'─'*66}{R}")
    print(f"  {B}APPLY CHANGES TO FC{R}")
    print(f"  {DIM}Vault backup: {vault_entries[0]['filename']} ({vault_entries[0]['age_str']}){R}")
    print()
    for cmd in cmds_to_send:
        print(f"    {G}{cmd}{R}")
    print(f"    {DIM}save{R}")
    print()

    try:
        answer = input(f"  {B}Apply these {len(cmds_to_send)} changes to the FC? (y/N): {R}").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return

    if answer != "y":
        print(f"  {DIM}Skipped.{R}")
        return

    # Reconnect to FC
    try:
        try:
            from inav_toolkit.msp import INAVDevice, auto_detect_fc
        except ImportError:
            from inav_msp import INAVDevice, auto_detect_fc

        port = None
        if args.device == "auto":
            port = auto_detect_fc()
            if not port:
                print(f"  {RED}✗ No FC found. Is it still connected?{R}")
                return
        else:
            port = args.device

        print(f"  Connecting to {port}...", end="", flush=True)
        fc = INAVDevice(port)
        info = fc.identify()
        if not info:
            print(f" {RED}failed{R}")
            fc.close()
            return
        print(f" {info.get('craft_name', '')} - {info.get('firmware', '')}")

        # Send commands
        print(f"  Sending {len(cmds_to_send)} commands...", end="", flush=True)
        results = fc.cli_batch(cmds_to_send, timeout=5.0, save=True)

        # Check results for errors
        errors = []
        for cmd, response in results:
            if "invalid" in response.lower() or "error" in response.lower():
                errors.append((cmd, response.strip()))

        if errors:
            print(f" {Y}done with {len(errors)} warning(s){R}")
            for cmd, resp in errors:
                print(f"    {Y}⚠ {cmd}: {resp}{R}")
        else:
            print(f" {G}done{R}")

        print(f"  {DIM}FC will reboot after save...{R}")

        # Wait for USB reset after save
        import time as _time3
        _time3.sleep(3.0)

        # Reconnect to verify
        try:
            fc.close()
        except Exception:
            pass
        _time3.sleep(2.0)

        try:
            fc2 = INAVDevice(port)
            info2 = fc2.identify()
            if info2:
                print(f"  {G}✓ FC reconnected: {info2.get('craft_name', '')} - changes applied{R}")
            fc2.close()
        except Exception:
            print(f"  {DIM}FC rebooting — changes saved, reconnect manually to verify{R}")

    except Exception as e:
        print(f"\n  {RED}✗ Auto-apply failed: {e}{R}")
        print(f"  {DIM}  Paste the CLI commands manually into INAV Configurator.{R}")


# ─── HTML Report ──────────────────────────────────────────────────────────────

def _create_nav_charts(nav_results, data, sr):
    """Generate matplotlib charts for nav HTML report.

    Returns dict of {name: base64_png_string}.
    """
    setup_dark_style()
    charts = {}

    # ─── 1. Heading timeline + jitter ───
    if "att_heading" in data:
        hdg = data["att_heading"] / 10.0  # decideg -> deg
        t = data["time_s"]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 5), sharex=True,
                                        gridspec_kw={"height_ratios": [2, 1]})
        fig.suptitle("Compass - Heading", fontsize=13, color="#c0caf5", fontweight="bold", y=0.98)

        ax1.plot(t, hdg, color="#7aa2f7", linewidth=0.6, alpha=0.9)
        ax1.set_ylabel("Heading (deg)")
        ax1.set_ylim(np.nanmin(hdg) - 10, np.nanmax(hdg) + 10)

        # Heading rate (downsampled to compass update rate)
        ds = max(1, int(sr / 50))
        hdg_ds = hdg[::ds]
        t_ds = t[::ds]
        hdg_unwrap = np.unwrap(np.deg2rad(hdg_ds))
        hdg_rate = np.abs(np.diff(hdg_unwrap) * (sr / ds)) * 180 / np.pi
        ax2.plot(t_ds[1:], hdg_rate, color="#f7768e", linewidth=0.5, alpha=0.8)
        jitter = nav_results.get("compass", {}).get("heading_jitter_deg")
        if jitter is not None:
            ax2.axhline(jitter, color="#e0af68", linestyle="--", linewidth=1, alpha=0.7,
                        label=f"RMS: {jitter:.1f} deg/s")
            ax2.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")
        ax2.set_ylabel("Rate (deg/s)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylim(0, min(max(hdg_rate) * 1.2 if len(hdg_rate) > 0 else 20, 60))

        fig.tight_layout()
        charts["heading"] = fig_to_base64(fig)

    # ─── 2. Baro altitude + nav altitude ───
    if "baro_alt" in data:
        baro = data["baro_alt"] / 100.0  # cm -> m
        t = data["time_s"]

        fig, ax = plt.subplots(1, 1, figsize=(14, 3.5))
        fig.suptitle("Barometer & Estimator Altitude", fontsize=13, color="#c0caf5",
                     fontweight="bold", y=1.0)

        ax.plot(t, baro, color="#e0af68", linewidth=0.8, alpha=0.9, label="Baro")

        if "nav_pos_u" in data:
            nav_u = data["nav_pos_u"] / 100.0
            ax.plot(t, nav_u, color="#9ece6a", linewidth=0.8, alpha=0.8, label="Nav Estimate")

        ax.set_ylabel("Altitude (m)")
        ax.set_xlabel("Time (s)")
        ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")

        noise = nav_results.get("baro", {}).get("noise_cm")
        if noise is not None:
            ax.text(0.01, 0.95, f"Baro noise: {noise:.0f}cm RMS", transform=ax.transAxes,
                    fontsize=9, color="#e0af68", va="top",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1b26", edgecolor="#565f89", alpha=0.8))

        fig.tight_layout()
        charts["altitude"] = fig_to_base64(fig)

    # ─── 3. GPS satellite count ───
    gps_frames = data.get("_gps_frames", [])
    if gps_frames and len(gps_frames) > 5:
        gps_t = []
        gps_sats = []
        for idx, fields in gps_frames:
            if "GPS_numSat" in fields:
                gps_t.append(idx / sr if sr > 0 else idx)
                gps_sats.append(fields["GPS_numSat"])

        if gps_sats:
            fig, ax = plt.subplots(1, 1, figsize=(14, 2.5))
            fig.suptitle("GPS Satellite Count", fontsize=13, color="#c0caf5",
                         fontweight="bold", y=1.02)

            ax.fill_between(gps_t, gps_sats, alpha=0.2, color="#9ece6a")
            ax.plot(gps_t, gps_sats, color="#9ece6a", linewidth=1.2)
            ax.axhline(6, color="#f7768e", linestyle="--", linewidth=0.8, alpha=0.5, label="Min safe (6)")
            ax.set_ylabel("Satellites")
            ax.set_xlabel("Time (s)")
            ax.set_ylim(0, max(gps_sats) + 3)
            ax.legend(loc="lower right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")

            avg_sats = nav_results.get("gps", {}).get("avg_sats")
            if avg_sats is not None:
                ax.text(0.01, 0.9, f"Avg: {avg_sats:.0f} sats", transform=ax.transAxes,
                        fontsize=9, color="#9ece6a", va="top",
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="#1a1b26", edgecolor="#565f89", alpha=0.8))

            fig.tight_layout()
            charts["gps"] = fig_to_base64(fig)

    # ─── 4. EPH timeline ───
    if "nav_eph" in data:
        eph = data["nav_eph"]
        valid = ~np.isnan(eph) & (eph > 0)
        if np.sum(valid) > 50:
            t = data["time_s"]
            fig, ax = plt.subplots(1, 1, figsize=(14, 2.5))
            fig.suptitle("GPS Accuracy (EPH)", fontsize=13, color="#c0caf5",
                         fontweight="bold", y=1.02)

            ax.plot(t[valid], eph[valid], color="#bb9af7", linewidth=0.8, alpha=0.9)
            ax.fill_between(t[valid], eph[valid], alpha=0.15, color="#bb9af7")
            ax.axhline(300, color="#e0af68", linestyle="--", linewidth=0.8, alpha=0.5, label="Marginal (300cm)")
            ax.axhline(500, color="#f7768e", linestyle="--", linewidth=0.8, alpha=0.5, label="Poor (500cm)")
            ax.set_ylabel("EPH (cm)")
            ax.set_xlabel("Time (s)")
            ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")

            fig.tight_layout()
            charts["eph"] = fig_to_base64(fig)

    # ─── 5. Position scatter (poshold only) ───
    poshold = nav_results.get("poshold")
    if poshold and poshold.get("score") is not None:
        if "nav_pos_n" in data and "nav_pos_e" in data:
            fig, ax = plt.subplots(1, 1, figsize=(5.5, 5.5))
            fig.suptitle("Position Hold Scatter", fontsize=13, color="#c0caf5",
                         fontweight="bold", y=0.98)

            n = data["nav_pos_n"] / 100.0  # cm -> m
            e = data["nav_pos_e"] / 100.0
            tn = data.get("nav_tgt_n", np.zeros_like(n)) / 100.0
            te = data.get("nav_tgt_e", np.zeros_like(e)) / 100.0

            dn = n - tn
            de = e - te

            # Color by time
            colors = np.linspace(0, 1, len(dn))
            ax.scatter(de, dn, c=colors, cmap="cool", s=1, alpha=0.5)
            ax.plot(0, 0, "x", color="#f7768e", markersize=12, markeredgewidth=2, label="Target")

            cep = poshold.get("cep_cm")
            if cep is not None:
                circle = plt.Circle((0, 0), cep / 100.0, fill=False, color="#e0af68",
                                    linestyle="--", linewidth=1, label=f"CEP: {cep:.0f}cm")
                ax.add_patch(circle)

            ax.set_xlabel("East error (m)")
            ax.set_ylabel("North error (m)")
            ax.set_aspect("equal")
            lim = max(abs(dn).max(), abs(de).max(), 2) * 1.2
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")

            fig.tight_layout()
            charts["position"] = fig_to_base64(fig)

    return charts


def _generate_nav_only_html(nav_results, config, data):
    """Generate a standalone HTML report for nav-only analysis."""
    craft = config.get("craft_name", "Unknown")
    fw = config.get("firmware_revision", "")
    duration = data["time_s"][-1] if "time_s" in data and len(data["time_s"]) > 0 else 0
    sr = data.get("sample_rate", 0)

    # Generate charts
    nav_charts = _create_nav_charts(nav_results, data, sr)

    # Score color helper
    def sc(v):
        if v is None:
            return "dim", "-"
        if v >= 80:
            return "good", f"{v}/100"
        if v >= 60:
            return "warn", f"{v}/100"
        return "bad", f"{v}/100"

    nav_score = nav_results.get("nav_score")
    ns_class, ns_text = sc(nav_score)

    # Score gradient
    if nav_score is not None:
        sg = ("linear-gradient(90deg, #9ece6a, #73daca)" if nav_score >= 80
              else "linear-gradient(90deg, #e0af68, #ff9e64)" if nav_score >= 60
              else "linear-gradient(90deg, #f7768e, #ff6b6b)")
    else:
        sg = "linear-gradient(90deg, #565f89, #565f89)"

    # Build score rows
    rows = ""
    for key, label, icon in [("compass", "Compass", "&#x1F9ED;"),
                              ("gps", "GPS", "&#x1F6F0;"),
                              ("baro", "Barometer", "&#x1F321;"),
                              ("estimator", "Estimator", "&#x1F4CA;")]:
        r = nav_results.get(key)
        if r and r.get("score") is not None:
            s = r["score"]
            css, txt = sc(s)
            bar_w = s
            bar_color = "#9ece6a" if s >= 80 else "#e0af68" if s >= 60 else "#f7768e"
            details = []
            if key == "compass":
                if r.get("heading_jitter_deg") is not None:
                    details.append(f"Jitter: {r['heading_jitter_deg']:.1f} deg/s")
                if r.get("throttle_correlation") is not None:
                    details.append(f"Motor corr: {r['throttle_correlation']:.2f}")
                if r.get("heading_drift_dps") is not None:
                    details.append(f"Drift: {r['heading_drift_dps']:.2f} deg/s")
            elif key == "gps":
                if r.get("avg_sats") is not None:
                    details.append(f"Sats: {r['avg_sats']:.0f} avg")
                if r.get("min_sats") is not None:
                    details.append(f"min {r['min_sats']}")
                if r.get("avg_eph") is not None:
                    details.append(f"EPH: {r['avg_eph']:.0f}cm")
                if r.get("position_jumps", 0) > 0:
                    details.append(f"{r['position_jumps']} jumps")
            elif key == "baro":
                if r.get("noise_cm") is not None:
                    details.append(f"Noise: {r['noise_cm']:.0f}cm RMS")
                if r.get("spikes", 0) > 0:
                    details.append(f"{r['spikes']} spikes")
                if r.get("throttle_correlation") is not None:
                    details.append(f"Throttle corr: {r['throttle_correlation']:.2f}")
            elif key == "estimator":
                if r.get("baro_vs_nav_corr") is not None:
                    details.append(f"Baro corr: {r['baro_vs_nav_corr']:.3f}")
                if r.get("max_divergence_cm") is not None:
                    details.append(f"Max div: {r['max_divergence_cm']:.0f}cm")
            detail_str = " | ".join(details)
            rows += f'''<div class="sc-row">
  <div class="sc-label">{icon} {label}</div>
  <div class="sc-score {css}">{txt}</div>
  <div class="sc-bar"><div class="sc-fill" style="width:{bar_w}%;background:{bar_color}"></div></div>
  <div class="sc-detail">{detail_str}</div>
</div>\n'''

    # Althold/poshold rows
    for key, label, icon in [("althold", "Alt Hold", "&#x2195;"), ("poshold", "Pos Hold", "&#x1F4CD;")]:
        r = nav_results.get(key)
        if r and r.get("score") is not None:
            s = r["score"]
            css, txt = sc(s)
            bar_w = s
            bar_color = "#9ece6a" if s >= 80 else "#e0af68" if s >= 60 else "#f7768e"
            details = []
            if key == "althold":
                if r.get("oscillation_cm") is not None:
                    details.append(f"Osc: {r['oscillation_cm']:.0f}cm")
            elif key == "poshold":
                if r.get("cep_cm") is not None:
                    details.append(f"CEP: {r['cep_cm']:.0f}cm")
                if r.get("max_drift_cm") is not None:
                    details.append(f"Max drift: {r['max_drift_cm']:.0f}cm")
                if r.get("toilet_bowl"):
                    details.append("TOILET BOWL")
            detail_str = " | ".join(details)
            rows += f'''<div class="sc-row">
  <div class="sc-label">{icon} {label}</div>
  <div class="sc-score {css}">{txt}</div>
  <div class="sc-bar"><div class="sc-fill" style="width:{bar_w}%;background:{bar_color}"></div></div>
  <div class="sc-detail">{detail_str}</div>
</div>\n'''

    # Build findings
    findings_html = ""
    all_findings = []
    for key in ("compass", "gps", "baro", "estimator", "althold", "poshold"):
        r = nav_results.get(key)
        if r and r.get("findings"):
            all_findings.extend(r["findings"])
    if all_findings:
        findings_html = '<section><h2>Findings</h2>\n'
        for severity, msg in all_findings:
            css = "warning" if severity == "WARNING" else "info"
            icon = "!" if severity == "WARNING" else "*"
            findings_html += f'<div class="fd {css}"><span class="fd-icon">{icon}</span> {msg}</div>\n'
        findings_html += '</section>'

    # Build charts HTML
    charts_html = ""
    chart_order = [("heading", "Compass"), ("altitude", "Altitude"),
                   ("gps", "GPS"), ("eph", "EPH"), ("position", "Position")]
    chart_sections = []
    for key, label in chart_order:
        if key in nav_charts:
            chart_sections.append(
                f'<div class="chart-card"><img src="data:image/png;base64,{nav_charts[key]}"></div>')

    if chart_sections:
        charts_html = '<section><h2>Charts</h2>\n' + "\n".join(chart_sections) + '\n</section>'

    # Config info
    config_html = ""
    config_items = []
    for k, label in [("mag_hardware", "Compass"), ("gps_provider", "GPS"),
                      ("nav_alt_p", "Alt P"), ("nav_pos_p", "Pos P"),
                      ("nav_heading_p", "Hdg P"), ("nav_hover_thr", "Hover Thr")]:
        v = config.get(k)
        if v is not None:
            config_items.append(f'<span class="cfg-item">{label}: {v}</span>')
    if config.get("gps_use_galileo") is not None:
        gnss = []
        if config.get("gps_use_galileo"): gnss.append("GAL")
        if config.get("gps_use_beidou"): gnss.append("BDS")
        if config.get("gps_use_glonass"): gnss.append("GLO")
        config_items.append(f'<span class="cfg-item">GNSS: GPS{" + " + "+".join(gnss) if gnss else " only"}</span>')
    if config_items:
        config_html = '<section><h2>FC Config</h2><div class="cfg-grid">' + "".join(config_items) + '</div></section>'

    # Tune warning
    tune_warn = ""
    if nav_results.get("_tune_warning"):
        tune_warn = '''<div class="tune-warn">
  <div class="tw-icon">!</div>
  <div class="tw-text"><strong>PID tuning incomplete</strong> - oscillation detected.
  Nav readings (especially baro and compass) are affected by vibration. Fix PIDs first, then re-check nav health.</div>
</div>'''

    return f"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nav Health - {craft}</title><style>
:root{{--bg:#0f1117;--cd:#1a1b26;--ca:#1e2030;--bd:#2a2d3e;--tx:#c0caf5;--dm:#7982a9;--bl:#7aa2f7;--gn:#9ece6a;--rd:#f7768e;--yl:#e0af68;--tl:#4ecdc4;--pp:#bb9af7;--or:#ff9e64}}
*{{box-sizing:border-box;margin:0;padding:0}}body{{font-family:'SF Mono','Cascadia Code','JetBrains Mono',monospace;background:var(--bg);color:var(--tx);line-height:1.6}}
.ct{{max-width:1000px;margin:0 auto;padding:24px}}
header{{background:linear-gradient(135deg,#1a1b26,#24283b);border-bottom:2px solid var(--bl);padding:24px 0;text-align:center}}
header h1{{font-size:1.5rem;letter-spacing:3px;text-transform:uppercase;color:var(--bl)}}
.mt{{color:var(--dm);font-size:.8rem;margin-top:8px}}
.hero{{text-align:center;padding:28px 0}}
.hero-score{{font-size:3rem;font-weight:700;background:{sg};-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}}
.hero-label{{font-size:.75rem;color:var(--dm);text-transform:uppercase;letter-spacing:2px;margin-top:4px}}
.hero-bar{{width:300px;height:8px;background:var(--cd);border-radius:4px;margin:12px auto;overflow:hidden;border:1px solid var(--bd)}}
.hero-fill{{height:100%;background:{sg};border-radius:4px;width:{nav_score or 0}%}}
section{{margin:28px 0}}
h2{{font-size:.85rem;color:var(--bl);letter-spacing:2px;text-transform:uppercase;border-bottom:1px solid var(--bd);padding-bottom:8px;margin-bottom:16px}}
.sc-row{{display:flex;align-items:center;gap:12px;padding:10px 16px;background:var(--cd);border:1px solid var(--bd);border-radius:6px;margin:6px 0}}
.sc-label{{min-width:130px;font-weight:600;font-size:.85rem}}
.sc-score{{min-width:60px;font-weight:700;font-size:.85rem;text-align:right}}
.sc-bar{{flex:0 0 120px;height:6px;background:var(--bg);border-radius:3px;overflow:hidden}}
.sc-fill{{height:100%;border-radius:3px}}
.sc-detail{{flex:1;color:var(--dm);font-size:.8rem}}
.good{{color:var(--gn)}}.warn{{color:var(--yl)}}.bad{{color:var(--rd)}}.dim{{color:var(--dm)}}
.fd{{padding:10px 16px;margin:6px 0;border-radius:6px;font-size:.83rem;line-height:1.5}}
.fd.warning{{background:rgba(224,175,104,.06);border-left:3px solid var(--yl)}}
.fd.info{{background:rgba(122,162,247,.06);border-left:3px solid var(--dm)}}
.fd-icon{{font-weight:700;margin-right:4px}}
.chart-card{{background:var(--cd);border:1px solid var(--bd);border-radius:8px;padding:12px;margin:12px 0;overflow:hidden}}
.chart-card img{{width:100%;height:auto;display:block;border-radius:4px}}
.cfg-grid{{display:flex;flex-wrap:wrap;gap:8px}}
.cfg-item{{background:var(--cd);border:1px solid var(--bd);border-radius:4px;padding:4px 12px;font-size:.8rem;color:var(--dm)}}
.tune-warn{{display:flex;gap:16px;background:rgba(247,118,142,.06);border:1px solid rgba(247,118,142,.3);border-radius:8px;padding:16px 20px;margin:16px 0;align-items:flex-start}}
.tw-icon{{font-size:1.4rem;font-weight:700;color:var(--rd);min-width:28px;text-align:center}}
.tw-text{{font-size:.85rem;color:var(--tx)}}
footer{{text-align:center;color:var(--dm);font-size:.7rem;padding:24px 0;border-top:1px solid var(--bd);margin-top:40px}}
</style></head><body>
<div class="ct">
<header><h1>Nav Health Report</h1>
<div class="mt">{craft} | {fw} | {duration:.1f}s | {sr:.0f}Hz | {datetime.now().strftime('%Y-%m-%d %H:%M')}</div></header>
<div class="hero"><div class="hero-label">Nav Score</div><div class="hero-score">{ns_text}</div>
<div class="hero-bar"><div class="hero-fill"></div></div></div>
{tune_warn}
<section><h2>Sensor Scores</h2>
{rows}</section>
{findings_html}
{charts_html}
{config_html}
<footer>INAV Nav Analyzer v{REPORT_VERSION}</footer>
</div></body></html>"""


def generate_trend_report(progression, craft_name, output_path):
    """Generate a standalone HTML trend report from flight progression data.

    Args:
        progression: dict from FlightDB.get_progression() with 'flights', 'trend', 'changes'
        craft_name: Name of the craft
        output_path: Where to save the HTML file
    """
    flights = progression.get("flights", [])
    if not flights:
        return

    trend = progression.get("trend", "insufficient")
    changes = progression.get("changes", [])

    # Build chart data as JSON arrays
    labels = []
    scores = []
    noise_scores = []
    pid_scores = []
    motor_scores = []
    osc_scores = []

    for i, f in enumerate(flights):
        labels.append(f"Flight {i+1}")
        scores.append(f.get("score") or 0)
        noise_scores.append(f.get("noise") or 0)
        pid_scores.append(f.get("pid") or 0)
        motor_scores.append(f.get("motor") or 0)
        osc_scores.append(f.get("osc") or 0)

    # Trend indicator
    trend_icon = {"improving": "↑", "stable": "→", "degrading": "↓"}.get(trend, "?")
    trend_color = {"improving": "#9ece6a", "stable": "#e0af68", "degrading": "#f7768e"}.get(trend, "#7aa2f7")

    changes_html = ""
    if changes:
        changes_html = "<ul>" + "".join(f"<li>{c}</li>" for c in changes) + "</ul>"

    # Flight history table
    table_rows = ""
    for i, f in enumerate(flights):
        sc = f.get("score") or 0
        sc_color = "#9ece6a" if sc >= 85 else "#e0af68" if sc >= 60 else "#f7768e"
        dur = f.get("duration") or 0
        table_rows += f"""<tr>
            <td>Flight {i+1}</td>
            <td style="color:{sc_color};font-weight:bold">{sc:.0f}</td>
            <td>{f.get('noise') or 0:.0f}</td>
            <td>{f.get('pid') or 0:.0f}</td>
            <td>{f.get('motor') or 0:.0f}</td>
            <td>{f.get('osc') or 0:.0f}</td>
            <td>{dur:.0f}s</td>
            <td>{f.get('verdict', '')}</td>
        </tr>"""

    import json as _json
    html = f"""<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<title>Tuning Progression — {craft_name}</title>
<style>
  body {{ background: #1a1b26; color: #c0caf5; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; margin: 0; padding: 20px; }}
  .container {{ max-width: 900px; margin: 0 auto; }}
  h1 {{ color: #7aa2f7; margin-bottom: 5px; }}
  h2 {{ color: #bb9af7; border-bottom: 1px solid #3b4261; padding-bottom: 8px; }}
  .trend {{ display: inline-block; padding: 4px 12px; border-radius: 6px;
            background: {trend_color}22; color: {trend_color}; font-weight: bold; font-size: 1.1em; }}
  .changes {{ background: #24283b; padding: 12px 16px; border-radius: 8px; margin: 12px 0; }}
  .changes li {{ margin: 4px 0; }}
  canvas {{ background: #24283b; border-radius: 8px; padding: 10px; margin: 16px 0; }}
  table {{ width: 100%; border-collapse: collapse; margin: 16px 0; }}
  th, td {{ padding: 8px 12px; text-align: center; border-bottom: 1px solid #3b4261; }}
  th {{ color: #7aa2f7; font-size: 0.85em; text-transform: uppercase; }}
  footer {{ color: #565f89; font-size: 0.8em; margin-top: 30px; text-align: center; }}
</style>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
</head><body>
<div class="container">
<h1>Tuning Progression — {craft_name}</h1>
<p class="trend">{trend_icon} {trend.title()}</p>
<span style="color:#565f89; margin-left:12px">{len(flights)} flights analyzed</span>

{"<div class='changes'><strong>Latest changes:</strong>" + changes_html + "</div>" if changes_html else ""}

<h2>Score Progression</h2>
<canvas id="scoreChart" height="250"></canvas>

<h2>Component Scores</h2>
<canvas id="componentChart" height="250"></canvas>

<h2>Flight History</h2>
<table>
<tr><th>Flight</th><th>Score</th><th>Noise</th><th>PID</th><th>Motor</th><th>Osc</th><th>Duration</th><th>Verdict</th></tr>
{table_rows}
</table>

<script>
const labels = {_json.dumps(labels)};
const scores = {_json.dumps(scores)};
const noise = {_json.dumps(noise_scores)};
const pid = {_json.dumps(pid_scores)};
const motor = {_json.dumps(motor_scores)};
const osc = {_json.dumps(osc_scores)};

const gridColor = '#3b4261';
const axisOpts = {{ ticks: {{ color: '#565f89' }}, grid: {{ color: gridColor }} }};

new Chart(document.getElementById('scoreChart'), {{
  type: 'line',
  data: {{
    labels: labels,
    datasets: [{{
      label: 'Overall Score',
      data: scores,
      borderColor: '#7aa2f7',
      backgroundColor: '#7aa2f722',
      fill: true,
      tension: 0.3,
      pointRadius: 5,
      pointBackgroundColor: scores.map(s => s >= 85 ? '#9ece6a' : s >= 60 ? '#e0af68' : '#f7768e'),
    }}]
  }},
  options: {{
    scales: {{ y: {{ ...axisOpts, min: 0, max: 100 }}, x: axisOpts }},
    plugins: {{ legend: {{ labels: {{ color: '#c0caf5' }} }} }}
  }}
}});

new Chart(document.getElementById('componentChart'), {{
  type: 'line',
  data: {{
    labels: labels,
    datasets: [
      {{ label: 'Noise',  data: noise,  borderColor: '#bb9af7', tension: 0.3 }},
      {{ label: 'PID',    data: pid,    borderColor: '#7dcfff', tension: 0.3 }},
      {{ label: 'Motor',  data: motor,  borderColor: '#e0af68', tension: 0.3 }},
      {{ label: 'Oscillation', data: osc, borderColor: '#f7768e', tension: 0.3 }},
    ]
  }},
  options: {{
    scales: {{ y: {{ ...axisOpts, min: 0, max: 100 }}, x: axisOpts }},
    plugins: {{ legend: {{ labels: {{ color: '#c0caf5' }} }} }}
  }}
}});
</script>

<footer>INAV Toolkit v{REPORT_VERSION} — Trend Report</footer>
</div></body></html>"""

    with open(output_path, "w") as f:
        f.write(html)


def _generate_vibration_html(accel_vib):
    """Generate HTML section for accelerometer vibration analysis."""
    if not accel_vib or not accel_vib.get("axes"):
        return ""

    score = accel_vib.get("score", 100)
    sc = "good" if score >= 85 else "warn" if score >= 60 else "bad"
    overall = accel_vib.get("overall_rms_g", 0)

    html = f'<section><h2>Structural Vibration</h2>'
    html += f'<div class="cc"><span class="{sc}" style="font-size:1rem">'
    html += f'Vibration Score: {score}/100 — {overall:.3f}g RMS overall</span></div>'

    # Per-axis table
    html += '<div class="cc"><table><tr><th>Axis</th><th>RMS (g)</th><th>Peaks</th><th>Status</th></tr>'
    for ax in accel_vib["axes"]:
        rms = ax["rms_g"]
        rc = "good" if rms < 0.1 else "warn" if rms < 0.3 else "bad"
        n_peaks = len(ax["peaks"])
        peak_freqs = ", ".join(f'{p["freq_hz"]:.0f}Hz' for p in ax["peaks"][:3])
        status = "Clean" if rms < 0.1 else "Moderate" if rms < 0.3 else "High"
        html += f'<tr><td style="font-weight:700">{ax["axis"]}</td>'
        html += f'<td class="{rc}">{rms:.3f}</td>'
        html += f'<td style="color:var(--dm)">{peak_freqs or "—"}</td>'
        html += f'<td class="{rc}">{status}</td></tr>'
    html += '</table></div>'

    # Findings
    findings = accel_vib.get("findings", [])
    if findings:
        html += '<div class="cc">'
        for f in findings:
            fc = "warning" if f["level"] == "WARNING" else "info"
            html += f'<div class="nav-findings"><div class="finding {fc}">'
            html += f'<strong>{f["text"]}</strong>'
            if f.get("detail"):
                html += f'<div style="color:var(--dm);font-size:.8rem;margin-top:4px">{f["detail"]}</div>'
            html += '</div></div>'
        html += '</div>'

    # Per-axis peak details
    for ax in accel_vib["axes"]:
        if ax["findings"]:
            html += f'<div class="cc"><h3>{ax["axis"]}-Axis Details</h3>'
            for f in ax["findings"]:
                fc = "warning" if f["level"] == "WARNING" else "info"
                html += f'<div class="nav-findings"><div class="finding {fc}">{f["text"]}</div></div>'
            html += '</div>'

    html += '</section>'
    return html


def _generate_map_html(data, nav_perf=None):
    """Generate HTML section with Leaflet map showing GPS flight track.

    Colors track by flight mode, marks deceleration events and home position.
    Returns empty string if no GPS data available.
    """
    gps_frames = data.get("_gps_frames", [])
    if not gps_frames or len(gps_frames) < 10:
        return ""

    sr = data.get("sample_rate", 1000)

    # Flexible field lookup for GPS frames (field names vary by firmware/decoder)
    def _gps_field(fields, *candidates):
        """Find a GPS field by trying multiple name variants."""
        for name in candidates:
            if name in fields:
                return fields[name]
        # Case-insensitive fallback
        fields_lower = {k.lower(): v for k, v in fields.items()}
        for name in candidates:
            if name.lower() in fields_lower:
                return fields_lower[name.lower()]
        return None

    # Extract GPS coordinates
    points = []
    for idx, fields in gps_frames:
        lat = _gps_field(fields, "GPS_coord[0]", "GPS_coord_0", "gps_coord[0]", "GPS_lat", "gps_lat")
        lon = _gps_field(fields, "GPS_coord[1]", "GPS_coord_1", "gps_coord[1]", "GPS_lon", "gps_lon")
        if lat is None or lon is None:
            # Debug: log what fields are available on first frame
            if idx == gps_frames[0][0] and not points:
                pass  # fields not matching — skip silently
            continue
        try:
            lat_deg = float(lat) / 1e7
            lon_deg = float(lon) / 1e7
        except (ValueError, TypeError):
            continue
        if abs(lat_deg) < 0.1 and abs(lon_deg) < 0.1:
            continue  # no fix

        speed = _gps_field(fields, "GPS_speed", "GPS_ground_speed", "gps_speed") or 0
        alt = _gps_field(fields, "GPS_altitude", "GPS_alt", "gps_altitude") or 0
        sats = _gps_field(fields, "GPS_numSat", "GPS_numsat", "gps_numsat") or 0
        time_s = idx / sr if sr > 0 else 0

        try:
            speed = float(speed)
            alt = float(alt)
            sats = int(sats)
        except (ValueError, TypeError):
            speed = 0
            alt = 0
            sats = 0

        points.append({
            "lat": lat_deg, "lon": lon_deg,
            "speed": speed, "alt": alt,
            "sats": sats, "t": round(time_s, 1),
            "idx": idx,
        })

    if len(points) < 5:
        return ""

    # Get flight mode at each GPS point using slow frames
    slow_frames = data.get("_slow_frames", [])
    mode_transitions = []
    for frame_idx, fields in slow_frames:
        mode_flags = fields.get("flightModeFlags")
        if mode_flags is None:
            for k in fields:
                if "flight" in k.lower() and "mode" in k.lower():
                    mode_flags = fields[k]
                    break
        if mode_flags is not None:
            try:
                flags = int(mode_flags)
                mode_transitions.append((frame_idx, flags))
            except (ValueError, TypeError):
                pass

    # Assign mode to each GPS point
    MODE_POSHOLD = 8
    MODE_RTH = 7
    MODE_ALTHOLD = 3
    MODE_ARM = 0

    def get_mode_at(idx):
        """Get flight mode name at a frame index."""
        flags = 0
        for fi, fl in mode_transitions:
            if fi <= idx:
                flags = fl
            else:
                break
        if flags & (1 << MODE_RTH):
            return "rth"
        if flags & (1 << MODE_POSHOLD):
            return "poshold"
        if flags & (1 << MODE_ALTHOLD):
            return "althold"
        if flags & (1 << MODE_ARM):
            return "manual"
        return "disarmed"

    for p in points:
        p["mode"] = get_mode_at(p["idx"])

    # Downsample to ~500 points
    if len(points) > 500:
        step = len(points) // 500
        points = points[::step]

    # Home position (first valid point)
    home = points[0]

    # Deceleration events from nav_perf
    decel_markers = []
    if nav_perf and nav_perf.get("deceleration"):
        for ev in nav_perf["deceleration"]:
            ev_t = ev["time_s"]
            # Find closest GPS point
            closest = min(points, key=lambda p: abs(p["t"] - ev_t))
            decel_markers.append({
                "lat": closest["lat"], "lon": closest["lon"],
                "t": ev_t, "overshoot_cm": ev["peak_error_cm"],
            })

    # Poshold CEP circle
    poshold_circle = None
    if nav_perf and nav_perf.get("poshold"):
        ph = nav_perf["poshold"]
        # Find center of poshold (average of poshold points)
        ph_points = [p for p in points if p["mode"] == "poshold"]
        if ph_points:
            center_lat = sum(p["lat"] for p in ph_points) / len(ph_points)
            center_lon = sum(p["lon"] for p in ph_points) / len(ph_points)
            poshold_circle = {
                "lat": center_lat, "lon": center_lon,
                "cep_m": ph["cep_cm"] / 100,
                "max_drift_m": ph["max_drift_cm"] / 100,
                "toilet_bowl": ph.get("toilet_bowl", False),
            }

    import json as _json
    map_data = _json.dumps({
        "points": [{"lat": p["lat"], "lon": p["lon"], "speed": p["speed"],
                     "alt": p["alt"], "mode": p["mode"], "t": p["t"]}
                    for p in points],
        "home": {"lat": home["lat"], "lon": home["lon"]},
        "decel": decel_markers,
        "poshold_circle": poshold_circle,
    })

    mode_colors = {
        "poshold": "#2d8a30",
        "rth": "#2563eb",
        "althold": "#d97706",
        "manual": "#6b7280",
        "disarmed": "#9ca3af",
    }

    return f'''<section id="sec-map"><h2>Flight Track</h2>
<div class="cc" style="padding:0;overflow:hidden;border-radius:8px">
<div id="flight-map" style="height:500px;width:100%"></div>
</div>
<div style="display:flex;gap:16px;margin:8px 0;font-size:.75rem;color:var(--dm)">
<span><span style="color:#2d8a30">━</span> POSHOLD</span>
<span><span style="color:#2563eb">━</span> RTH</span>
<span><span style="color:#d97706">━</span> ALTHOLD</span>
<span><span style="color:#6b7280">━</span> Manual</span>
<span><span style="color:#f7768e">●</span> Decel overshoot</span>
<span><span style="color:#bb9af7">◯</span> CEP circle</span>
</div>

<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
(function() {{
const D = {map_data};
const modeColors = {_json.dumps(mode_colors)};

const map = L.map('flight-map', {{
    zoomControl: true,
    attributionControl: true,
}});
window._flightMap = map;

const streets = L.tileLayer('https://{{s}}.basemaps.cartocdn.com/rastertiles/voyager/{{z}}/{{x}}/{{y}}@2x.png', {{
    attribution: '© <a href="https://www.openstreetmap.org/copyright">OSM</a> © <a href="https://carto.com/">CARTO</a>',
    maxZoom: 19,
    subdomains: 'abcd',
}});
const satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
    attribution: '© Esri, Maxar, Earthstar Geographics',
    maxZoom: 19,
}});
streets.addTo(map);
L.control.layers({{ "Street": streets, "Satellite": satellite }}, {{}}, {{ position: 'topright' }}).addTo(map);

// Draw track segments colored by mode
let currentMode = null;
let segment = [];

function flushSegment() {{
    if (segment.length > 1 && currentMode) {{
        L.polyline(segment, {{
            color: modeColors[currentMode] || '#6b7280',
            weight: 4, opacity: 0.9,
        }}).addTo(map);
    }}
    segment = [];
}}

D.points.forEach(p => {{
    if (p.mode !== currentMode) {{
        flushSegment();
        currentMode = p.mode;
    }}
    segment.push([p.lat, p.lon]);
}});
flushSegment();

// Home marker
L.circleMarker([D.home.lat, D.home.lon], {{
    radius: 8, color: '#ff9e64', fillColor: '#ff9e64',
    fillOpacity: 0.9, weight: 2,
}}).addTo(map).bindPopup('<b>Home</b>');

// Deceleration overshoot markers
D.decel.forEach(d => {{
    L.circleMarker([d.lat, d.lon], {{
        radius: 7, color: '#f7768e', fillColor: '#f7768e',
        fillOpacity: 0.8, weight: 2,
    }}).addTo(map).bindPopup(
        `<b>Decel overshoot</b><br>${{d.overshoot_cm.toFixed(0)}}cm at ${{d.t.toFixed(1)}}s`
    );
}});

// Poshold CEP circle
if (D.poshold_circle) {{
    const pc = D.poshold_circle;
    L.circle([pc.lat, pc.lon], {{
        radius: pc.cep_m, color: '#bb9af7', fillColor: '#bb9af7',
        fillOpacity: 0.1, weight: 2, dashArray: '5,5',
    }}).addTo(map).bindPopup(
        `<b>POSHOLD CEP</b><br>${{pc.cep_m.toFixed(1)}}m (50th pct)<br>Max drift: ${{pc.max_drift_m.toFixed(1)}}m`
        + (pc.toilet_bowl ? '<br><span style="color:#f7768e">Toilet bowl detected</span>' : '')
    );
    // Max drift circle
    L.circle([pc.lat, pc.lon], {{
        radius: pc.max_drift_m, color: '#f7768e',
        fillOpacity: 0, weight: 1, dashArray: '3,6',
    }}).addTo(map);
}}

// Track hover popup
const trackPoints = D.points.map(p => [p.lat, p.lon]);
if (trackPoints.length > 0) {{
    // Fit bounds
    const bounds = L.latLngBounds(trackPoints);
    map.fitBounds(bounds, {{ padding: [30, 30] }});
}}

// Click on track for info
D.points.forEach((p, i) => {{
    if (i % 5 !== 0) return;  // every 5th point for performance
    L.circleMarker([p.lat, p.lon], {{
        radius: 2, color: modeColors[p.mode] || '#7982a9',
        fillOpacity: 0.5, weight: 0,
    }}).addTo(map).bindPopup(
        `<b>${{p.t.toFixed(1)}}s</b><br>Speed: ${{(p.speed/100).toFixed(1)}} m/s<br>` +
        `Alt: ${{(p.alt/100).toFixed(0)}}m<br>Mode: ${{p.mode}}`
    );
}});
}})();
</script>
</section>'''


def _generate_nav_perf_html(nav_perf):
    """Generate HTML section for nav controller performance."""
    if not nav_perf or not nav_perf.get("findings"):
        return ""

    html = '<section id="sec-nav"><h2>Navigation Performance</h2>'
    nav_score = nav_perf.get("score")
    if nav_score is not None:
        sc = "good" if nav_score >= 85 else "warn" if nav_score >= 60 else "bad"
        html += f'<div class="cc"><span class="{sc}" style="font-size:1.2rem">Nav Score: {nav_score}/100</span></div>'

    # Deceleration
    decel = nav_perf.get("deceleration", [])
    if decel:
        avg_peak = sum(e["peak_error_cm"] for e in decel) / len(decel)
        avg_osc = sum(e["oscillation_count"] for e in decel) / len(decel)
        avg_settle = sum(e["settling_time_s"] for e in decel) / len(decel)
        worst = max(decel, key=lambda e: e["peak_error_cm"])
        html += f'<div class="cc"><h3>Deceleration ({len(decel)} events)</h3>'
        html += f'<table><tr><th>Metric</th><th>Value</th></tr>'
        html += f'<tr><td>Avg overshoot</td><td>{avg_peak:.0f}cm</td></tr>'
        html += f'<tr><td>Avg oscillations</td><td>{avg_osc:.1f}</td></tr>'
        html += f'<tr><td>Avg settling time</td><td>{avg_settle:.1f}s</td></tr>'
        html += f'<tr><td>Worst overshoot</td><td class="bad">{worst["peak_error_cm"]:.0f}cm at {worst["time_s"]:.1f}s</td></tr>'
        html += '</table></div>'

    # Position hold
    ph = nav_perf.get("poshold")
    if ph:
        cep_c = "good" if ph["cep_cm"] < 150 else "warn" if ph["cep_cm"] < 400 else "bad"
        html += f'<div class="cc"><h3>Position Hold ({ph["hold_duration_s"]:.0f}s)</h3>'
        html += f'<table><tr><th>Metric</th><th>Value</th></tr>'
        html += f'<tr><td>CEP</td><td class="{cep_c}">{ph["cep_cm"]:.0f}cm</td></tr>'
        html += f'<tr><td>Max drift</td><td>{ph["max_drift_cm"]:.0f}cm</td></tr>'
        html += f'<tr><td>RMS error</td><td>{ph["rms_error_cm"]:.0f}cm</td></tr>'
        if ph.get("toilet_bowl"):
            html += f'<tr><td>Toilet bowl</td><td class="bad">Detected (period {ph["tb_period_s"]:.1f}s)</td></tr>'
        html += '</table></div>'

    # Altitude hold
    ah = nav_perf.get("althold")
    if ah:
        alt_c = "good" if ah["rms_error_cm"] < 50 else "warn" if ah["rms_error_cm"] < 150 else "bad"
        html += f'<div class="cc"><h3>Altitude Hold ({ah["hold_duration_s"]:.0f}s)</h3>'
        html += f'<table><tr><th>Metric</th><th>Value</th></tr>'
        html += f'<tr><td>RMS error</td><td class="{alt_c}">{ah["rms_error_cm"]:.0f}cm</td></tr>'
        html += f'<tr><td>Max error</td><td>{ah["max_error_cm"]:.0f}cm</td></tr>'
        if ah.get("oscillation"):
            html += f'<tr><td>Oscillation</td><td class="warn">{ah["osc_freq_hz"]:.1f}Hz</td></tr>'
        html += '</table></div>'

    # Findings
    for f in nav_perf.get("findings", []):
        fc = "warning" if f["level"] == "WARNING" else "info"
        html += f'<div class="nav-findings"><div class="finding {fc}">{f["text"]}</div></div>'

    # Nav actions
    nav_actions = nav_perf.get("nav_actions", [])
    if nav_actions:
        html += '<div class="cc"><h3>Nav PID Recommendations</h3>'
        for a in nav_actions:
            html += f'<div style="margin:8px 0"><strong>{a["action"]}</strong>'
            html += f'<div style="color:var(--dm);font-size:.8rem">{a["reason"]}</div>'
            html += f'<code style="color:var(--gn)">{a["cli"]}</code></div>'
        html += '</div>'

    html += '</section>'
    return html


def _generate_whatif_html(config, plan, pid_results, noise_results):
    """Generate the interactive What-If simulation section for the HTML report."""

    # Extract current values
    axes = ["roll", "pitch", "yaw"]
    current = {}
    for ax in axes:
        current[ax] = {
            "p": config.get(f"{ax}_p", 40),
            "i": config.get(f"{ax}_i", 30),
            "d": config.get(f"{ax}_d", 25),
            "ff": config.get(f"{ax}_ff", 60),
        }

    # Extract recommended values from plan actions
    recommended = {ax: dict(current[ax]) for ax in axes}
    for a in plan.get("actions", []):
        if a.get("deferred"):
            continue
        subs = a.get("sub_actions", [])
        for sa in subs:
            param = sa.get("param", "")
            for ax in axes:
                for term in ["p", "i", "d", "ff"]:
                    if param == f"{ax}_{term}":
                        recommended[ax][term] = sa.get("new", current[ax][term])

    # Extract measurements
    measurements = {}
    for pid in (pid_results or []):
        if pid is None:
            continue
        ax = pid["axis"].lower()
        measurements[ax] = {
            "overshoot": pid.get("avg_overshoot_pct"),
            "delay": pid.get("tracking_delay_ms"),
        }

    # Filter values
    gyro_lpf = config.get("gyro_lowpass_hz", 100)
    dterm_lpf = config.get("dterm_lpf_hz", 100)

    # Downsample spectrum for embedding (max 200 points)
    spectrum_data = []
    for nr in (noise_results or []):
        if nr is None:
            continue
        freqs = nr["freqs"]
        psd = nr["psd_db"]
        if hasattr(freqs, 'tolist'):
            # Downsample to 200 points
            step = max(1, len(freqs) // 200)
            spectrum_data.append({
                "axis": nr["axis"],
                "freqs": [float(f) for f in freqs[::step]],
                "psd": [float(p) for p in psd[::step]],
            })

    # Build JSON data for JS
    import json as _json
    whatif_data = _json.dumps({
        "current": current,
        "recommended": recommended,
        "measurements": measurements,
        "gyro_lpf": gyro_lpf,
        "dterm_lpf": dterm_lpf,
        "spectrum": spectrum_data,
    })

    return f'''<section id="sec-whatif"><h2>What-If Simulation</h2>
<div class="cc" style="padding:20px">
<p style="color:var(--dm);font-size:.8rem;margin-bottom:16px">
Explore how PID and filter changes would affect your tune. The "Current" column shows what was flying.
"Recommended" shows the analyzer's suggestions. Adjust the sliders in "Your Scenario" to see predicted effects.
<strong>Predictions are estimates</strong> — fly and re-analyze for real data.</p>

<div id="wif-container"></div>

<div style="margin-top:20px">
<h3>Filter Preview</h3>
<p style="color:var(--dm);font-size:.8rem">Drag the slider to see how different LPF cutoff frequencies affect the noise spectrum.</p>
<div style="display:flex;align-items:center;gap:16px;margin:12px 0">
  <label style="color:var(--bl)">Gyro LPF:</label>
  <input type="range" id="wif-glpf" min="20" max="200" value="{gyro_lpf}" style="flex:1">
  <span id="wif-glpf-val" style="color:var(--gn);min-width:50px">{gyro_lpf}Hz</span>
  <span id="wif-glpf-lag" style="color:var(--dm);font-size:.8rem"></span>
</div>
<canvas id="wif-spectrum" width="800" height="250" style="width:100%;background:var(--bg);border:1px solid var(--bd);border-radius:4px"></canvas>
</div>

<div style="margin-top:20px;text-align:center">
<button id="wif-copy" style="background:var(--bl);color:var(--bg);border:none;padding:10px 24px;border-radius:6px;cursor:pointer;font-weight:600;font-size:.9rem">
Copy CLI Commands</button>
<span id="wif-copied" style="color:var(--gn);margin-left:12px;display:none">Copied!</span>
</div>
</div>

<script>
(function() {{
const D = {whatif_data};
const axes = ["roll","pitch","yaw"];
const terms = ["p","i","d","ff"];
const termLabels = {{"p":"P","i":"I","d":"D","ff":"FF"}};
const cliMap = {{"roll_p":"mc_p_roll","roll_i":"mc_i_roll","roll_d":"mc_d_roll","roll_ff":"mc_cd_roll",
"pitch_p":"mc_p_pitch","pitch_i":"mc_i_pitch","pitch_d":"mc_d_pitch","pitch_ff":"mc_cd_pitch",
"yaw_p":"mc_p_yaw","yaw_i":"mc_i_yaw","yaw_d":"mc_d_yaw","yaw_ff":"mc_cd_yaw"}};

// Build PID table
const ct = document.getElementById("wif-container");
let h = '<table style="width:100%"><tr><th>Axis</th><th>Term</th><th>Current</th><th>Recommended</th><th colspan="2">Your Scenario</th><th>Predicted Effect</th></tr>';

axes.forEach(ax => {{
  terms.forEach((t,ti) => {{
    const cur = D.current[ax][t];
    const rec = D.recommended[ax][t];
    const maxVal = t==="ff"?120:t==="p"?200:t==="i"?200:150;
    const axLabel = ti===0 ? `<td rowspan="4" class="ax-${{ax}}" style="font-weight:700;font-size:1.1rem">${{ax.charAt(0).toUpperCase()+ax.slice(1)}}</td>` : "";
    const changed = rec !== cur;
    const recStyle = changed ? 'style="color:var(--yl)"' : '';
    h += `<tr>${{axLabel}}<td>${{termLabels[t]}}</td><td>${{cur}}</td><td ${{recStyle}}>${{rec}}</td>`;
    h += `<td><input type="range" id="wif-${{ax}}-${{t}}" min="0" max="${{maxVal}}" value="${{rec}}" style="width:120px"></td>`;
    h += `<td id="wif-${{ax}}-${{t}}-val" style="min-width:30px;color:var(--gn)">${{rec}}</td>`;
    h += `<td id="wif-${{ax}}-${{t}}-fx" style="color:var(--dm);font-size:.8rem"></td></tr>`;
  }});
}});
h += '</table>';

// Prediction display
h += '<div style="margin-top:16px;display:grid;grid-template-columns:repeat(3,1fr);gap:12px">';
axes.forEach(ax => {{
  h += `<div class="cc" style="text-align:center"><h3 class="ax-${{ax}}">${{ax.charAt(0).toUpperCase()+ax.slice(1)}}</h3>`;
  h += `<div id="wif-pred-${{ax}}" style="font-size:.85rem"></div></div>`;
}});
h += '</div>';
ct.innerHTML = h;

// Simple prediction model
function predict(ax) {{
  const p = parseInt(document.getElementById(`wif-${{ax}}-p`).value);
  const d = parseInt(document.getElementById(`wif-${{ax}}-d`).value);
  const ff = parseInt(document.getElementById(`wif-${{ax}}-ff`).value);
  const curP = D.current[ax].p;
  const curD = D.current[ax].d;
  const curFF = D.current[ax].ff;
  const m = D.measurements[ax];

  if (!m || m.overshoot===null) {{
    document.getElementById(`wif-pred-${{ax}}`).innerHTML = '<span style="color:var(--dm)">No measurement data</span>';
    return;
  }}

  // Overshoot model: scales with effective gain (P + FF*0.5) / damping (D)
  const curGain = curP + curFF * 0.5;
  const curDamp = Math.max(curD, 1);
  const newGain = p + ff * 0.5;
  const newDamp = Math.max(d, 1);
  const gainRatio = (newGain / newDamp) / (curGain / curDamp);
  const predOS = Math.max(0, m.overshoot * gainRatio);

  // Delay model: inversely proportional to P
  const predDelay = m.delay ? m.delay * (curP / Math.max(p, 5)) : null;

  const osColor = predOS > 30 ? "var(--rd)" : predOS > 15 ? "var(--yl)" : "var(--gn)";
  const dlColor = predDelay && predDelay > 35 ? "var(--yl)" : "var(--gn)";

  let html = `<div>Overshoot: <span style="color:var(--dm)">${{m.overshoot.toFixed(0)}}%</span> -> <span style="color:${{osColor}}">${{predOS.toFixed(0)}}%</span></div>`;
  if (predDelay !== null) {{
    html += `<div>Delay: <span style="color:var(--dm)">${{m.delay.toFixed(0)}}ms</span> -> <span style="color:${{dlColor}}">${{predDelay.toFixed(0)}}ms</span></div>`;
  }}
  document.getElementById(`wif-pred-${{ax}}`).innerHTML = html;
}}

// Slider event listeners
axes.forEach(ax => {{
  terms.forEach(t => {{
    const sl = document.getElementById(`wif-${{ax}}-${{t}}`);
    sl.addEventListener("input", () => {{
      document.getElementById(`wif-${{ax}}-${{t}}-val`).textContent = sl.value;
      const cur = D.current[ax][t];
      const diff = parseInt(sl.value) - cur;
      const fx = document.getElementById(`wif-${{ax}}-${{t}}-fx`);
      if (diff > 0) fx.textContent = `+${{diff}} from current`;
      else if (diff < 0) fx.textContent = `${{diff}} from current`;
      else fx.textContent = "= current";
      predict(ax);
    }});
    // Trigger initial prediction
    sl.dispatchEvent(new Event("input"));
  }});
}});

// Spectrum canvas
const canvas = document.getElementById("wif-spectrum");
const ctx = canvas.getContext("2d");
const glpfSlider = document.getElementById("wif-glpf");
const glpfVal = document.getElementById("wif-glpf-val");
const glpfLag = document.getElementById("wif-glpf-lag");

function drawSpectrum() {{
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!D.spectrum || !D.spectrum.length) return;

  const maxFreq = 500;
  const minDb = -80, maxDb = 0;
  const xScale = w / maxFreq;
  const yScale = h / (maxDb - minDb);
  const cutoff = parseInt(glpfSlider.value);

  // Draw grid
  ctx.strokeStyle = "#2a2d3e";
  ctx.lineWidth = 0.5;
  for (let f = 0; f <= maxFreq; f += 50) {{
    const x = f * xScale;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.fillStyle = "#565f89"; ctx.font = "10px monospace";
    ctx.fillText(f + "Hz", x + 2, h - 4);
  }}

  // Draw spectra
  const colors = ["#ff6b6b","#4ecdc4","#ffd93d"];
  D.spectrum.forEach((sp, si) => {{
    if (!sp.freqs.length) return;

    // Original spectrum
    ctx.strokeStyle = colors[si % 3];
    ctx.globalAlpha = 0.4;
    ctx.lineWidth = 1;
    ctx.beginPath();
    sp.freqs.forEach((f, i) => {{
      const x = f * xScale;
      const y = h - (sp.psd[i] - minDb) * yScale;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }});
    ctx.stroke();

    // Filtered spectrum (after LPF)
    ctx.strokeStyle = colors[si % 3];
    ctx.globalAlpha = 1.0;
    ctx.lineWidth = 2;
    ctx.beginPath();
    sp.freqs.forEach((f, i) => {{
      const x = f * xScale;
      // PT1 filter attenuation: -20dB/decade above cutoff
      const ratio = f / cutoff;
      const attenuation = -10 * Math.log10(1 + ratio * ratio);
      const filteredDb = sp.psd[i] + attenuation;
      const y = h - (filteredDb - minDb) * yScale;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }});
    ctx.stroke();
  }});
  ctx.globalAlpha = 1.0;

  // Draw cutoff line
  const cx = cutoff * xScale;
  ctx.strokeStyle = "#9ece6a";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#9ece6a"; ctx.font = "12px monospace";
  ctx.fillText(cutoff + "Hz", cx + 4, 16);

  // Legend
  ctx.font = "10px monospace";
  ctx.globalAlpha = 0.4;
  ctx.fillStyle = colors[0]; ctx.fillText("Roll (raw)", 10, 16);
  ctx.fillStyle = colors[1]; ctx.fillText("Pitch (raw)", 10, 28);
  ctx.fillStyle = colors[2]; ctx.fillText("Yaw (raw)", 10, 40);
  ctx.globalAlpha = 1.0;
  ctx.fillStyle = "#c0caf5"; ctx.fillText("Bold = filtered", 10, 56);

  // Phase lag at 50Hz for this cutoff
  const sigFreq = 50;
  const lagDeg = Math.atan(sigFreq / cutoff) * (180 / Math.PI);
  const lagMs = lagDeg / (360 * sigFreq) * 1000;
  glpfVal.textContent = cutoff + "Hz";
  glpfLag.textContent = `Phase lag: ${{lagDeg.toFixed(0)}} deg / ${{lagMs.toFixed(1)}}ms at ${{sigFreq}}Hz`;
}}

glpfSlider.addEventListener("input", drawSpectrum);
drawSpectrum();

// Copy CLI button
document.getElementById("wif-copy").addEventListener("click", () => {{
  let cmds = [];
  axes.forEach(ax => {{
    terms.forEach(t => {{
      const val = parseInt(document.getElementById(`wif-${{ax}}-${{t}}`).value);
      const cur = D.current[ax][t];
      if (val !== cur) {{
        const key = cliMap[ax + "_" + t];
        if (key) cmds.push(`set ${{key}} = ${{val}}`);
      }}
    }});
  }});
  const glpf = parseInt(glpfSlider.value);
  if (glpf !== D.gyro_lpf) cmds.push(`set gyro_main_lpf_hz = ${{glpf}}`);
  if (cmds.length) cmds.push("save");
  else cmds.push("# No changes from current config");

  navigator.clipboard.writeText(cmds.join("\\n")).then(() => {{
    const cp = document.getElementById("wif-copied");
    cp.style.display = "inline";
    setTimeout(() => cp.style.display = "none", 2000);
  }});
}});
}})();
</script>
</section>'''


def _generate_history_html(flight_diff, prog_data):
    """Generate HTML section for flight history with chart and diff."""
    if not flight_diff and not prog_data:
        return ""

    html = ""

    # Flight-to-flight diff
    if flight_diff and flight_diff.get("has_previous"):
        prev = flight_diff["previous"]
        curr = flight_diff["current"]
        delta = flight_diff["score_delta"]
        dc = "good" if delta > 0 else "bad" if delta < 0 else "warn"
        arrow = "↗" if delta > 0 else "↘" if delta < 0 else "→"

        html += '<section><h2>vs Previous Flight</h2>'
        html += f'<div class="cc"><span class="{dc}" style="font-size:1.3rem">'
        html += f'Score: {prev["score"]:.0f} → {curr["score"]:.0f}  {arrow} {delta:+.0f}</span></div>'

        # Metric changes
        metrics = flight_diff.get("metric_changes", [])
        if metrics:
            html += '<div class="cc"><h3>Metric Changes</h3><table>'
            html += '<tr><th>Metric</th><th>Previous</th><th>Current</th><th>Direction</th></tr>'
            for m in metrics:
                mc = "good" if m["direction"] == "better" else "bad"
                ax_str = f'{m["axis"]} ' if m["axis"] else ""
                old_str = f'{m["old"]:.1f}{m["unit"]}' if isinstance(m["old"], float) else f'{m["old"]}{m["unit"]}'
                new_str = f'{m["new"]:.1f}{m["unit"]}' if isinstance(m["new"], float) else f'{m["new"]}{m["unit"]}'
                html += f'<tr><td>{ax_str}{m["metric"]}</td><td>{old_str}</td>'
                html += f'<td class="{mc}">{new_str}</td>'
                html += f'<td class="{mc}">{"↗" if m["direction"] == "better" else "↘"} {m["direction"]}</td></tr>'
            html += '</table></div>'

        # Config changes
        config_changes = flight_diff.get("config_changes", [])
        if config_changes:
            html += '<div class="cc"><h3>Config Changes Since Last Flight</h3><table>'
            html += '<tr><th>Parameter</th><th>Previous</th><th>Current</th></tr>'
            for ch in config_changes:
                html += f'<tr><td style="color:var(--bl)">{ch["param"]}</td>'
                html += f'<td>{ch["old"]}</td><td>{ch["new"]}</td></tr>'
            html += '</table></div>'

        # Verdict
        verdict = flight_diff.get("verdict", "")
        if verdict:
            vc = "good" if delta > 0 else "warn" if abs(delta) < 3 else "bad"
            html += f'<div class="cc"><span class="{vc}">{verdict}</span></div>'
        html += '</section>'

    # Flight progression chart
    if prog_data and len(prog_data.get("flights", [])) >= 2:
        flights = prog_data["flights"]
        import json as _json

        chart_data = _json.dumps([{
            "id": f.get("id"),
            "score": f.get("score"),
            "noise": f.get("noise"),
            "pid": f.get("pid"),
            "motor": f.get("motor"),
        } for f in flights])

        html += f'''<section><h2>Score Progression</h2>
<div class="cc" style="padding:20px">
<canvas id="hist-chart" width="700" height="250" style="width:100%;background:var(--bg);border-radius:4px"></canvas>
</div>

<div class="cc"><h3>Flight Table</h3><table>
<tr><th>#</th><th>Score</th><th>Noise</th><th>PID</th><th>Motor</th></tr>'''

        for f in flights:
            sc = "good" if (f.get("score") or 0) >= 85 else "warn" if (f.get("score") or 0) >= 60 else "bad"
            score = f'{f["score"]:.0f}' if f.get("score") is not None else "-"
            noise = f'{f["noise"]:.0f}' if f.get("noise") is not None else "-"
            pid = f'{f["pid"]:.0f}' if f.get("pid") is not None else "-"
            motor = f'{f["motor"]:.0f}' if f.get("motor") is not None else "-"
            html += f'<tr><td class="{sc}">{f.get("id","?")}</td><td class="{sc}">{score}</td>'
            html += f'<td>{noise}</td><td>{pid}</td><td>{motor}</td></tr>'

        html += f'''</table></div>

<script>
(function() {{
const flights = {chart_data};
const canvas = document.getElementById("hist-chart");
if (!canvas) return;
const ctx = canvas.getContext("2d");
const w = canvas.width, h = canvas.height;
const pad = {{t:20,r:20,b:30,l:40}};
const pw = w-pad.l-pad.r, ph = h-pad.t-pad.b;

// Grid
ctx.strokeStyle = "#2a2d3e"; ctx.lineWidth = 0.5;
for (let y = 0; y <= 100; y += 20) {{
  const py = pad.t + ph * (1 - y/100);
  ctx.beginPath(); ctx.moveTo(pad.l,py); ctx.lineTo(w-pad.r,py); ctx.stroke();
  ctx.fillStyle = "#565f89"; ctx.font = "10px monospace";
  ctx.fillText(y+"", pad.l-25, py+4);
}}

function drawLine(key, color) {{
  const vals = flights.map(f => f[key]);
  if (vals.every(v => v === null)) return;
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  vals.forEach((v,i) => {{
    if (v === null) return;
    const x = pad.l + (i/(flights.length-1||1)) * pw;
    const y = pad.t + ph * (1 - v/100);
    if (!started) {{ ctx.moveTo(x,y); started = true; }}
    else ctx.lineTo(x,y);
    // Dot
    ctx.fillStyle = color;
    ctx.fillRect(x-3,y-3,6,6);
  }});
  ctx.stroke();
}}

drawLine("score", "#7aa2f7");
drawLine("noise", "#9ece6a");
drawLine("pid", "#bb9af7");
drawLine("motor", "#e0af68");

// Legend
ctx.font = "10px monospace";
[["Score","#7aa2f7"],["Noise","#9ece6a"],["PID","#bb9af7"],["Motor","#e0af68"]].forEach(([l,c],i) => {{
  ctx.fillStyle = c;
  ctx.fillRect(pad.l + i*80, h-12, 10, 10);
  ctx.fillText(l, pad.l + i*80 + 14, h-3);
}});

// X labels
ctx.fillStyle = "#565f89"; ctx.font = "10px monospace";
flights.forEach((f,i) => {{
  const x = pad.l + (i/(flights.length-1||1)) * pw;
  ctx.fillText("#"+f.id, x-5, h-16);
}});
}})();
</script>
</section>'''

    return html


def generate_html_report(plan, noise_results, pid_results, motor_analysis, dterm_results, config, data, charts, nav_results=None, nav_perf=None, flight_diff=None, prog_data=None, preflight=None, accel_vib=None, recipe=None, power_results=None, propwash_results=None, failsafe_results=None):
    scores = plan["scores"]
    overall = scores["overall"]
    sg = "linear-gradient(90deg, #9ece6a, #73daca)" if overall >= 85 else "linear-gradient(90deg, #e0af68, #ff9e64)" if overall >= 60 else "linear-gradient(90deg, #f7768e, #ff6b6b)"

    # Pre-flight checklist HTML
    pf_html = ""
    if preflight and preflight.get("items"):
        n_crit = preflight["n_critical"]
        n_warn = preflight["n_warning"]
        if n_crit:
            border_color = "var(--rd)"
            header = f"⚠ Pre-Flight Safety: {n_crit} critical issue{'s' if n_crit > 1 else ''}"
            if n_warn:
                header += f", {n_warn} warning{'s' if n_warn > 1 else ''}"
        elif n_warn:
            border_color = "var(--yl)"
            header = f"Pre-Flight: {n_warn} warning{'s' if n_warn > 1 else ''}"
        else:
            border_color = "var(--dm)"
            header = "Pre-Flight Notes"

        pf_html = f'<div class="cc" style="border-left:4px solid {border_color}">'
        pf_html += f'<h3 style="color:{border_color}">{header}</h3>'
        for item in preflight["items"]:
            ic = "var(--rd)" if item["level"] == "CRITICAL" else "var(--yl)" if item["level"] == "WARNING" else "var(--dm)"
            icon = "✗" if item["level"] == "CRITICAL" else "⚠" if item["level"] == "WARNING" else "ℹ"
            pf_html += f'<div style="margin:8px 0;color:{ic}"><strong>{icon}</strong> {item["text"]}</div>'
            if item.get("fix"):
                for fix_line in item["fix"].split(" && "):
                    pf_html += f'<div style="margin-left:20px;color:var(--gn);font-size:.8rem">Fix: {fix_line}</div>'
        pf_html += '</div>'

    ah = ""
    active_actions = [a for a in plan["actions"] if not a.get("deferred")]
    deferred_actions = [a for a in plan["actions"] if a.get("deferred")]

    for i, a in enumerate(active_actions, 1):
        uc = (a["urgency"] or "normal").lower()
        ub = f'<span class="ub {uc}">{a["urgency"]}</span>' if a["urgency"] else ""
        ah += f'<div class="ac {uc}"><div class="an">{i}</div><div class="ab"><div class="am">{a["action"]} {ub}</div><div class="ar">{a["reason"]}</div></div></div>'
    if deferred_actions:
        ah += '<div class="ac deferred"><div class="ab"><div class="am" style="color:#565f89">⏸ Deferred PID changes (fix filters first, then re-fly)</div>'
        for a in deferred_actions:
            orig = a.get("original_action", a["action"].replace("[DEFERRED] ", ""))
            ah += f'<div class="ar" style="color:#565f89">{orig}</div>'
        ah += '</div></div>'
    if not active_actions and not deferred_actions:
        ah = '<div class="ac ok"><div class="ab"><div class="am">✓ No changes needed, go fly!</div></div></div>'

    ch = ""
    if config_has_pid(config) or config_has_filters(config):
        cr = ""
        for ax in ["roll","pitch","yaw"]:
            cr += f'<tr><td class="ax-{ax}">{ax.capitalize()}</td><td>{config.get(f"{ax}_p","-")}</td><td>{config.get(f"{ax}_i","-")}</td><td>{config.get(f"{ax}_d","-")}</td></tr>'
        fi = ""
        for k, l in [("gyro_lowpass_hz","Gyro LPF"),("dterm_lpf_hz","D-term LPF"),("yaw_lpf_hz","Yaw LPF"),("gyro_lowpass2_hz","Gyro LPF2")]:
            v = config.get(k)
            if v is not None and v != 0: fi += f'<span class="fc">{l}: {v}Hz</span>'
        # Dynamic notch status
        dyn_en = config.get("dyn_notch_enabled")
        dyn_q = config.get("dyn_notch_q")
        dyn_min = config.get("dyn_notch_min_hz")
        if dyn_en and dyn_en not in (0, "0", "OFF"):
            dyn_detail = f"ON (Q={dyn_q}, min={dyn_min}Hz)" if dyn_q else "ON"
            fi += f'<span class="fc">Dyn Notch: {dyn_detail}</span>'
        elif dyn_en is not None:
            fi += f'<span class="fc">Dyn Notch: OFF</span>'
        # RPM filter status
        rpm_en = config.get("rpm_filter_enabled")
        if rpm_en is not None:
            rpm_str = "ON" if rpm_en and rpm_en not in (0, "0", "OFF") else "OFF"
            fi += f'<span class="fc">RPM Filter: {rpm_str}</span>'
        ch = f'<section><h2>Current Config</h2><div class="cg"><div class="cc"><h3>PID</h3><table><tr><th>Axis</th><th>P</th><th>I</th><th>D</th></tr>{cr}</table></div><div class="cc"><h3>Filters</h3><div class="fcs">{fi or "<span class=fc>No filter values in headers</span>"}</div></div></div></section>'

    pt = ""
    for pid in pid_results:
        if pid is None: continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        oc = ("bad" if _os>BAD_OVERSHOOT else "warn" if _os>OK_OVERSHOOT else "good") if _os is not None else "good"
        dc = ("bad" if _dl>BAD_DELAY_MS else "warn" if _dl>OK_DELAY_MS else "good") if _dl is not None else "good"
        dl_str = f"{_dl:.1f}ms" if _dl is not None else "N/A"
        os_str = f"{_os:.1f}%" if _os is not None else "N/A"
        pt += f'<tr><td class="ax-{pid["axis"].lower()}">{pid["axis"]}</td><td class="{dc}">{dl_str}</td><td class="{oc}">{os_str}</td><td>{pid["rms_error"]:.1f}</td></tr>'

    mt = ""
    if motor_analysis:
        if motor_analysis.get("idle_detected", False):
            mt = '<tr><td colspan="4" style="color:var(--dm);text-align:center">Idle/ground - no throttle variation</td></tr>'
        else:
            for m in motor_analysis["motors"]:
                sc = "bad" if m["saturation_pct"]>MOTOR_SAT_WARN else "good"
                mt += f'<tr><td>M{m["motor"]}</td><td>{m["avg_pct"]:.1f}%</td><td>{m["std_pct"]:.1f}%</td><td class="{sc}">{m["saturation_pct"]:.1f}%</td></tr>'

    ci = lambda k: f'<img src="data:image/png;base64,{charts[k]}">' if charts.get(k) else ""
    vc = plan["verdict"].lower().replace("_","-")

    # Build firmware string properly
    fw_str = config.get("firmware_revision", "")
    if not fw_str:
        fw_str = f"{config.get('firmware_version', '')}"
    if "INAV" not in fw_str and "inav" not in fw_str.lower():
        fw_str = f"INAV {fw_str}" if fw_str else "INAV"

    # Nav sections
    nav_perf_html = _generate_nav_perf_html(nav_perf) if nav_perf else ""
    nav_sensors_html = generate_nav_html_section(nav_results) if nav_results else ""
    map_html = _generate_map_html(data, nav_perf)
    whatif_html = _generate_whatif_html(config, plan, pid_results, noise_results)
    history_html = _generate_history_html(flight_diff, prog_data)
    has_nav = bool(nav_perf_html or nav_sensors_html or map_html)

    # CLI commands
    cli_html = ""
    active_actions = [a for a in plan["actions"] if not a.get("deferred")]
    cli_cmds = generate_cli_commands(active_actions)
    if cli_cmds:
        cli_html = '<div class="cc"><h3>Paste into INAV Configurator CLI tab:</h3><pre style="color:var(--gn);margin:12px 0">'
        cli_html += "\n".join(cli_cmds)
        cli_html += '</pre></div>'

    # Stale data warning
    stale_html = ""
    mismatches = config.get("_diff_mismatches", [])
    if len(mismatches) >= 3:
        stale_html = '<div class="cc" style="border-left:4px solid var(--yl);background:rgba(224,175,104,.06)">'
        stale_html += '<h3 style="color:var(--yl)">⚠ Stale Data</h3>'
        stale_html += f'<p style="color:var(--dm)">FC config has changed since this flight ({len(mismatches)} parameters differ). '
        stale_html += 'Analysis reflects what was flying, not the current FC config.</p>'
        stale_html += '<table style="margin-top:8px"><tr><th>Parameter</th><th>In Flight</th><th>FC Now</th></tr>'
        for param, bb_val, diff_val in mismatches[:10]:
            stale_html += f'<tr><td style="color:var(--bl)">{param}</td><td>{bb_val}</td><td class="warn">{diff_val}</td></tr>'
        stale_html += '</table></div>'

    # Info notes (wind buffeting, etc.)
    info_html = ""
    info_items = plan.get("info", [])
    if info_items:
        info_html = '<div class="cc"><h3>Notes</h3>'
        for item in info_items:
            info_html += f'<div style="margin:6px 0;color:var(--dm)">ℹ {item["text"]}</div>'
            if item.get("detail"):
                info_html += f'<div style="margin-left:16px;color:var(--dm);font-size:.8rem">{item["detail"]}</div>'
        info_html += '</div>'

    # Noise sources summary
    noise_summary_html = ""
    noise_fp = plan.get("noise_fingerprint")
    if noise_fp and noise_fp.get("peaks") and noise_fp.get("summary"):
        noise_summary_html = '<div class="cc"><h3>Noise Sources</h3>'
        sources = {}
        for pk in noise_fp["peaks"]:
            src = pk.get("source", "unknown")
            sources.setdefault(src, []).append(pk)
        for src, peaks in sources.items():
            src_label = src.replace("_", " ").capitalize()
            freq_range = f'{min(p["freq_hz"] for p in peaks):.0f}-{max(p["freq_hz"] for p in peaks):.0f}Hz' if len(peaks) > 1 else f'{peaks[0]["freq_hz"]:.0f}Hz'
            axes = ", ".join(sorted(set(a for p in peaks for a in (p.get("axes") or []))))
            noise_summary_html += f'<div style="margin:4px 0">• <strong>{src_label}</strong> at {freq_range}'
            if axes:
                noise_summary_html += f' on {axes}'
            noise_summary_html += '</div>'
        noise_summary_html += '</div>'

    # Compact measurements table
    meas_html = '<div class="cc"><h3>Measurements</h3>'
    # Hover oscillation
    hover_osc = plan["scores"].get("hover_osc", [])
    if hover_osc:
        meas_html += '<table><tr><th>Axis</th><th>Hover</th><th>RMS</th><th>P2P</th><th>Freq</th></tr>'
        for osc in hover_osc:
            sev = osc["severity"]
            sc2 = "bad" if sev == "severe" else "warn" if sev in ("moderate", "mild") else "good"
            freq_str = f'{osc["dominant_freq_hz"]:.0f}Hz' if osc.get("dominant_freq_hz") else "—"
            meas_html += f'<tr><td class="ax-{osc["axis"].lower()}">{osc["axis"]}</td>'
            meas_html += f'<td class="{sc2}">{sev}</td>'
            meas_html += f'<td>{osc["gyro_rms"]:.1f}°/s</td><td>{osc["gyro_p2p"]:.0f}°/s</td>'
            meas_html += f'<td>{freq_str}</td></tr>'
        meas_html += '</table>'

    # PID response
    meas_html += '<table style="margin-top:12px"><tr><th>Axis</th><th>Overshoot</th><th>Delay</th><th>RMS Error</th></tr>'
    for pid in pid_results:
        if pid is None:
            continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        oc = ("bad" if _os > BAD_OVERSHOOT else "warn" if _os > OK_OVERSHOOT else "good") if _os is not None else ""
        dc = ("bad" if _dl > BAD_DELAY_MS else "warn" if _dl > OK_DELAY_MS else "good") if _dl is not None else ""
        os_str = f"{_os:.1f}%" if _os is not None else "N/A"
        dl_str = f"{_dl:.1f}ms" if _dl is not None else "N/A"
        meas_html += f'<tr><td class="ax-{pid["axis"].lower()}">{pid["axis"]}</td>'
        meas_html += f'<td class="{oc}">{os_str}</td><td class="{dc}">{dl_str}</td>'
        meas_html += f'<td>{pid["rms_error"]:.1f}</td></tr>'
    meas_html += '</table>'

    # Motor balance
    if motor_analysis and not motor_analysis.get("idle_detected"):
        meas_html += '<table style="margin-top:12px"><tr><th>Motor</th><th>Avg</th><th>Saturation</th></tr>'
        for m in motor_analysis["motors"]:
            sc2 = "bad" if m["saturation_pct"] > MOTOR_SAT_WARN else "good"
            meas_html += f'<tr><td>M{m["motor"]}</td><td>{m["avg_pct"]:.1f}%</td>'
            meas_html += f'<td class="{sc2}">{m["saturation_pct"]:.1f}%</td></tr>'
        meas_html += '</table>'
    meas_html += '</div>'

    # Tuning recipe HTML
    recipe_html = ""
    if recipe:
        recipe_html = f'<div class="cc" style="border-left:4px solid var(--pp)"><h3 style="color:var(--pp)">Tuning Recipe: {recipe["recipe_name"]}</h3>'
        recipe_html += f'<p style="color:var(--dm);margin:8px 0">{recipe["description"]}</p>'
        recipe_html += '<pre style="color:var(--gn);margin:8px 0">'
        recipe_html += "\n".join(recipe["cli_commands"])
        recipe_html += '</pre><div style="margin-top:8px">'
        for r in recipe["reasoning"]:
            recipe_html += f'<div style="color:var(--dm);font-size:.8rem">• {r}</div>'
        recipe_html += '</div></div>'

    # Power HTML
    power_html = ""
    if power_results and power_results.get("avg_cell_v") is not None:
        p = power_results
        vc = "good" if p["min_cell_v"] > 3.5 else "warn" if p["min_cell_v"] > 3.3 else "bad"
        power_html = '<div class="cc"><h3>Power & Battery</h3><table>'
        power_html += f'<tr><td>Cell voltage (avg)</td><td>{p["avg_cell_v"]:.2f}V</td></tr>'
        power_html += f'<tr><td>Cell voltage (min)</td><td class="{vc}">{p["min_cell_v"]:.2f}V</td></tr>'
        power_html += f'<tr><td>Voltage sag</td><td>{p["sag_v"]:.2f}V</td></tr>'
        if p.get("avg_current_a") is not None:
            power_html += f'<tr><td>Average current</td><td>{p["avg_current_a"]:.1f}A</td></tr>'
            power_html += f'<tr><td>Average power</td><td>{p["avg_power_w"]:.0f}W</td></tr>'
            power_html += f'<tr><td>Consumed</td><td>{p["total_mah"]:.0f}mAh ({p["total_wh"]:.1f}Wh)</td></tr>'
        power_html += '</table>'
        for f in p.get("findings", []):
            fc = "warning" if f["level"] == "WARNING" else "info"
            power_html += f'<div class="nav-findings"><div class="finding {fc}">{f["text"]}</div></div>'
        power_html += '</div>'

    # Propwash HTML
    propwash_html = ""
    if propwash_results and propwash_results.get("events"):
        pw = propwash_results
        sc = "good" if pw["score"] >= 85 else "warn" if pw["score"] >= 60 else "bad"
        propwash_html = f'<section><h2>Propwash Analysis</h2>'
        propwash_html += f'<div class="cc"><span class="{sc}">Propwash Score: {pw["score"]}/100</span>'
        propwash_html += f' — {len(pw["events"])} events, worst on {pw.get("worst_axis", "?")}'
        for f in pw.get("findings", []):
            fc = "warning" if f["level"] == "WARNING" else "info"
            propwash_html += f'<div class="nav-findings"><div class="finding {fc}">{f["text"]}</div></div>'
        propwash_html += '</div></section>'

    # Failsafe HTML
    failsafe_html = ""
    if failsafe_results and failsafe_results.get("events"):
        fs = failsafe_results
        failsafe_html = '<div class="cc" style="border-left:4px solid var(--yl)"><h3 style="color:var(--yl)">Flight Events</h3>'
        failsafe_html += f'<table><tr><th>Type</th><th>Start</th><th>Duration</th><th>Recovered</th></tr>'
        for ev in fs["events"]:
            rc = "good" if ev["recovered"] else "bad"
            failsafe_html += f'<tr><td>{ev["type"]}</td><td>{ev["start_s"]:.1f}s</td>'
            failsafe_html += f'<td>{ev["duration_s"]:.1f}s</td><td class="{rc}">{"Yes" if ev["recovered"] else "No"}</td></tr>'
        failsafe_html += '</table></div>'

    return f"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>INAV Tuning Report — {config.get('craft_name','')}</title><style>
:root{{--bg:#0f1117;--cd:#1a1b26;--ca:#1e2030;--bd:#2a2d3e;--tx:#c0caf5;--dm:#7982a9;--bl:#7aa2f7;--gn:#9ece6a;--rd:#f7768e;--yl:#e0af68;--tl:#4ecdc4;--pp:#bb9af7;--or:#ff9e64}}
*{{box-sizing:border-box;margin:0;padding:0}}body{{font-family:'SF Mono','Cascadia Code','JetBrains Mono',monospace;background:var(--bg);color:var(--tx);line-height:1.6}}
.ct{{max-width:1200px;margin:0 auto;padding:0 24px 24px}}
header{{background:linear-gradient(135deg,#1a1b26,#24283b);border-bottom:2px solid var(--bl);padding:20px 24px;position:sticky;top:0;z-index:100}}
header h1{{font-size:1.2rem;letter-spacing:3px;text-transform:uppercase;color:var(--bl);display:inline}}
.hdr-info{{color:var(--dm);font-size:.75rem;margin-top:6px}}
.hdr-score{{display:inline-block;float:right;text-align:right}}
.hdr-score .sv{{font-size:1.8rem;font-weight:700;color:var(--bl)}}
.hdr-score .sl{{font-size:.6rem;color:var(--dm);text-transform:uppercase;letter-spacing:1px}}
.hdr-sub{{display:flex;gap:20px;font-size:.75rem;color:var(--dm);margin-top:4px}}
.tabs{{display:flex;gap:0;background:var(--cd);border-bottom:2px solid var(--bd);position:sticky;top:85px;z-index:99;overflow-x:auto}}
.tab{{padding:10px 18px;cursor:pointer;color:var(--dm);font-size:.75rem;letter-spacing:1px;text-transform:uppercase;border-bottom:2px solid transparent;transition:all .2s;white-space:nowrap}}
.tab:hover{{color:var(--tx);background:var(--ca)}}.tab.active{{color:var(--bl);border-bottom-color:var(--bl);background:var(--ca)}}
.tab-panel{{display:none}}.tab-panel.active{{display:block}}
.ss{{text-align:center;padding:24px 0}}.sb{{width:300px;height:20px;background:var(--cd);border-radius:10px;margin:8px auto;overflow:hidden;border:1px solid var(--bd)}}
.sf{{height:100%;background:{sg};border-radius:10px;width:{overall:.0f}%}}
.sd{{display:flex;justify-content:center;gap:24px;margin-top:8px;font-size:.8rem;color:var(--dm)}}
.vd{{padding:12px 20px;border-radius:8px;font-weight:600;text-align:center;margin:12px auto;max-width:600px}}
.vd.dialed-in{{background:rgba(158,206,106,.12);color:var(--gn);border:1px solid rgba(158,206,106,.3)}}
.vd.nearly-there{{background:rgba(158,206,106,.08);color:var(--gn);border:1px solid rgba(158,206,106,.2)}}
.vd.getting-better{{background:rgba(224,175,104,.1);color:var(--yl);border:1px solid rgba(224,175,104,.2)}}
.vd.needs-work{{background:rgba(224,175,104,.1);color:var(--or);border:1px solid rgba(224,175,104,.2)}}
.vd.rough{{background:rgba(247,118,142,.1);color:var(--rd);border:1px solid rgba(247,118,142,.2)}}
section{{margin:24px 0}}h2{{font-size:1rem;color:var(--bl);letter-spacing:2px;text-transform:uppercase;border-bottom:1px solid var(--bd);padding-bottom:8px;margin-bottom:16px}}
h3{{font-size:.85rem;color:var(--pp);margin-bottom:8px}}.cc{{background:var(--cd);border:1px solid var(--bd);border-radius:8px;padding:16px;margin:12px 0;overflow-x:auto}}
.cc img{{width:100%;height:auto;display:block;border-radius:4px}}
.ac{{display:flex;gap:16px;background:var(--cd);border:1px solid var(--bd);border-left:4px solid var(--bd);border-radius:6px;padding:16px 20px;margin:10px 0;align-items:flex-start}}
.ac.critical{{border-left-color:var(--rd);background:rgba(247,118,142,.04)}}.ac.important{{border-left-color:var(--yl)}}.ac.normal{{border-left-color:var(--bl)}}
.ac.ok{{border-left-color:var(--gn);background:rgba(158,206,106,.06)}}.an{{font-size:1.4rem;font-weight:700;color:var(--bl);min-width:28px}}
.am{{font-weight:600;font-size:.95rem}}.ar{{color:var(--dm);font-size:.8rem;margin-top:4px}}
.ub{{font-size:.65rem;padding:1px 6px;border-radius:3px;font-weight:700;letter-spacing:1px;vertical-align:middle;margin-left:8px}}
.ub.critical{{background:rgba(247,118,142,.2);color:var(--rd)}}.ub.important{{background:rgba(224,175,104,.2);color:var(--yl)}}
table{{width:100%;border-collapse:collapse;margin:8px 0}}th{{background:var(--ca);color:var(--bl);font-size:.7rem;letter-spacing:1px;text-transform:uppercase;padding:8px 14px;text-align:left;border-bottom:1px solid var(--bd)}}
td{{padding:7px 14px;border-bottom:1px solid var(--bd);font-size:.85rem}}.good{{color:var(--gn);font-weight:600}}.warn{{color:var(--yl);font-weight:600}}.bad{{color:var(--rd);font-weight:600}}
.ax-roll{{color:#ff6b6b;font-weight:700}}.ax-pitch{{color:#4ecdc4;font-weight:700}}.ax-yaw{{color:#ffd93d;font-weight:700}}
.cg{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}.fcs{{display:flex;flex-wrap:wrap;gap:8px}}.fc{{background:var(--ca);border:1px solid var(--bd);border-radius:4px;padding:4px 10px;font-size:.8rem}}
.nav-section{{margin:24px 0}}.nav-scores{{width:100%;border-collapse:collapse}}.nav-scores td,.nav-scores th{{padding:7px 14px;border-bottom:1px solid var(--bd);font-size:.85rem}}.nav-findings{{margin:12px 0}}.nav-findings .finding{{padding:6px 12px;margin:4px 0;border-radius:4px;font-size:.85rem}}.nav-findings .warning{{background:rgba(255,215,0,0.1);border-left:3px solid var(--yl)}}.nav-findings .info{{background:rgba(100,149,237,0.1);border-left:3px solid var(--pp)}}
footer{{text-align:center;color:var(--dm);font-size:.7rem;padding:24px 0;border-top:1px solid var(--bd);margin-top:40px}}
</style></head><body>
<header>
<div style="display:flex;justify-content:space-between;align-items:center;max-width:1200px;margin:0 auto">
<div>
<h1>▲ INAV Tuning Report</h1>
<div class="hdr-info">{config.get('craft_name','')} — {fw_str}</div>
<div class="hdr-sub"><span>Duration: {data['time_s'][-1]:.1f}s</span><span>Sample rate: {data['sample_rate']:.0f}Hz</span><span>{datetime.now().strftime('%Y-%m-%d %H:%M')}</span></div>
</div>
<div class="hdr-score"><div class="sl">Score</div><div class="sv">{overall:.0f}<span style="font-size:.9rem;color:var(--dm)">/100</span></div>
<div class="hdr-sub" style="justify-content:flex-end">{'<span>Noise:'+f'{scores["noise"]:.0f}'+'</span>' if scores["noise"] is not None else ''}{'<span>PID:'+f'{scores["pid"]:.0f}'+'</span>' if scores["pid"] is not None else ''}<span>Motors:{scores['motor']:.0f}</span></div>
</div></div>
</header>

<div class="tabs" id="tabbar">
<div class="tab active" data-tab="overview">Overview</div>
<div class="tab" data-tab="pid">PID Tuning</div>
<div class="tab" data-tab="noise">Noise & Filters</div>
<div class="tab" data-tab="motors">Motors</div>
{'<div class="tab" data-tab="navperf">Nav Performance</div>' if nav_perf_html else ''}
{'<div class="tab" data-tab="navsensors">Nav Sensors</div>' if nav_sensors_html else ''}
{'<div class="tab" data-tab="map">Map</div>' if map_html else ''}
{'<div class="tab" data-tab="history">History</div>' if history_html else ''}
<div class="tab" data-tab="whatif">What-If</div>
</div>

<div class="ct">

<!-- OVERVIEW TAB -->
<div class="tab-panel active" id="tab-overview">
<div class="ss"><div class="sb"><div class="sf"></div></div>
<div class="sd">{'<span>Noise:'+f'{scores["noise"]:.0f}'+'</span>' if scores["noise"] is not None else ''}{'<span>PID:'+f'{scores["pid"]:.0f}'+'</span>' if scores["pid"] is not None else ''}<span>Motors:{scores['motor']:.0f}</span></div></div>
<div class="vd {vc}">▸ {plan['verdict_text']}</div>
{stale_html}
{pf_html}
{info_html}
{noise_summary_html}
<section><h2>Actions</h2>{ah}</section>
{cli_html}
{recipe_html}
{meas_html}
{power_html}
{failsafe_html}
{ch}
</div>

<!-- PID TUNING TAB -->
<div class="tab-panel" id="tab-pid">
<section><h2>PID Response</h2>
<div class="cc"><table><tr><th>Axis</th><th>Delay</th><th>Overshoot</th><th>RMS Error</th></tr>{pt}</table></div>
<div class="cc">{ci("pid")}</div>
</section>
</div>

<!-- NOISE & FILTERS TAB -->
<div class="tab-panel" id="tab-noise">
<section><h2>Noise Spectrum</h2><div class="cc">{ci("noise")}</div></section>
{'<section><h2>D-Term Noise</h2><div class="cc">'+ci("dterm")+'</div></section>' if charts.get("dterm") else ''}
{_generate_vibration_html(accel_vib) if accel_vib and accel_vib.get("axes") else ''}
{propwash_html}
</div>

<!-- MOTORS TAB -->
<div class="tab-panel" id="tab-motors">
{'<section><h2>Motor Balance</h2><div class="cc"><table><tr><th>Motor</th><th>Avg</th><th>StdDev</th><th>Saturation</th></tr>'+mt+'</table></div><div class="cc">'+ci("motor")+'</div></section>' if motor_analysis else '<section><h2>Motors</h2><div class="cc" style="color:var(--dm)">No motor data available.</div></section>'}
</div>

<!-- NAV PERFORMANCE TAB -->
{'<div class="tab-panel" id="tab-navperf">' + nav_perf_html + '</div>' if nav_perf_html else ''}

<!-- NAV SENSORS TAB -->
{'<div class="tab-panel" id="tab-navsensors">' + nav_sensors_html + '</div>' if nav_sensors_html else ''}

<!-- MAP TAB -->
{'<div class="tab-panel" id="tab-map">' + map_html + '</div>' if map_html else ''}

<!-- HISTORY TAB -->
{'<div class="tab-panel" id="tab-history">' + history_html + '</div>' if history_html else ''}

<!-- WHAT-IF TAB -->
<div class="tab-panel" id="tab-whatif">
{whatif_html}
</div>

</div>
<footer>INAV Blackbox Analyzer v{REPORT_VERSION} — Test changes with props off first</footer>

<script>
document.querySelectorAll('.tab').forEach(tab => {{
  tab.addEventListener('click', () => {{
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
    // Leaflet needs resize trigger when map tab becomes visible
    if (tab.dataset.tab === 'map' && window._flightMap) {{
      setTimeout(() => window._flightMap.invalidateSize(), 100);
    }}
  }});
}});
</script>
</body></html>"""


# ─── Markdown Report ─────────────────────────────────────────────────────────

def generate_markdown_report(plan, config, data, noise_results, pid_results,
                             motor_analysis, profile, quality=None):
    """Generate a Markdown report suitable for pasting into forums/Discord.

    Returns the markdown string.
    """
    scores = plan["scores"]
    craft = config.get("craft_name", "Unknown")
    fw = config.get("firmware_revision", "?")
    sr = data.get("sample_rate", 0)
    duration = data["time_s"][-1] if "time_s" in data and len(data.get("time_s", [])) > 0 else 0

    lines = []
    lines.append(f"## {t('banner.analyzer')} — {craft}")
    lines.append(f"")
    lines.append(f"**{t('banner.firmware')}:** {fw}  ")
    lines.append(f"**{t('section.duration')}:** {duration:.1f}s @ {sr:.0f}Hz  ")
    lines.append(f"**{t('banner.profile')}:** {profile['name']}  ")
    lines.append(f"**{t('label.date')}:** {datetime.now().strftime('%Y-%m-%d %H:%M')}  ")
    lines.append(f"")

    # Log quality (if available)
    if quality:
        grade = quality["grade"]
        grade_text = t(f"quality.{grade.lower()}")
        emoji = "✅" if grade == "GOOD" else "⚠️" if grade == "MARGINAL" else "❌"
        lines.append(f"**{t('quality.title')}:** {emoji} {grade_text}")
        for issue in quality.get("issues", []):
            sym = "❌" if issue["severity"] == "FAIL" else "⚠️"
            lines.append(f"- {sym} {issue['message']}")
        lines.append(f"")

    # Scores
    lines.append(f"### {t('section.scores')}")
    lines.append(f"")
    lines.append(f"| {t('compare.metric')} | {t('label.score')} |")
    lines.append(f"|--------|-------|")
    lines.append(f"| **{t('section.overall')}** | **{scores.get('overall', 0):.0f}/100** |")
    for key, tkey in [("noise", "section.noise"), ("pid", "section.pid"),
                       ("motor", "section.motor"), ("osc", "section.oscillation")]:
        val = scores.get(key)
        if val is not None:
            lines.append(f"| {t(tkey)} | {val:.0f}/100 |")
    lines.append(f"")

    # Verdict
    verdict = plan.get("verdict_text", plan.get("verdict", "?"))
    lines.append(f"**{t('section.verdict')}:** {verdict}")
    lines.append(f"")

    # Findings
    findings = plan.get("findings", [])
    if findings:
        lines.append(f"### {t('section.findings')}")
        lines.append(f"")
        for f in findings:
            sev = f.get("severity", "info")
            emoji = "🔴" if sev == "critical" else "🟡" if sev == "warning" else "🟢"
            lines.append(f"- {emoji} {f.get('message', f.get('finding', ''))}")
        lines.append(f"")

    # Noise fingerprint
    noise_fp = plan.get("noise_fingerprint", {})
    fp_peaks = noise_fp.get("peaks", []) if isinstance(noise_fp, dict) else []
    if fp_peaks:
        lines.append(f"### Noise Sources")
        lines.append(f"")
        if isinstance(noise_fp, dict) and noise_fp.get("summary"):
            lines.append(f"**{noise_fp['summary']}**")
            lines.append(f"")
        for fp in fp_peaks[:8]:
            source = fp.get("source", "unknown")
            conf = fp.get("confidence", "?")
            freq = fp.get("freq_hz", 0)
            power = fp.get("power_db", 0)
            detail = fp.get("detail", "")
            lines.append(f"- **{source}** ({conf} confidence) — {freq:.0f}Hz, {power:.1f}dB"
                        f"{f' ({detail})' if detail else ''}")
        lines.append(f"")

    # PID response summary
    pid_summary = []
    for i, ax in enumerate(AXIS_NAMES):
        pr = pid_results[i] if i < len(pid_results) else None
        if pr and pr.get("tracking_delay_ms") is not None:
            pid_summary.append(f"- **{ax}**: delay {pr['tracking_delay_ms']:.1f}ms, "
                              f"overshoot {pr.get('avg_overshoot_pct', 0):.1f}%")
    if pid_summary:
        lines.append(f"### PID Response")
        lines.append(f"")
        lines.extend(pid_summary)
        lines.append(f"")

    # Recommended actions (the CLI commands)
    actions = [a for a in plan.get("actions", []) if not a.get("deferred")]
    if actions:
        lines.append(f"### {t('section.recommended')}")
        lines.append(f"")
        lines.append(f"```")
        for a in actions:
            param = a.get("param", "")
            new_val = a.get("new", "")
            if param and new_val != "":
                lines.append(f"set {param} = {new_val}")
        lines.append(f"save")
        lines.append(f"```")
        lines.append(f"")

        # Show what changed
        lines.append(f"| {t('label.setting')} | {t('label.current')} | {t('label.recommended')} | {t('label.reason')} |")
        lines.append(f"|---------|---------|-------------|--------|")
        for a in actions:
            param = a.get("param", "")
            if param:
                reason = a.get("reason", a.get("action", ""))[:60]
                lines.append(f"| `{param}` | {a.get('current', '?')} | {a.get('new', '?')} | {reason} |")
        lines.append(f"")

    deferred = [a for a in plan.get("actions", []) if a.get("deferred")]
    if deferred:
        lines.append(f"### {t('section.deferred')}")
        lines.append(f"")
        for a in deferred:
            lines.append(f"- `set {a.get('param', '?')} = {a.get('new', '?')}` — {a.get('action', '')}")
        lines.append(f"")

    lines.append(f"---")
    lines.append(f"*{t('report.generated_by')} [INAV Toolkit v{REPORT_VERSION}]"
                f"(https://github.com/agoliveira/INAV-Toolkit)*")

    return "\n".join(lines)


# ─── State file ───────────────────────────────────────────────────────────────

def save_state(filepath, plan, config, data):
    active = [a for a in plan["actions"] if not a.get("deferred")]
    deferred = [a for a in plan["actions"] if a.get("deferred")]
    state = {"version": REPORT_VERSION, "timestamp": datetime.now().isoformat(),
             "scores": plan["scores"], "verdict": plan["verdict"],
             "actions": [{"action": a["action"], "param": a.get("param", ""),
                           "current": str(a.get("current", "")), "new": str(a.get("new", ""))} for a in active],
             "deferred_actions": [{"action": a.get("original_action", a["action"]),
                                    "param": a.get("param", ""),
                                    "current": str(a.get("current", "")), "new": str(a.get("new", "")),
                                    "reason": "Fix filters first, then re-fly"} for a in deferred],
             "config": {k: v for k, v in config.items() if k != "raw" and not isinstance(v, np.ndarray)}}
    sp = filepath.rsplit(".", 1)[0] + "_state.json"
    with open(sp, "w") as f:
        json.dump(state, f, indent=2, default=str)
    return sp


# ─── Log Splitter ─────────────────────────────────────────────────────────────

def split_blackbox_logs(filepath, output_dir=None):
    """Split a multi-log blackbox file into individual log segments.

    Blackbox files from dataflash dumps contain multiple flights concatenated,
    each starting with 'H Product:Blackbox' header lines.

    Args:
        filepath: Path to the .bbl/.TXT file
        output_dir: Directory to write split files (default: same as source,
                    or /tmp if source is read-only)

    Returns:
        List of file paths (original if single log, split files if multiple).
        Split files are saved as {basename}_log1.bbl, _log2.bbl, etc.
    """
    with open(filepath, "rb") as f:
        raw = f.read()

    # Find all log boundaries
    marker = b"H Product:Blackbox"
    positions = []
    start = 0
    while True:
        idx = raw.find(marker, start)
        if idx < 0:
            break
        positions.append(idx)
        start = idx + len(marker)

    if len(positions) <= 1:
        # Single log - return as-is
        return [filepath]

    # Determine output directory
    if output_dir is None:
        output_dir = os.path.dirname(filepath) or "."
    # Ensure directory exists, test writability
    try:
        os.makedirs(output_dir, exist_ok=True)
        test_file = os.path.join(output_dir, ".split_test")
        with open(test_file, "w") as f:
            f.write("")
        os.remove(test_file)
    except OSError:
        output_dir = "/tmp"

    basename = os.path.splitext(os.path.basename(filepath))[0]
    ext = os.path.splitext(filepath)[1] or ".bbl"

    # Split into separate files
    split_files = []
    for i, pos in enumerate(positions):
        end = positions[i + 1] if i + 1 < len(positions) else len(raw)
        segment = raw[pos:end]

        # Skip tiny segments (< 1KB) - likely just headers without data
        if len(segment) < 1024:
            continue

        split_path = os.path.join(output_dir, f"{basename}_log{i + 1}{ext}")
        with open(split_path, "wb") as f:
            f.write(segment)
        split_files.append(split_path)

    return split_files


def count_blackbox_logs(filepath):
    """Count how many separate logs are in a blackbox file.

    Returns count without splitting the file.
    """
    with open(filepath, "rb") as f:
        raw = f.read()

    marker = b"H Product:Blackbox"
    count = 0
    start = 0
    while True:
        idx = raw.find(marker, start)
        if idx < 0:
            break
        count += 1
        start = idx + len(marker)

    return max(1, count)


# ─── Blackbox Readiness Check ─────────────────────────────────────────────────

# Required fields per analysis mode
_FIELDS_PID_TUNING = {
    "required": ["gyroADC[0]", "axisP[0]", "motor[0]", "rcCommand[0]"],
    "recommended": ["gyroRaw[0]", "axisD[0]", "axisF[0]", "axisI[0]"],
}
_FIELDS_NAV_ANALYSIS = {
    "required": ["navPos[0]", "navTgtPos[0]", "navVel[0]", "mcVelAxisP[0]"],
    "recommended": ["mcPosAxisP[0]", "mcVelAxisOut[0]", "navState", "navFlags",
                     "mcVelAxisI[0]", "mcVelAxisD[0]", "mcVelAxisFF[0]",
                     "navTgtVel[0]", "navTgtHdg", "navEPH"],
}
_FIELDS_NAV_GPS = {
    "required": ["GPS_coord[0]", "GPS_fixType", "GPS_numSat"],
    "recommended": ["GPS_speed", "GPS_hdop", "GPS_velned[0]"],
}
_FIELDS_NAV_SLOW = {
    "required": ["flightModeFlags"],
    "recommended": ["wind[0]", "activeFlightModeFlags"],
}


def check_blackbox_readiness_from_dump(config_text):
    """Check blackbox configuration from FC dump/diff output (pre-download).

    Returns dict with:
        ok (bool): True if blackbox is enabled and configured
        items (list of dicts): [{level, text, fix}] — INFO/WARNING/CRITICAL items
        has_gps (bool): GPS feature enabled
        has_blackbox (bool): Blackbox feature enabled
        bb_device (str): Blackbox device type
        bb_rate (str): Logging rate fraction
    """
    items = []
    has_blackbox = False
    has_gps = False
    bb_device = "unknown"
    bb_rate = "1/1"

    if not config_text:
        return {"ok": False, "items": [{"level": "CRITICAL",
                "text": "No config available", "fix": None}],
                "has_gps": False, "has_blackbox": False,
                "bb_device": "unknown", "bb_rate": "unknown"}

    # Parse features
    features = []
    settings = {}
    for line in config_text.splitlines():
        line = line.strip()
        if line.startswith("feature ") and not line.startswith("feature -"):
            features.append(line[8:].strip())
        elif line.startswith("set "):
            m = re.match(r"set\s+(\S+)\s*=\s*(.*)", line)
            if m:
                settings[m.group(1).strip()] = m.group(2).strip()

    # Check blackbox feature
    has_blackbox = "BLACKBOX" in features
    if not has_blackbox:
        items.append({"level": "CRITICAL",
            "text": "Blackbox feature not enabled — nothing is being logged",
            "fix": "feature BLACKBOX"})

    # Check blackbox device
    bb_dev = settings.get("blackbox_device", "").upper()
    if bb_dev:
        bb_device = bb_dev
    if has_blackbox and bb_dev in ("NONE", ""):
        items.append({"level": "CRITICAL",
            "text": "Blackbox device is NONE — no storage target configured",
            "fix": "set blackbox_device = SPIFLASH"})

    # Check logging rate
    rate_num = settings.get("blackbox_rate_num", "1")
    rate_den = settings.get("blackbox_rate_denom", "1")
    bb_rate = f"{rate_num}/{rate_den}"
    try:
        if int(rate_den) > 1:
            items.append({"level": "INFO",
                "text": f"Logging at {bb_rate} rate — full rate (1/1) gives best PID analysis data",
                "fix": "set blackbox_rate_num = 1\n    set blackbox_rate_denom = 1"})
    except ValueError:
        pass

    # Check GPS
    has_gps = "GPS" in features
    if not has_gps:
        items.append({"level": "INFO",
            "text": "GPS not enabled — nav analysis (--nav) won't have position data",
            "fix": "set feature GPS"})

    ok = has_blackbox and bb_dev not in ("NONE", "")
    return {"ok": ok, "items": items, "has_gps": has_gps,
            "has_blackbox": has_blackbox, "bb_device": bb_device, "bb_rate": bb_rate}


def preflight_checklist(config_text, frame_inches=None):
    """Run pre-flight safety checklist on FC dump config.

    Checks for dangerous configuration issues that should be fixed
    before flying. Warns sternly but doesn't block.

    Args:
        config_text: Raw dump/diff text from FC
        frame_inches: Frame size for context-aware checks

    Returns dict with:
        items (list of dicts): [{level, text, fix}] — CRITICAL/WARNING/INFO
        n_critical (int): Number of critical issues
        n_warning (int): Number of warnings
    """
    items = []

    if not config_text:
        return {"items": [], "n_critical": 0, "n_warning": 0}

    # Parse features and settings
    features = []
    features_disabled = []
    beepers_disabled = []
    settings = {}
    for line in config_text.splitlines():
        line = line.strip()
        if line.startswith("feature -"):
            features_disabled.append(line[9:].strip())
        elif line.startswith("feature "):
            features.append(line[8:].strip())
        elif line.startswith("beeper -"):
            beepers_disabled.append(line[8:].strip())
        elif line.startswith("set "):
            m = re.match(r"set\s+(\S+)\s*=\s*(.*)", line)
            if m:
                settings[m.group(1).strip()] = m.group(2).strip()

    # ═══ CRITICAL: Safety beepers ═══
    critical_beepers = ["BAT_CRIT_LOW", "BAT_LOW", "RX_LOST", "RX_LOST_LANDING", "HW_FAILURE"]
    missing_beepers = [b for b in critical_beepers if b in beepers_disabled]
    if missing_beepers:
        items.append({
            "level": "CRITICAL",
            "text": f"Safety beepers DISABLED: {', '.join(missing_beepers)}. "
                    f"No audible warning for low battery, signal loss, or hardware failure.",
            "fix": " && ".join(f"beeper {b}" for b in missing_beepers),
        })

    # ═══ CRITICAL: Failsafe procedure ═══
    failsafe = settings.get("failsafe_procedure", "").upper()
    if failsafe == "DROP":
        items.append({
            "level": "CRITICAL",
            "text": "Failsafe set to DROP — quad will fall out of the sky on signal loss. "
                    "Use SET-THR, RTH, or LAND instead.",
            "fix": "set failsafe_procedure = RTH",
        })

    # ═══ CRITICAL: No failsafe throttle with SET-THR ═══
    if failsafe == "SET-THR":
        fs_thr = settings.get("failsafe_throttle", "1000")
        try:
            if int(fs_thr) <= 1000:
                items.append({
                    "level": "CRITICAL",
                    "text": f"Failsafe procedure is SET-THR but throttle is {fs_thr}µs — "
                            f"this will not maintain flight. Set to hover throttle (~1400-1500).",
                    "fix": "set failsafe_throttle = 1450",
                })
        except ValueError:
            pass

    # ═══ WARNING: Battery voltage limits ═══
    min_cell = settings.get("vbat_min_cell_voltage", "")
    warn_cell = settings.get("vbat_warning_cell_voltage", "")
    try:
        if min_cell and float(min_cell) / 100 < 3.0:
            items.append({
                "level": "WARNING",
                "text": f"Battery min cell voltage very low ({float(min_cell)/100:.2f}V). "
                        f"LiPo should not go below 3.3V, Li-ion not below 2.8V.",
                "fix": "set vbat_min_cell_voltage = 330",
            })
    except ValueError:
        pass

    # ═══ WARNING: Motor protocol for frame size ═══
    motor_proto = settings.get("motor_pwm_protocol", "").upper()
    if motor_proto in ("STANDARD", "ONESHOT125", "ONESHOT42", "MULTISHOT"):
        items.append({
            "level": "WARNING",
            "text": f"Motor protocol is {motor_proto} — consider DSHOT300 or DSHOT600 "
                    f"for better reliability and ESC telemetry support.",
            "fix": "set motor_pwm_protocol = DSHOT300",
        })

    # ═══ WARNING: Arming without GPS fix ═══
    nav_extra_arming = settings.get("nav_extra_arming_safety", "").upper()
    if nav_extra_arming in ("OFF", "ALLOW_BYPASS"):
        has_gps = "GPS" in features
        if has_gps:
            items.append({
                "level": "WARNING",
                "text": "GPS arming safety disabled — quad can arm without GPS fix. "
                        "RTH and position hold won't work without a fix.",
                "fix": "set nav_extra_arming_safety = ON",
            })

    # ═══ WARNING: RTH altitude ═══
    rth_alt = settings.get("nav_rth_altitude", "")
    try:
        if rth_alt and int(rth_alt) < 1500:
            items.append({
                "level": "WARNING",
                "text": f"RTH altitude is {int(rth_alt)/100:.0f}m — very low. "
                        f"Risk of hitting obstacles during return. 30-50m recommended.",
                "fix": "set nav_rth_altitude = 3000",
            })
    except ValueError:
        pass

    # ═══ WARNING: D-term LPF too high for frame ═══
    if frame_inches and frame_inches >= 8:
        dterm_lpf = settings.get("dterm_lpf_hz", "")
        try:
            if dterm_lpf and int(dterm_lpf) > 80:
                items.append({
                    "level": "WARNING",
                    "text": f"D-term LPF at {dterm_lpf}Hz — high for {frame_inches}-inch. "
                            f"45-75Hz typical. High values let motor noise through → hot motors.",
                    "fix": f"set dterm_lpf_hz = {min(75, int(dterm_lpf))}",
                })
        except ValueError:
            pass

    # ═══ INFO: Blackbox logging suggestions ═══
    bb_rate_denom = settings.get("blackbox_rate_denom", "1")
    try:
        if int(bb_rate_denom) > 2:
            items.append({
                "level": "INFO",
                "text": f"Blackbox logging at 1/{bb_rate_denom} rate. "
                        f"Full rate (1/1) gives best analysis data.",
                "fix": "set blackbox_rate_denom = 1",
            })
    except ValueError:
        pass

    n_critical = sum(1 for i in items if i["level"] == "CRITICAL")
    n_warning = sum(1 for i in items if i["level"] == "WARNING")
    return {"items": items, "n_critical": n_critical, "n_warning": n_warning}


def _print_preflight(checklist):
    """Print pre-flight checklist results to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()

    if not checklist["items"]:
        print(f"  {G}✓ Pre-flight checklist: all good{R}")
        return

    n_crit = checklist["n_critical"]
    n_warn = checklist["n_warning"]

    if n_crit:
        print(f"\n  {RED}{B}{'━'*66}{R}")
        print(f"  {RED}{B}  ⚠  PRE-FLIGHT SAFETY CHECK — {n_crit} CRITICAL ISSUE{'S' if n_crit != 1 else ''}{R}")
        print(f"  {RED}{B}{'━'*66}{R}")
    elif n_warn:
        print(f"\n  {Y}{'─'*66}{R}")
        print(f"  {Y}  Pre-flight check: {n_warn} warning{'s' if n_warn != 1 else ''}{R}")
        print(f"  {Y}{'─'*66}{R}")

    icons = {"CRITICAL": f"{RED}✗", "WARNING": f"{Y}⚠", "INFO": f"{DIM}ℹ"}

    for item in checklist["items"]:
        level = item["level"]
        icon = icons.get(level, "?")
        print(f"\n  {icon} {B}{item['text']}{R}")
        if item.get("fix"):
            fix_lines = item["fix"].split(" && ")
            for fl in fix_lines:
                print(f"    {G}Fix: {fl}{R}")

    if n_crit:
        print(f"\n  {RED}{B}  FIX THESE BEFORE FLYING!{R}")
        print(f"  {RED}{B}{'━'*66}{R}")
    print()


def check_blackbox_readiness_from_headers(raw_params, nav_mode=False):
    """Check field availability from BBL headers (pre-analysis).

    Args:
        raw_params: dict from parse_headers_from_bbl()
        nav_mode: True if --nav was requested

    Returns dict with:
        ok (bool): True if minimum fields present for the requested mode
        items (list of dicts): [{level, text}]
        has_pid_fields (bool): PID tuning fields present
        has_nav_fields (bool): Nav analysis fields present
        has_gps_fields (bool): GPS G-frame fields present
        missing_pid (list): Missing required PID fields
        missing_nav (list): Missing required nav fields
    """
    items = []

    # Parse I-frame fields
    i_fields_str = raw_params.get("Field I name", "")
    i_fields = [f.strip() for f in i_fields_str.split(",")] if i_fields_str else []
    i_fields_lower = [f.lower() for f in i_fields]

    # Parse G-frame fields
    g_fields_str = raw_params.get("Field G name", "")
    g_fields = [f.strip() for f in g_fields_str.split(",")] if g_fields_str else []

    # Parse S-frame fields
    s_fields_str = raw_params.get("Field S name", "")
    s_fields = [f.strip() for f in s_fields_str.split(",")] if s_fields_str else []

    def _has_field(name, field_list):
        """Check if field exists (case-insensitive)."""
        name_l = name.lower()
        return any(name_l == f.lower() for f in field_list)

    # Check PID tuning fields
    missing_pid = [f for f in _FIELDS_PID_TUNING["required"] if not _has_field(f, i_fields)]
    missing_pid_rec = [f for f in _FIELDS_PID_TUNING["recommended"] if not _has_field(f, i_fields)]
    has_pid_fields = len(missing_pid) == 0

    if missing_pid:
        items.append({"level": "WARNING",
            "text": f"PID tuning fields missing: {', '.join(missing_pid)}. "
                    f"PID analysis will be limited."})

    # Check nav fields
    missing_nav = [f for f in _FIELDS_NAV_ANALYSIS["required"] if not _has_field(f, i_fields)]
    missing_nav_gps = [f for f in _FIELDS_NAV_GPS["required"] if not _has_field(f, g_fields)]
    missing_nav_slow = [f for f in _FIELDS_NAV_SLOW["required"] if not _has_field(f, s_fields)]
    has_nav_fields = len(missing_nav) == 0
    has_gps_fields = len(missing_nav_gps) == 0

    if nav_mode:
        if missing_nav:
            items.append({"level": "WARNING",
                "text": f"Nav analysis fields missing from I-frames: {', '.join(missing_nav)}. "
                        f"Enable GPS and fly with nav modes for full nav data."})
        if missing_nav_gps:
            items.append({"level": "WARNING",
                "text": f"GPS fields missing from G-frames: {', '.join(missing_nav_gps)}. "
                        f"GPS may not have been enabled during this flight."})
        if missing_nav_slow:
            items.append({"level": "INFO",
                "text": f"Flight mode fields missing from S-frames: {', '.join(missing_nav_slow)}. "
                        f"Mode transition analysis won't work."})
        if not missing_nav and not missing_nav_gps:
            items.append({"level": "OK",
                "text": "All nav analysis fields present"})
    else:
        # Not in nav mode — just note availability
        if has_nav_fields and has_gps_fields:
            items.append({"level": "OK",
                "text": "Nav fields present — nav analysis will run automatically"})

    ok = has_pid_fields if not nav_mode else (has_pid_fields and has_nav_fields)
    return {"ok": ok, "items": items,
            "has_pid_fields": has_pid_fields, "has_nav_fields": has_nav_fields,
            "has_gps_fields": has_gps_fields,
            "missing_pid": missing_pid, "missing_nav": missing_nav}


def _print_readiness(check_result, source="config"):
    """Print readiness check results to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    icons = {"CRITICAL": f"{RED}✗", "WARNING": f"{Y}⚠", "INFO": f"{DIM}ℹ", "OK": f"{G}✓"}

    for item in check_result.get("items", []):
        level = item["level"]
        icon = icons.get(level, "?")
        print(f"  {icon} {item['text']}{R}")
        if item.get("fix"):
            for fix_line in item["fix"].split("\n"):
                print(f"  {DIM}  Fix: {fix_line.strip()}{R}")


# ─── Config Backup Vault ──────────────────────────────────────────────────────

def _vault_dir(blackbox_dir="./blackbox"):
    """Get the config vault directory path."""
    return os.path.join(blackbox_dir, "config_vault")


def vault_archive(config_raw, craft_name, blackbox_dir="./blackbox"):
    """Archive a config dump with timestamp.

    Saves to blackbox/config_vault/{craft}_{timestamp}.txt
    Returns the archive path.
    """
    vault = _vault_dir(blackbox_dir)
    os.makedirs(vault, exist_ok=True)

    craft = (craft_name or "fc").replace(" ", "_")
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{craft}_{ts}.txt"
    path = os.path.join(vault, filename)

    with open(path, "w") as f:
        f.write(config_raw)

    return path


def vault_list(craft_name=None, blackbox_dir="./blackbox", limit=20):
    """List archived configs, newest first.

    Returns list of dicts: [{path, craft, timestamp, n_settings, age_str}]
    """
    vault = _vault_dir(blackbox_dir)
    if not os.path.isdir(vault):
        return []

    entries = []
    for fname in sorted(os.listdir(vault), reverse=True):
        if not fname.endswith(".txt"):
            continue

        # Parse filename: CRAFT_NAME_YYYYMMDD_HHMMSS.txt
        parts = fname[:-4].rsplit("_", 2)
        if len(parts) < 3:
            continue

        try:
            ts_str = f"{parts[-2]}_{parts[-1]}"
            ts = datetime.strptime(ts_str, "%Y%m%d_%H%M%S")
            craft = "_".join(parts[:-2])
        except ValueError:
            continue

        if craft_name and craft.replace("_", " ").lower() != craft_name.lower():
            continue

        # Count settings
        fpath = os.path.join(vault, fname)
        try:
            with open(fpath, "r") as f:
                text = f.read()
            n_settings = len([l for l in text.splitlines() if l.strip().startswith("set ")])
        except Exception:
            n_settings = 0

        # Age string
        age = datetime.now() - ts
        if age.days > 0:
            age_str = f"{age.days}d ago"
        elif age.seconds > 3600:
            age_str = f"{age.seconds // 3600}h ago"
        else:
            age_str = f"{age.seconds // 60}m ago"

        entries.append({
            "path": fpath,
            "filename": fname,
            "craft": craft.replace("_", " "),
            "timestamp": ts,
            "ts_str": ts.strftime("%Y-%m-%d %H:%M"),
            "n_settings": n_settings,
            "age_str": age_str,
        })

        if len(entries) >= limit:
            break

    return entries


def vault_diff(path_a, path_b):
    """Diff two config files from the vault.

    Returns dict with:
        added (list): settings only in B
        removed (list): settings only in A
        changed (list): [{param, old, new}]
        unchanged (int): count of identical settings
    """
    def parse_settings(path):
        settings = {}
        try:
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("set ") and " = " in line:
                        parts = line[4:].split(" = ", 1)
                        if len(parts) == 2:
                            settings[parts[0].strip()] = parts[1].strip()
        except Exception:
            pass
        return settings

    a = parse_settings(path_a)
    b = parse_settings(path_b)

    all_params = sorted(set(list(a.keys()) + list(b.keys())))
    added = []
    removed = []
    changed = []
    unchanged = 0

    for param in all_params:
        va = a.get(param)
        vb = b.get(param)
        if va is None and vb is not None:
            added.append({"param": param, "value": vb})
        elif va is not None and vb is None:
            removed.append({"param": param, "value": va})
        elif va != vb:
            changed.append({"param": param, "old": va, "new": vb})
        else:
            unchanged += 1

    return {
        "added": added,
        "removed": removed,
        "changed": changed,
        "unchanged": unchanged,
    }


def _print_vault_list(entries):
    """Print vault listing to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    if not entries:
        print(f"\n  {DIM}No configs in vault. Configs are archived automatically "
              f"when using --device auto.{R}")
        return

    print(f"\n  {B}CONFIG VAULT:{R} {len(entries)} archived config{'s' if len(entries) != 1 else ''}")
    print(f"  {B}{'#':>3}  {'Date':16s}  {'Settings':>8}  {'Age':>8}  Craft{R}")
    print(f"  {DIM}{'─'*60}{R}")
    for i, e in enumerate(entries, 1):
        print(f"  {DIM}{i:>3}{R}  {e['ts_str']:16s}  {e['n_settings']:>8}  {e['age_str']:>8}  {e['craft']}")
    print(f"\n  {DIM}Path: {os.path.dirname(entries[0]['path'])}{R}")
    print(f"  {DIM}Use --config-diff N M to compare config #N with #M{R}")


def _print_vault_diff(diff, path_a, path_b):
    """Print vault diff to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()

    n_changes = len(diff["changed"]) + len(diff["added"]) + len(diff["removed"])
    if n_changes == 0:
        print(f"\n  {G}✓ Configs are identical ({diff['unchanged']} settings).{R}")
        return

    print(f"\n  {B}CONFIG DIFF:{R} {n_changes} difference{'s' if n_changes != 1 else ''}, "
          f"{diff['unchanged']} unchanged")
    print(f"  {DIM}A: {os.path.basename(path_a)}{R}")
    print(f"  {DIM}B: {os.path.basename(path_b)}{R}")

    if diff["changed"]:
        print(f"\n  {B}Changed ({len(diff['changed'])}):{R}")
        for ch in diff["changed"]:
            try:
                old_f = float(ch["old"])
                new_f = float(ch["new"])
                pct = ((new_f - old_f) / old_f * 100) if old_f != 0 else 0
                pct_str = f"  {DIM}({pct:+.0f}%){R}" if abs(pct) > 1 else ""
            except (ValueError, ZeroDivisionError):
                pct_str = ""
            print(f"    {Y}{ch['param']}{R}: {ch['old']} → {ch['new']}{pct_str}")

    if diff["added"]:
        print(f"\n  {B}Added in B ({len(diff['added'])}):{R}")
        for a in diff["added"][:20]:
            print(f"    {G}+ {a['param']}{R} = {a['value']}")
        if len(diff["added"]) > 20:
            print(f"    {DIM}... and {len(diff['added']) - 20} more{R}")

    if diff["removed"]:
        print(f"\n  {B}Removed from B ({len(diff['removed'])}):{R}")
        for r in diff["removed"][:20]:
            print(f"    {RED}- {r['param']}{R} = {r['value']}")
        if len(diff["removed"]) > 20:
            print(f"    {DIM}... and {len(diff['removed']) - 20} more{R}")


# ─── Post-Analysis Cleanup ────────────────────────────────────────────────────

def _post_analysis_cleanup(blackbox_dir, raw_download, split_files, analyzed_file,
                           archive=False, keep_logs=False):
    """Clean up blackbox directory after successful analysis.

    Default: delete raw download and all split files.
    --archive: compress analyzed flight to archive/ first.
    --keep-logs: skip everything.

    Also cleans stale .bbl files from previous sessions.
    """
    if keep_logs:
        return

    R, B, C, G, Y, RED, DIM = _colors()
    deleted = 0
    archived = 0

    # Archive the analyzed flight if requested
    if archive and analyzed_file and os.path.isfile(analyzed_file):
        import gzip
        archive_dir = os.path.join(blackbox_dir, "archive")
        os.makedirs(archive_dir, exist_ok=True)
        gz_name = os.path.basename(analyzed_file) + ".gz"
        gz_path = os.path.join(archive_dir, gz_name)
        try:
            with open(analyzed_file, "rb") as f_in:
                with gzip.open(gz_path, "wb") as f_out:
                    f_out.write(f_in.read())
            archived_size = os.path.getsize(gz_path)
            original_size = os.path.getsize(analyzed_file)
            ratio = archived_size / original_size * 100 if original_size > 0 else 0
            print(f"  Archived: {gz_name} ({archived_size // 1024}KB, {ratio:.0f}% of original)")
            archived = 1
        except Exception as e:
            print(f"  Warning: archive failed: {e}")

    # Delete the raw download file
    if raw_download and os.path.isfile(raw_download):
        try:
            sz = os.path.getsize(raw_download) // 1024
            os.remove(raw_download)
            deleted += 1
        except Exception:
            pass

    # Delete all split files
    if split_files:
        for sf in split_files:
            if sf and os.path.isfile(sf):
                try:
                    os.remove(sf)
                    deleted += 1
                except Exception:
                    pass

    # Clean stale .bbl files from previous sessions
    # (anything not from the current download)
    current_base = os.path.basename(raw_download) if raw_download else ""
    if os.path.isdir(blackbox_dir):
        for fname in os.listdir(blackbox_dir):
            if not fname.endswith(".bbl"):
                continue
            fpath = os.path.join(blackbox_dir, fname)
            if fpath == raw_download:
                continue  # already handled
            if any(fpath == sf for sf in (split_files or [])):
                continue  # already handled
            # This is a stale .bbl from a previous session
            try:
                os.remove(fpath)
                deleted += 1
            except Exception:
                pass

    if deleted > 0 or archived > 0:
        parts = []
        if deleted > 0:
            parts.append(f"{deleted} log files removed")
        if archived > 0:
            parts.append(f"1 archived")
        print(f"  Cleanup: {', '.join(parts)}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="INAV Blackbox Analyzer v2.4.1 - Prescriptive Tuning",
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=f"inav-analyze {REPORT_VERSION}")
    parser.add_argument("logfile", nargs="?", default=None,
                        help="Blackbox log (.bbl/.bfl/.bbs/.txt or decoded .csv). "
                             "Optional when --device is used.")
    parser.add_argument("-o", "--output", help="HTML report filename")
    parser.add_argument("--no-html", action="store_true")
    parser.add_argument("--no-browser", action="store_true",
                        help="Don't open the HTML report in the default browser.")
    parser.add_argument("--no-terminal", action="store_true")
    parser.add_argument("--previous", help="Previous state .json for comparison")
    # Device communication
    parser.add_argument("--device", metavar="PORT",
                        help="Download blackbox from FC via serial. Use 'auto' to scan "
                             "or specify port (e.g., /dev/ttyACM0, COM3).")
    parser.add_argument("--no-erase", action="store_true",
                        help="Don't erase FC flash after successful download and analysis. "
                             "Default: flash is erased automatically after pipeline completes.")
    parser.add_argument("--archive", action="store_true",
                        help="Compress the analyzed flight log to blackbox/archive/ instead of deleting. "
                             "Builds a compressed history for re-analysis with future versions.")
    parser.add_argument("--keep-logs", action="store_true",
                        help="Skip all cleanup - keep raw downloads, splits, and don't erase flash.")
    parser.add_argument("--download-only", action="store_true",
                        help="Download blackbox from device but don't analyze.")
    parser.add_argument("--blackbox-dir", default="./blackbox",
                        help="Directory to save downloaded logs (default: ./blackbox).")
    # Frame and prop profile arguments
    parser.add_argument("--frame", type=int, metavar="INCHES",
                        help="Frame size in inches (3-15). Determines PID response thresholds. "
                             "Default: same as --props, or 5 if neither specified.")
    parser.add_argument("--props", type=int, metavar="INCHES",
                        help="Prop diameter in inches (3-15). Determines filter ranges and "
                             "noise frequency predictions. Default: same as --frame, or 5.")
    parser.add_argument("--blades", type=int, metavar="N", default=3,
                        help="Number of prop blades (2, 3, 4). Affects harmonic frequency "
                             "prediction. Default: 3 (triblade, most common for racing/freestyle).")
    parser.add_argument("--cells", type=int, metavar="S",
                        help="Battery cell count (e.g., 4 for 4S, 6 for 6S). "
                             "Used with --kv to predict noise frequencies.")
    parser.add_argument("--kv", type=int, metavar="KV",
                        help="Motor KV rating (e.g., 2400, 1750, 900). "
                             "Used with --cells to predict where prop noise will be.")
    parser.add_argument("--no-narrative", action="store_true",
                        help="Omit the plain-English description of the quad's behavior.")
    parser.add_argument("--no-color", action="store_true",
                        help="Disable colored terminal output.")
    parser.add_argument("--config", metavar="FILE", default=None,
                        help="Path to a CLI 'dump all' or 'diff all' file for enriched analysis. "
                             "Adds config cross-referencing to nav and tuning results. "
                             "Not needed with --device (config is pulled automatically). "
                             "Also auto-discovered if a *_dump.txt or *_diff.txt file sits next to the BBL.")
    parser.add_argument("--nav", action="store_true",
                        help="(deprecated, nav analysis now runs automatically when data is available)")
    parser.add_argument("--no-nav", action="store_true",
                        help="Skip navigation analysis even if nav fields are present in the log.")
    parser.add_argument("--no-db", action="store_true",
                        help="Skip storing results in flight database.")
    parser.add_argument("--db-path", default="./inav_flights.db",
                        help="Path to flight database (default: ./inav_flights.db)")
    parser.add_argument("--history", action="store_true",
                        help="Show flight history and progression for the craft, then exit.")
    parser.add_argument("--trend", metavar="FILE", nargs="?", const="auto",
                        help="Generate HTML trend report showing score progression across flights. "
                             "Optionally specify output filename (default: <craft>_trend.html).")
    parser.add_argument("--compare", metavar="FILE",
                        help="Second log file for side-by-side comparison. "
                             "Usage: inav-analyze flight_A.bbl --compare flight_B.bbl")
    parser.add_argument("--replay", action="store_true",
                        help="Generate interactive HTML replay with time-series plots.")
    parser.add_argument("--check-log", action="store_true",
                        help="Assess log quality and exit. Reports duration, sample rate, "
                             "field completeness, corrupt frames, and usability grade.")
    parser.add_argument("--config-history", action="store_true",
                        help="List archived FC configs from the vault, then exit. "
                             "Configs are archived automatically on each --device auto run.")
    parser.add_argument("--config-diff", nargs=2, metavar=("N", "M"), type=int,
                        help="Compare two archived configs from the vault by number. "
                             "Use --config-history to see available configs. "
                             "Usage: inav-analyze --config-diff 1 2")
    parser.add_argument("--report", metavar="FORMAT", choices=["md", "markdown"],
                        help="Generate Markdown report alongside HTML. "
                             "Usage: inav-analyze flight.bbl --report md")
    parser.add_argument("--lang", metavar="LANG",
                        help="Language for output (en, pt_BR, es). "
                             "Auto-detects from INAV_LANG env var or system locale.")
    args = parser.parse_args()

    # Initialize localization
    try:
        from inav_toolkit.i18n import set_locale, detect_locale
    except ImportError:
        from i18n import set_locale, detect_locale
    lang = args.lang or detect_locale()
    set_locale(lang)

    if args.no_color:
        _disable_colors()

    # ── Device mode: download blackbox from FC ──
    logfile = args.logfile
    config_raw = None
    device_port = None
    if args.device:
        try:
            try:
                from inav_toolkit.msp import INAVDevice, auto_detect_fc, find_serial_ports
            except ImportError:
                from inav_msp import INAVDevice, auto_detect_fc, find_serial_ports
            import time as _time
        except ImportError:
            print("  ERROR: MSP module not found.")
            print("    Install: pip install inav-toolkit")
            sys.exit(1)

        print(f"\n  ▲ INAV Blackbox Analyzer v{REPORT_VERSION}")

        fc = None
        if args.device == "auto":
            print("  Scanning for INAV flight controller...")
            fc, info = auto_detect_fc()
            if not fc:
                print("  ERROR: No INAV flight controller found.")
                ports = find_serial_ports()
                if ports:
                    print(f"    Ports found but none responded as INAV: {', '.join(ports)}")
                    print("    Make sure the FC is powered and not in DFU mode.")
                else:
                    print("    No serial ports detected. Is the FC connected via USB?")
                sys.exit(1)
            print(f"  Found: {fc.port_path}")
        else:
            port = args.device
            # On Windows, COM ports don't exist as filesystem paths
            is_com = port.upper().startswith("COM")
            if not is_com and not os.path.exists(port):
                print(f"  ERROR: Port not found: {port}")
                sys.exit(1)
            print(f"  Connecting: {port}")
            fc = INAVDevice(port)
            fc.open()
            info = fc.get_info()

        try:
            if not info:
                print("  ERROR: No response from FC. Check connection and baud rate.")
                sys.exit(1)

            if info.get("fc_variant") != "INAV":
                print(f"  ERROR: Not an INAV FC (got: {info.get('fc_variant', '?')})")
                sys.exit(1)

            print(f"  Connected: {info['craft_name'] or '(unnamed)'} - {info['firmware']}")

            # Allow serial to settle after identification handshake
            _time.sleep(0.2)
            fc._ser.reset_input_buffer()

            summary = fc.get_dataflash_summary()
            if not summary:
                print("  ERROR: Dataflash not responding (no MSP response).")
                print("    Try disconnecting and reconnecting USB.")
                sys.exit(1)
            if not summary["supported"]:
                print(f"  ERROR: Dataflash not supported (flags={summary}).")
                sys.exit(1)

            used_kb = summary['used_size'] / 1024
            total_kb = summary['total_size'] / 1024
            pct = summary['used_size'] * 100 // summary['total_size'] if summary['total_size'] > 0 else 0
            print(f"  Dataflash: {used_kb:.0f}KB / {total_kb:.0f}KB ({pct}% used)")

            # Pull full config dump (all parameters — used for analysis, fingerprinting, and backup)
            config_raw = None
            print("  Pulling configuration (dump all)...", end="", flush=True)
            config_raw = fc.get_dump_all(timeout=30.0)
            if config_raw:
                n_settings = len([l for l in config_raw.splitlines() if l.strip().startswith("set ")])
                print(f" {n_settings} parameters")
                config_path = os.path.join(args.blackbox_dir, f"{info['craft_name'] or 'fc'}_dump.txt")
                os.makedirs(args.blackbox_dir, exist_ok=True)
                with open(config_path, "w") as f:
                    f.write(config_raw)
                print(f"  Saved: {config_path}")

                # Archive to config vault
                vault_path = vault_archive(config_raw, info.get("craft_name"), args.blackbox_dir)
                print(f"  Vault: {vault_path}")
            else:
                print(" no response (FC may not support CLI over MSP)")

            # Check blackbox readiness from dump
            if config_raw:
                readiness = check_blackbox_readiness_from_dump(config_raw)
                if not readiness["ok"]:
                    _print_readiness(readiness)
                    sys.exit(1)
                if readiness["items"]:
                    _print_readiness(readiness)

            # Pre-flight safety checklist
            if config_raw:
                checklist = preflight_checklist(config_raw, frame_inches=args.frame)
                _print_preflight(checklist)

            if summary['used_size'] == 0:
                print("  No blackbox data to download.")
                sys.exit(0)

            print()
            filepath = fc.download_blackbox(
                output_dir=args.blackbox_dir,
                erase_after=False,  # erase happens after successful analysis
            )

            if not filepath:
                # First attempt failed (often stale serial after CLI exit)
                # Reconnect and retry once
                print("  Retrying download...")
                try:
                    fc.close()
                except Exception:
                    pass
                import time as _time2
                _time2.sleep(1.0)
                fc = INAVDevice(fc.port_path)
                fc.open()
                fc.get_info()
                filepath = fc.download_blackbox(
                    output_dir=args.blackbox_dir,
                    erase_after=False,
                )

            if not filepath:
                print("  ERROR: Download failed.")
                sys.exit(1)

            # Store port for post-analysis erase
            device_port = fc.port_path if hasattr(fc, 'port_path') else args.device

            if args.download_only:
                print(f"\n  To analyze:\n    python3 {sys.argv[0]} {filepath}")
                sys.exit(0)

            logfile = filepath
            print()  # visual separator before analysis

        except KeyboardInterrupt:
            print("\n  Interrupted.")
            sys.exit(1)
        except Exception as e:
            print(f"  ERROR: {e}")
            sys.exit(1)
        finally:
            if fc:
                fc.close()

    if not logfile:
        parser.error("logfile is required when --device is not specified")

    if not os.path.isfile(logfile):
        print(f"ERROR: File not found: {logfile}"); sys.exit(1)

    # ── Load diff from file (when not using --device) ──
    if config_raw is None:
        config_file = None
        if args.config:
            # Explicit file path: --config my_dump.txt
            config_file = args.config
        else:
            # Auto-discover config files in same directory as BBL
            # Prefer dump (complete) over diff (only changed params)
            log_dir = os.path.dirname(os.path.abspath(logfile))
            candidates = []
            for fname in os.listdir(log_dir):
                if (fname.endswith("_dump.txt") or fname.endswith("_diff.txt")
                        or fname in ("dump.txt", "dump_all.txt", "diff.txt", "diff_all.txt")):
                    candidates.append(os.path.join(log_dir, fname))
            if candidates:
                # Prefer dump over diff, then most recent
                def _score(path):
                    is_dump = "_dump" in path or "dump" in os.path.basename(path)
                    return (1 if is_dump else 0, os.path.getmtime(path))
                config_file = max(candidates, key=_score)

        if config_file:
            if os.path.isfile(config_file):
                try:
                    with open(config_file, "r", errors="ignore") as f:
                        config_raw = f.read()
                    n_settings = len([l for l in config_raw.splitlines()
                                      if l.strip().startswith("set ")])
                    if n_settings > 0:
                        print(f"  Config: {os.path.basename(config_file)} ({n_settings} settings)")
                    else:
                        config_raw = None  # not a valid config file
                except Exception:
                    config_raw = None
            else:
                print(f"  Warning: Config file not found: {config_file}")

    # ── Config vault: list or diff archived configs ──
    if getattr(args, 'config_history', False):
        entries = vault_list(blackbox_dir=args.blackbox_dir)
        _print_vault_list(entries)
        sys.exit(0)

    if getattr(args, 'config_diff', None):
        n, m = args.config_diff
        entries = vault_list(blackbox_dir=args.blackbox_dir, limit=100)
        if not entries:
            print("  No configs in vault.")
            sys.exit(1)
        if n < 1 or n > len(entries) or m < 1 or m > len(entries):
            print(f"  Invalid config numbers. Use --config-history to see {len(entries)} available.")
            sys.exit(1)
        path_a = entries[n - 1]["path"]
        path_b = entries[m - 1]["path"]
        diff = vault_diff(path_a, path_b)
        _print_vault_diff(diff, path_a, path_b)
        sys.exit(0)

    # ── History mode: show progression and exit ──
    if args.history or args.trend:
        try:
            from inav_toolkit.flight_db import FlightDB
        except ImportError:
            from inav_flight_db import FlightDB
        db = FlightDB(args.db_path)
        # Need to peek at craft name from headers
        rp = parse_headers_from_bbl(logfile) if os.path.isfile(logfile) else {}
        cfg = extract_fc_config(rp)
        craft = cfg.get("craft_name", "unknown")
        html_path = None
        if args.trend:
            if args.trend == "auto":
                safe_name = craft.replace(" ", "_").replace("/", "_")
                html_path = f"{safe_name}_trend.html"
            else:
                html_path = args.trend
        _print_flight_history(db, craft, html_path=html_path)
        db.close()
        sys.exit(0)

    # ── Comparison mode: analyze two flights side-by-side ──
    if args.compare:
        if not logfile:
            parser.error("Two log files required: inav-analyze A.bbl --compare B.bbl")
        if not os.path.isfile(args.compare):
            print(f"ERROR: Comparison file not found: {args.compare}")
            sys.exit(1)
        _run_comparison(logfile, args.compare, args, config_raw)
        return

    # ── Replay mode: interactive HTML time-series ──
    if args.replay:
        if not logfile:
            parser.error("logfile required for --replay")
        _run_replay(logfile, args, config_raw)
        return

    # ── Log quality check mode ──
    if getattr(args, 'check_log', False):
        if not logfile:
            parser.error("logfile required for --check-log")
        raw_params = parse_headers_from_bbl(logfile)
        config = extract_fc_config(raw_params)
        ext = os.path.splitext(logfile)[1].lower()
        if ext in (".bbl", ".bfl", ".bbs"):
            data = decode_blackbox_native(logfile, raw_params, quiet=True)
        else:
            data = parse_csv_log(logfile)
        quality = assess_log_quality(data, config, logfile)
        print_log_quality(quality, verbose=True)
        sys.exit(0 if quality["usable"] else 1)

    # ── Multi-log detection and splitting ──
    n_logs = count_blackbox_logs(logfile)
    log_files = [logfile]
    if n_logs > 1:
        print(f"\n  Found {n_logs} flights in {os.path.basename(logfile)}")
        log_files = split_blackbox_logs(logfile, output_dir=args.blackbox_dir)
        print(f"  Split into {len(log_files)} files:")
        for lf in log_files:
            print(f"    {os.path.basename(lf)}")
        print()

    # ── Process flights ──
    # Workflow: user flies → downloads → implements suggestions → flies again.
    # Flash may contain flights from multiple sessions (config changes between
    # arm cycles). Only the LATEST session's best flight deserves full analysis.
    # Earlier flights are stored in DB for progression but shown as one-liners.
    analyzed_file = None
    if getattr(args, 'nav', False):
        # Nav mode: skip multi-flight tuning scan, analyze last flight directly
        target = log_files[-1]
        if len(log_files) > 1:
            print(f"\n  Nav mode: analyzing last flight ({len(log_files)} in flash)")
        _analyze_single_log(target, args, config_raw)
        analyzed_file = target
    elif len(log_files) > 1:
        analyzed_file = _process_multi_log(log_files, args, config_raw)
    else:
        # Single flight - full analysis
        _analyze_single_log(log_files[0], args, config_raw)
        analyzed_file = log_files[0]

    # ── Post-analysis cleanup (only in device mode) ──
    if device_port and not args.keep_logs:
        # raw_download is the original downloaded file (pre-split)
        # For single-flight, it's the same as the analyzed file
        raw_download = logfile
        split_files = log_files if n_logs > 1 else []

        _post_analysis_cleanup(
            args.blackbox_dir, raw_download, split_files, analyzed_file,
            archive=args.archive, keep_logs=args.keep_logs)

        # Erase FC flash
        if not args.no_erase:
            try:
                try:
                    from inav_toolkit.msp import INAVDevice
                except ImportError:
                    from inav_msp import INAVDevice
                fc2 = INAVDevice(device_port)
                fc2.open()
                fc2.erase_dataflash()
                fc2.close()
            except Exception as e:
                print(f"  Erase failed: {e}")


def _process_multi_log(log_files, args, config_raw):
    """Handle multiple flights from a single flash download.

    Strategy:
    1. Quick-scan all flights (summary_only) for metadata
    2. Group into sessions by config fingerprint
    3. Select best flight from latest session (longest non-idle)
    4. Show compact table of all flights with session boundaries
    5. Full analysis only on the selected flight
    """
    R, B, C, G, Y, RED, DIM = _colors()

    # ── Phase 1: Quick-scan all flights ──
    print(f"  Scanning {len(log_files)} flights...")
    summaries = []
    for lf in log_files:
        s = _analyze_single_log(lf, args, config_raw, summary_only=True)
        if s:
            summaries.append(s)

    if not summaries:
        print("  ERROR: Could not parse any flights.")
        return

    # ── Phase 2: Classify flights as current or old ──
    # If we have the live FC diff, that's ground truth: flights whose
    # header config matches the diff are "current session" (post-change),
    # flights that don't match are "old session" (pre-change).
    # Without a diff, fall back to comparing consecutive flights.
    current_fp = _fingerprint_from_diff(config_raw)
    is_current = []  # True/False per flight

    if current_fp:
        # Ground truth from FC diff
        for s in summaries:
            is_current.append(s.get("config_key", "") == current_fp)
    else:
        # Fallback: group by consecutive matching config fingerprints.
        # Assume the last group is "current" since user hasn't changed
        # config after their most recent flight.
        last_fp = summaries[-1].get("config_key", "")
        for s in summaries:
            is_current.append(s.get("config_key", "") == last_fp)

    n_current = sum(is_current)
    n_old = len(summaries) - n_current
    multi_session = n_old > 0

    # ── Phase 3: Select best flight from current session ──
    # Pick the longest non-idle flight from the current config group.
    best_idx = None
    best_dur = -1
    for i, s in enumerate(summaries):
        if not is_current[i]:
            continue
        dur = s.get("duration", 0)
        idle = s.get("idle", False)
        if not idle and dur > best_dur:
            best_dur = dur
            best_idx = i

    if best_idx is None:
        # All current flights are idle - pick longest current
        for i, s in enumerate(summaries):
            if not is_current[i]:
                continue
            dur = s.get("duration", 0)
            if dur > best_dur:
                best_dur = dur
                best_idx = i

    if best_idx is None:
        # No current flights at all (shouldn't happen) - pick last
        best_idx = len(summaries) - 1

    # ── Phase 4: Print compact flight table ──
    print(f"\n  {B}{'═' * 66}{R}")
    if multi_session:
        src = "diff" if current_fp else "headers"
        print(f"  {B}FLASH CONTENTS:{R} {len(summaries)} flights, "
              f"{n_old} old + {n_current} current (detected from {src})")
    else:
        print(f"  {B}FLASH CONTENTS:{R} {len(summaries)} flights, same config throughout")
    print(f"  {B}{'─' * 66}{R}")
    print(f"  {'#':>3}  {'Duration':>8}  {'Score':>7}  {'Status':<28}  {'Notes'}")
    print(f"  {'─'*3}  {'─'*8}  {'─'*7}  {'─'*28}  {'─'*15}")

    # Print with session boundary markers
    prev_current = None
    for i, s in enumerate(summaries):
        score = s.get("score")
        dur = s.get("duration", 0)
        idle = s.get("idle", False)
        verdict = s.get("verdict", "")

        # Don't show 0/100 for bad/ground data — use dash
        if verdict in ("UNUSABLE", "GROUND_ONLY") or (score is not None and score == 0):
            sc_str = "     —"
            sc_c = DIM
        elif score is not None:
            sc_str = f"{score:.0f}/100"
            sc_c = G if score >= 85 else Y if score >= 60 else RED
        else:
            sc_str = "     —"
            sc_c = DIM
        dur_str = f"{dur:.1f}s" if dur and dur > 0.1 else "  <1s" if dur else "    —"

        if idle:
            status = f"{DIM}idle/ground{R}"
        else:
            status = _verdict_short(verdict)

        notes = ""
        if i == best_idx:
            notes = f"{G}◄ analyzing this one{R}"
        elif not is_current[i]:
            notes = f"{DIM}old config{R}"

        # Session boundary marker
        if multi_session and prev_current is not None and is_current[i] != prev_current:
            print(f"  {Y}{'· ' * 33}{R}")
            print(f"  {Y}  ◆ Config changed - recommendations above no longer apply{R}")
            print(f"  {Y}{'· ' * 33}{R}")

        is_best = (i == best_idx)
        print(f"  {DIM if not is_best else ''}{i+1:3}{R}  {dur_str:>8}  "
              f"{sc_c if is_best else DIM}{sc_str:>7}{R}  "
              f"{status:<28}  {notes}")
        prev_current = is_current[i]

    print(f"  {B}{'═' * 66}{R}")

    # Summary message
    best_s = summaries[best_idx]
    n_idle = sum(1 for s in summaries if s.get("idle", False))
    if n_idle > 0:
        print(f"\n  {DIM}{n_idle} ground segment{'s' if n_idle > 1 else ''} skipped "
              f"(armed but no flight){R}")
    if n_current == 0 and current_fp:
        # All flights predate the current config
        print(f"\n  {Y}{B}⚠ All flights on flash predate your current FC config.{R}")
        print(f"  {DIM}  Erase flash and fly again for analysis of your current tune.{R}")
        print(f"  {DIM}  Showing best available flight for reference:{R}")
    elif multi_session:
        print(f"  {DIM}{n_old} flight{'s' if n_old > 1 else ''} from previous config "
              f"(recommendations no longer apply){R}")

    dur_str = f"{best_s.get('duration', 0):.1f}s" if best_s.get('duration') else "?"
    config_label = "current config" if is_current[best_idx] else "old config — reference only"
    print(f"\n  Analyzing flight #{best_idx + 1} ({dur_str}, {config_label})...\n")

    # ── Phase 5: Full analysis on the selected flight ──
    print(f"{'═' * 70}")
    _analyze_single_log(log_files[best_idx], args, config_raw)

    return log_files[best_idx]


def _verdict_short(verdict):
    """Short display string for verdict codes."""
    return {"DIALED_IN": "✓ dialed in", "NEARLY_THERE": "✓ nearly there",
            "GETTING_BETTER": "↗ getting better", "NEEDS_WORK": "⚠ needs work",
            "ROUGH": "✖ rough", "NEED_DATA": "? need data",
            "GROUND_ONLY": "- ground", "UNUSABLE": "✖ bad data"}.get(verdict, verdict or "?")


def _config_fingerprint(config):
    """Generate a short fingerprint of tuning-relevant config for change detection."""
    parts = []
    for ax in ["roll", "pitch", "yaw"]:
        p = config.get(f"{ax}_p")
        d = config.get(f"{ax}_d")
        if p is not None:
            parts.append(f"{ax[0]}P{p}D{d or 0}")
    for k in ["gyro_lowpass_hz", "dterm_lpf_hz", "dyn_notch_min_hz"]:
        v = config.get(k)
        if v is not None:
            parts.append(f"{k}={v}")
    return "|".join(parts) if parts else ""


def _fingerprint_from_diff(config_raw):
    """Build a config fingerprint from CLI 'diff all' output.

    This represents the FC's CURRENT config - the ground truth.
    Flights whose header fingerprint matches this are from the current
    session; flights that don't match are from before the user applied
    changes.
    """
    if not config_raw:
        return ""

    try:
        from inav_toolkit.flight_db import parse_diff_output
    except ImportError:
        from inav_flight_db import parse_diff_output
    diff_settings = parse_diff_output(config_raw)

    # Map CLI names → config keys (same mapping as merge_diff_into_config)
    config = {}
    for cli_name, cli_value in diff_settings.items():
        config_key = CLI_TO_CONFIG.get(cli_name)
        if config_key is None:
            continue
        try:
            config[config_key] = int(cli_value)
        except (ValueError, TypeError):
            config[config_key] = cli_value

    return _config_fingerprint(config)


def _print_config_review(config_raw, config, frame_inches, plan):
    """Run parameter analyzer on FC config and show findings not covered by flight analysis.

    Accepts either 'dump all' or 'diff all' output. Dump is preferred since it
    includes every parameter including unchanged defaults, which matters for
    nav PID checks on large frames.

    Only shows CRITICAL and WARNING findings from categories that the blackbox
    analyzer doesn't cover (safety, nav, motor protocol, GPS, battery, RX).
    Filter and PID findings are skipped since the flight-based analysis is
    more accurate for those.
    """
    R, B, C, G, Y, RED, DIM = _colors()

    if not config_raw:
        return

    try:
        try:
            from inav_toolkit.param_analyzer import parse_diff_all, run_all_checks, CRITICAL, WARNING
        except ImportError:
            from inav_param_analyzer import parse_diff_all, run_all_checks, CRITICAL, WARNING
    except ImportError:
        return  # param analyzer not available

    try:
        parsed = parse_diff_all(config_raw)
    except Exception:
        return

    # Load blackbox state if available for cross-reference
    bb_state = None
    # Run all checks
    findings = run_all_checks(parsed, frame_inches=frame_inches, blackbox_state=bb_state)

    # Filter: skip categories the blackbox analyzer already handles from flight data
    # These are more accurately assessed from actual flight data than from config alone
    skip_categories = {"Filter", "PID"}
    # Also skip OK and INFO - only show actionable issues
    relevant = [f for f in findings
                if f.severity in (CRITICAL, WARNING)
                and f.category not in skip_categories]

    if not relevant:
        return

    n_crit = sum(1 for f in relevant if f.severity == CRITICAL)
    n_warn = sum(1 for f in relevant if f.severity == WARNING)

    label_parts = []
    if n_crit: label_parts.append(f"{n_crit} critical")
    if n_warn: label_parts.append(f"{n_warn} warning{'s' if n_warn > 1 else ''}")

    print(f"\n{B}{Y}{'─' * 70}{R}")
    print(f"  {B}CONFIG REVIEW{R} ({', '.join(label_parts)} from parameter analysis)")
    print(f"{B}{Y}{'─' * 70}{R}")

    for f in relevant:
        if f.severity == CRITICAL:
            icon = f"{RED}{B}✖{R}"
            sev_str = f"{RED}{B}CRITICAL{R}"
        else:
            icon = f"{Y}⚠{R}"
            sev_str = f"{Y}WARNING{R}"

        print(f"\n  {icon} [{sev_str}] {B}{f.title}{R}")
        # Wrap detail text
        import textwrap
        for line in textwrap.wrap(f.detail, width=64):
            print(f"    {DIM}{line}{R}")
        if f.cli_fix:
            # Show just the first line of the CLI fix as a hint
            fix_line = f.cli_fix.strip().split("\n")[0]
            if len(f.cli_fix.strip().split("\n")) > 1:
                fix_line += " ..."
            print(f"    {G}Fix: {fix_line}{R}")

    print(f"\n  {DIM}Run 'inav-params diff.txt' for the full config review.{R}")
    print(f"{B}{Y}{'─' * 70}{R}")


def _print_flight_history(db, craft, html_path=None):
    """Print flight history table for a craft, optionally generate HTML trend report."""
    R, B, C, G, Y, RED, DIM = _colors()
    history = db.get_craft_history(craft, limit=20)
    if not history:
        print(f"  No flight history for '{craft}'")
        return

    print(f"\n{B}{C}{'═' * 70}{R}")
    print(f"  {B}FLIGHT HISTORY: {craft}{R}  ({len(history)} flights)")
    print(f"{B}{C}{'─' * 70}{R}")
    print(f"  {'#':>3}  {'Date':16}  {'Dur':>5}  {'Score':>5}  {'Noise':>5}  {'PID':>5}  {'Osc':>5}  {'Verdict'}")
    print(f"  {'─'*3}  {'─'*16}  {'─'*5}  {'─'*5}  {'─'*5}  {'─'*5}  {'─'*5}  {'─'*16}")

    for i, f in enumerate(reversed(history)):  # Chronological order
        ts = f["timestamp"][:16].replace("T", " ")
        dur = f"{f['duration_s']:.0f}s" if f["duration_s"] else "?"
        sc = f["overall_score"]
        sc_str = f"{sc:.0f}" if sc is not None else "?"
        ns = f"{f['noise_score']:.0f}" if f["noise_score"] is not None else "?"
        ps = f"{f['pid_score']:.0f}" if f["pid_score"] is not None else "N/A"
        osc = f"{f['osc_score']:.0f}" if f["osc_score"] is not None else "?"
        v = f["verdict"] or "?"

        sc_c = G if (sc or 0) >= 85 else Y if (sc or 0) >= 60 else RED if sc is not None else DIM
        v_str = "- ground" if v == "GROUND_ONLY" else v
        print(f"  {i+1:3}  {ts}  {dur:>5}  {sc_c}{sc_str:>5}{R}  {ns:>5}  {ps:>5}  {osc:>5}  {DIM}{v_str}{R}")

    # Show progression
    prog = db.get_progression(craft)
    if prog["changes"]:
        print(f"\n  {B}Latest changes:{R}")
        for ch in prog["changes"]:
            print(f"    {ch}")

    print(f"{B}{C}{'═' * 70}{R}\n")

    # Generate HTML trend report if requested
    if html_path:
        _generate_trend_html(history, craft, prog, html_path)


def _generate_trend_html(history, craft, progression, output_path):
    """Generate an HTML trend report with embedded SVG charts."""
    # Build chronological flight data (exclude ground-only)
    flights = [f for f in reversed(history)
               if f.get("verdict") != "GROUND_ONLY" and f.get("overall_score") is not None]

    if len(flights) < 2:
        print(f"  Need at least 2 scored flights for a trend report (have {len(flights)}).")
        return

    # Build data arrays
    labels = [f["timestamp"][:10] for f in flights]
    scores = [f["overall_score"] or 0 for f in flights]
    noise = [f["noise_score"] or 0 for f in flights]
    pid = [f["pid_score"] or 0 for f in flights]
    osc = [f["osc_score"] or 0 for f in flights]
    motor = [f["motor_score"] or 0 for f in flights]

    # Build per-axis PID data
    axis_p = {"Roll": [], "Pitch": [], "Yaw": []}
    axis_d = {"Roll": [], "Pitch": [], "Yaw": []}
    for f in flights:
        for axis in ["Roll", "Pitch", "Yaw"]:
            ax_data = next((a for a in f.get("axes", []) if a["axis"] == axis), None)
            axis_p[axis].append(ax_data["p_value"] if ax_data and ax_data.get("p_value") else None)
            axis_d[axis].append(ax_data["d_value"] if ax_data and ax_data.get("d_value") else None)

    n = len(flights)

    def svg_line_chart(title, series, width=700, height=250):
        """Generate SVG line chart with multiple series."""
        # series = [(label, values, color), ...]
        margin = {"top": 30, "right": 120, "bottom": 50, "left": 50}
        w = width - margin["left"] - margin["right"]
        h = height - margin["top"] - margin["bottom"]

        all_vals = [v for _, vals, _ in series for v in vals if v is not None]
        if not all_vals:
            return ""
        y_min = max(0, min(all_vals) - 5)
        y_max = min(100, max(all_vals) + 5)
        if y_max - y_min < 10:
            y_max = y_min + 10

        def x(i):
            return margin["left"] + (i / max(1, n - 1)) * w if n > 1 else margin["left"] + w / 2

        def y(v):
            return margin["top"] + h - ((v - y_min) / (y_max - y_min)) * h

        svg = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">']
        svg.append(f'<text x="{width//2}" y="18" text-anchor="middle" fill="#ccc" font-size="14">{title}</text>')

        # Grid lines
        for tick in range(int(y_min), int(y_max) + 1, 10):
            yp = y(tick)
            svg.append(f'<line x1="{margin["left"]}" y1="{yp}" x2="{margin["left"]+w}" y2="{yp}" stroke="#333" stroke-dasharray="3"/>')
            svg.append(f'<text x="{margin["left"]-5}" y="{yp+4}" text-anchor="end" fill="#888" font-size="11">{tick}</text>')

        # X labels
        step = max(1, n // 8)
        for i in range(0, n, step):
            svg.append(f'<text x="{x(i)}" y="{height-10}" text-anchor="middle" fill="#888" font-size="10">{labels[i]}</text>')

        # Lines
        for label, vals, color in series:
            points = [(x(i), y(v)) for i, v in enumerate(vals) if v is not None]
            if len(points) < 2:
                continue
            path = " ".join(f"{'M' if j==0 else 'L'}{px:.1f},{py:.1f}" for j, (px, py) in enumerate(points))
            svg.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2"/>')
            # Dots
            for px, py in points:
                svg.append(f'<circle cx="{px}" cy="{py}" r="3" fill="{color}"/>')
            # Legend
            ly = margin["top"] + 15 + series.index((label, vals, color)) * 18
            lx = margin["left"] + w + 10
            svg.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+15}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
            svg.append(f'<text x="{lx+20}" y="{ly+4}" fill="#ccc" font-size="11">{label}</text>')

        svg.append('</svg>')
        return "\n".join(svg)

    # Build charts
    score_chart = svg_line_chart("Overall Score", [
        ("Overall", scores, "#4fc3f7"),
    ])
    breakdown_chart = svg_line_chart("Score Breakdown", [
        ("Noise", noise, "#81c784"),
        ("PID", pid, "#ffb74d"),
        ("Oscillation", osc, "#e57373"),
        ("Motor", motor, "#ba68c8"),
    ])

    # PID value chart
    pid_series = []
    colors_p = {"Roll": "#4fc3f7", "Pitch": "#81c784", "Yaw": "#ffb74d"}
    for axis in ["Roll", "Pitch", "Yaw"]:
        if any(v is not None for v in axis_p[axis]):
            pid_series.append((f"{axis} P", axis_p[axis], colors_p[axis]))
    pid_chart = svg_line_chart("P-gain Evolution", pid_series) if pid_series else ""

    # Trend text
    trend = progression.get("trend", "insufficient")
    trend_icon = {"improving": "↑ Improving", "stable": "→ Stable",
                  "degrading": "↓ Degrading"}.get(trend, "? Insufficient data")
    changes_html = ""
    if progression.get("changes"):
        changes_html = "<ul>" + "".join(f"<li>{ch}</li>" for ch in progression["changes"]) + "</ul>"

    # Flight table
    table_rows = ""
    for i, f in enumerate(flights):
        sc = f["overall_score"]
        sc_class = "good" if (sc or 0) >= 85 else "ok" if (sc or 0) >= 60 else "bad"
        table_rows += f"""<tr>
            <td>{i+1}</td>
            <td>{f['timestamp'][:16].replace('T',' ')}</td>
            <td>{f['duration_s']:.0f}s</td>
            <td class="{sc_class}">{sc:.0f}</td>
            <td>{f['noise_score']:.0f if f['noise_score'] else '-'}</td>
            <td>{f['pid_score']:.0f if f['pid_score'] else 'N/A'}</td>
            <td>{f['verdict'] or '?'}</td>
        </tr>"""

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Tuning Trend - {craft}</title>
<style>
body {{ background:#1a1a2e; color:#ccc; font-family:system-ui; max-width:800px; margin:0 auto; padding:20px; }}
h1 {{ color:#4fc3f7; border-bottom:1px solid #333; padding-bottom:10px; }}
h2 {{ color:#81c784; margin-top:30px; }}
.trend {{ font-size:1.4em; padding:15px; border-radius:8px; margin:15px 0;
          background:#222; border-left:4px solid {{ 'improving':'#81c784','stable':'#ffb74d','degrading':'#e57373' }}.get(trend,'#888'); }}
table {{ width:100%; border-collapse:collapse; margin:15px 0; }}
th, td {{ padding:8px 12px; text-align:right; border-bottom:1px solid #333; }}
th {{ color:#4fc3f7; text-align:right; }}
td:first-child, th:first-child {{ text-align:center; }}
td:nth-child(2), th:nth-child(2) {{ text-align:left; }}
.good {{ color:#81c784; font-weight:bold; }}
.ok {{ color:#ffb74d; }}
.bad {{ color:#e57373; font-weight:bold; }}
svg {{ width:100%; height:auto; background:#222; border-radius:8px; margin:10px 0; }}
ul {{ color:#aaa; }}
footer {{ text-align:center; color:#555; margin-top:30px; padding:10px; border-top:1px solid #333; }}
</style></head><body>
<h1>Tuning Trend: {craft}</h1>
<div class="trend">{trend_icon}</div>
{changes_html}

<h2>Score Progression</h2>
{score_chart}

<h2>Component Breakdown</h2>
{breakdown_chart}

{f'<h2>PID Progression</h2>{pid_chart}' if pid_chart else ''}

<h2>Flight Log</h2>
<table>
<tr><th>#</th><th>Date</th><th>Dur</th><th>Score</th><th>Noise</th><th>PID</th><th>Verdict</th></tr>
{table_rows}
</table>

<footer>INAV Blackbox Analyzer v{REPORT_VERSION} - {n} flights analyzed</footer>
</body></html>"""

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    R, B, C, G, Y, RED, DIM = _colors()
    print(f"  {G}✓{R} Trend report: {output_path}")


# ─── Comparison Mode ─────────────────────────────────────────────────────────

def _analyze_for_compare(logfile, args, config_raw=None):
    """Run analysis pipeline on a single file and return structured results.
    Returns dict with: config, data, noise_results, pid_results, motor_analysis,
                        dterm_results, plan, noise_fp, hover_osc, profile
    """
    raw_params = parse_headers_from_bbl(logfile)
    config = extract_fc_config(raw_params)
    if config_raw:
        merge_diff_into_config(config, config_raw)

    # Auto-detect frame
    craft = config.get("craft_name", "")
    detected_frame = None
    if craft:
        m = re.search(r'\b(\d{1,2})(?:\s*(?:inch|in|"|' + r"'" + r'))?(?:\s|$)', craft, re.I)
        if m:
            candidate = int(m.group(1))
            if 3 <= candidate <= 15:
                detected_frame = candidate

    frame_inches = args.frame or detected_frame
    prop_inches = args.props or frame_inches
    n_blades = args.blades
    profile = get_frame_profile(frame_inches, prop_inches, n_blades)

    # Decode
    ext = os.path.splitext(logfile)[1].lower()
    is_blackbox = ext in (".bbl", ".bfl", ".bbs")
    if is_blackbox:
        data = decode_blackbox_native(logfile, raw_params, quiet=True)
    else:
        data = parse_csv_log(logfile)

    sr = data["sample_rate"]

    # Run all analyses
    hover_osc = detect_hover_oscillation(data, sr, profile)
    noise_results = [analyze_noise(data, ax, f"gyro_{ax.lower()}", sr) for ax in AXIS_NAMES]
    pid_results = [analyze_pid_response(data, i, sr) for i in range(3)]
    motor_analysis = analyze_motors(data, sr, config)
    dterm_results = analyze_dterm_noise(data, sr)

    rpm_range = estimate_rpm_range(args.kv, args.cells)
    prop_harmonics = estimate_prop_harmonics(rpm_range, n_blades) if rpm_range else None
    noise_fp = fingerprint_noise(noise_results, config, prop_harmonics)

    motor_response = analyze_motor_response(data, sr)
    phase_lag = None
    if config_has_filters(config):
        sig_freq = (profile["noise_band_mid"][0] + profile["noise_band_mid"][1]) / 4
        phase_lag = estimate_total_phase_lag(config, profile, sig_freq)

    plan = generate_action_plan(noise_results, pid_results, motor_analysis, dterm_results,
                                config, data, profile, phase_lag, motor_response,
                                rpm_range, prop_harmonics, hover_osc)
    plan["noise_fingerprint"] = noise_fp

    return {
        "config": config, "data": data, "sr": sr,
        "noise_results": noise_results, "pid_results": pid_results,
        "motor_analysis": motor_analysis, "dterm_results": dterm_results,
        "plan": plan, "noise_fp": noise_fp, "hover_osc": hover_osc,
        "profile": profile, "logfile": logfile,
    }


def _create_comparison_noise_chart(nr_a, nr_b, label_a, label_b):
    """Create overlaid noise spectrum chart for two flights."""
    setup_dark_style()
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    fig.suptitle("Noise Spectrum Comparison", fontsize=14, color="#c0caf5", fontweight="bold", y=1.02)

    for i in range(3):
        ax = axes[i]
        a_r = nr_a[i] if nr_a[i] else None
        b_r = nr_b[i] if nr_b[i] else None

        if a_r:
            ax.plot(a_r["freqs"], a_r["psd_db"], color="#565f89", linewidth=1.5, alpha=0.7, label=label_a)
            ax.fill_between(a_r["freqs"], a_r["psd_db"], -80, alpha=0.08, color="#565f89")
        if b_r:
            ax.plot(b_r["freqs"], b_r["psd_db"], color=AXIS_COLORS[i], linewidth=1.5, alpha=0.9, label=label_b)
            ax.fill_between(b_r["freqs"], b_r["psd_db"], -80, alpha=0.12, color=AXIS_COLORS[i])

        ax.set_title(AXIS_NAMES[i], color=AXIS_COLORS[i], fontweight="bold")
        ax.set_xlabel("Frequency (Hz)")
        if i == 0:
            ax.set_ylabel("Power (dB)")
        max_freq = 1000
        if a_r:
            max_freq = min(max_freq, a_r["freqs"][-1])
        ax.set_xlim(0, max_freq)
        ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")

    fig.tight_layout()
    return fig_to_base64(fig)


def _create_comparison_pid_chart(pid_a, pid_b, sr_a, sr_b, label_a, label_b):
    """Create PID response comparison chart."""
    setup_dark_style()
    fig, axes = plt.subplots(3, 1, figsize=(18, 10), sharex=True)
    fig.suptitle("PID Response Comparison", fontsize=14, color="#c0caf5", fontweight="bold", y=1.01)

    for i in range(3):
        ax = axes[i]
        pa = pid_a[i] if pid_a[i] else None
        pb = pid_b[i] if pid_b[i] else None

        if pa:
            sp, gy = pa["setpoint"], pa["gyro"]
            ws = int(sr_a * 2)
            bs, bv = 0, 0
            for s in range(0, max(1, len(sp) - ws), ws // 4):
                v = np.var(sp[s:s + ws])
                if v > bv:
                    bv, bs = v, s
            sl = slice(bs, min(bs + ws, len(sp)))
            t = np.arange(sl.stop - sl.start) / sr_a * 1000
            n = min(len(t), sl.stop - sl.start)
            ax.plot(t[:n], gy[sl][:n], color="#565f89", linewidth=1.0, alpha=0.6, label=f"Gyro ({label_a})")
            ax.plot(t[:n], sp[sl][:n], color="#3b4261", linewidth=1.0, alpha=0.4, linestyle="--", label=f"SP ({label_a})")

        if pb:
            sp, gy = pb["setpoint"], pb["gyro"]
            ws = int(sr_b * 2)
            bs, bv = 0, 0
            for s in range(0, max(1, len(sp) - ws), ws // 4):
                v = np.var(sp[s:s + ws])
                if v > bv:
                    bv, bs = v, s
            sl = slice(bs, min(bs + ws, len(sp)))
            t = np.arange(sl.stop - sl.start) / sr_b * 1000
            n = min(len(t), sl.stop - sl.start)
            ax.plot(t[:n], gy[sl][:n], color=AXIS_COLORS[i], linewidth=1.2, alpha=0.9, label=f"Gyro ({label_b})")
            ax.plot(t[:n], sp[sl][:n], color=AXIS_COLORS[i], linewidth=1.0, alpha=0.4, linestyle="--", label=f"SP ({label_b})")

        ax.set_title(AXIS_NAMES[i], color=AXIS_COLORS[i], fontweight="bold")
        ax.set_ylabel("deg/s")
        ax.legend(loc="upper right", fontsize=7, facecolor="#1a1b26", edgecolor="#565f89", ncol=2)

    axes[-1].set_xlabel("Time (ms)")
    fig.tight_layout()
    return fig_to_base64(fig)


def _generate_comparison_html(res_a, res_b, charts, label_a, label_b):
    """Generate comparison HTML report."""
    sa = res_a["plan"]["scores"]
    sb = res_b["plan"]["scores"]

    def _delta_html(key, higher_is_better=True):
        va = sa.get(key)
        vb = sb.get(key)
        if va is None or vb is None:
            return "-"
        d = vb - va
        if abs(d) < 0.5:
            return '<span style="color:var(--dm)">→ 0</span>'
        color = "var(--gn)" if (d > 0) == higher_is_better else "var(--rd)"
        sign = "+" if d > 0 else ""
        return f'<span style="color:{color};font-weight:700">{sign}{d:.0f}</span>'

    def _score_cell(val, is_b=False):
        if val is None:
            return '<td style="color:var(--dm)">-</td>'
        color = "var(--gn)" if val >= 85 else "var(--yl)" if val >= 60 else "var(--rd)"
        weight = "700" if is_b else "400"
        return f'<td style="color:{color};font-weight:{weight}">{val:.0f}</td>'

    # Config diff table
    cfg_a = res_a["config"]
    cfg_b = res_b["config"]
    config_rows = ""
    config_keys = set()
    for k in ["roll_p", "roll_i", "roll_d", "pitch_p", "pitch_i", "pitch_d",
              "yaw_p", "yaw_i", "yaw_d", "gyro_lowpass_hz", "dterm_lpf_hz",
              "dyn_notch_min_hz", "dyn_notch_q", "looptime"]:
        va = cfg_a.get(k, "-")
        vb = cfg_b.get(k, "-")
        if va != vb:
            config_rows += (f'<tr><td>{k}</td><td style="color:var(--dm)">{va}</td>'
                           f'<td style="color:var(--bl);font-weight:600">{vb}</td></tr>')
            config_keys.add(k)

    ci = lambda k: f'<img src="data:image/png;base64,{charts[k]}" style="width:100%;border-radius:4px">' if charts.get(k) else ""

    craft_a = cfg_a.get("craft_name", os.path.basename(res_a["logfile"]))
    craft_b = cfg_b.get("craft_name", os.path.basename(res_b["logfile"]))

    # Craft/frame mismatch warning
    warnings_html = ""
    if craft_a != craft_b and craft_a and craft_b:
        warnings_html += (f'<div style="background:rgba(224,175,104,.1);border:1px solid rgba(224,175,104,.3);'
                          f'border-radius:6px;padding:12px;margin:12px 0;color:var(--yl);font-size:.85rem">'
                          f'⚠ Different craft: "{craft_a}" vs "{craft_b}" — thresholds may differ</div>')

    # Big improvement arrow
    ov_a = sa.get("overall", 0) or 0
    ov_b = sb.get("overall", 0) or 0
    ov_delta = ov_b - ov_a
    if ov_delta > 5:
        arrow_html = (f'<div style="text-align:center;padding:24px;margin:16px 0;border-radius:12px;'
                      f'background:rgba(158,206,106,.08);border:2px solid rgba(158,206,106,.3)">'
                      f'<div style="font-size:3rem;color:var(--gn)">▲</div>'
                      f'<div style="font-size:1.5rem;font-weight:700;color:var(--gn)">+{ov_delta:.0f} points</div>'
                      f'<div style="color:var(--dm);font-size:.85rem;margin-top:4px">Tune improved</div></div>')
    elif ov_delta < -5:
        arrow_html = (f'<div style="text-align:center;padding:24px;margin:16px 0;border-radius:12px;'
                      f'background:rgba(247,118,142,.08);border:2px solid rgba(247,118,142,.3)">'
                      f'<div style="font-size:3rem;color:var(--rd)">▼</div>'
                      f'<div style="font-size:1.5rem;font-weight:700;color:var(--rd)">{ov_delta:.0f} points</div>'
                      f'<div style="color:var(--dm);font-size:.85rem;margin-top:4px">Tune degraded</div></div>')
    else:
        arrow_html = (f'<div style="text-align:center;padding:16px;margin:16px 0;border-radius:12px;'
                      f'background:rgba(122,162,247,.06);border:1px solid rgba(122,162,247,.2)">'
                      f'<div style="font-size:1.2rem;color:var(--dm)">→ Roughly the same ({ov_delta:+.0f})</div></div>')

    # Motor balance comparison
    motor_html = ""
    ma_a = res_a.get("motor_analysis")
    ma_b = res_b.get("motor_analysis")
    if ma_a and ma_b and not ma_a.get("idle_detected") and not ma_b.get("idle_detected"):
        sp_a = ma_a.get("balance_spread_pct", 0)
        sp_b = ma_b.get("balance_spread_pct", 0)
        sp_d = sp_b - sp_a
        sp_color = "var(--gn)" if sp_d < -0.5 else "var(--rd)" if sp_d > 0.5 else "var(--dm)"

        motor_rows = ""
        for i, (m_a, m_b) in enumerate(zip(ma_a["motors"], ma_b["motors"])):
            sat_d = m_b["saturation_pct"] - m_a["saturation_pct"]
            sc = "var(--gn)" if sat_d < -0.5 else "var(--rd)" if sat_d > 0.5 else "var(--dm)"
            motor_rows += (f'<tr><td>M{m_a["motor"]}</td>'
                          f'<td>{m_a["avg_pct"]:.1f}%</td><td>{m_b["avg_pct"]:.1f}%</td>'
                          f'<td>{m_a["saturation_pct"]:.1f}%</td><td>{m_b["saturation_pct"]:.1f}%</td>'
                          f'<td style="color:{sc}">{sat_d:+.1f}%</td></tr>')

        motor_html = (f'<section><h2>Motor Balance</h2><div class="cc"><table>'
                     f'<tr><th>Motor</th><th>Avg A</th><th>Avg B</th>'
                     f'<th>Sat A</th><th>Sat B</th><th>Δ Sat</th></tr>'
                     f'{motor_rows}'
                     f'<tr style="border-top:2px solid var(--bd)"><td style="font-weight:700">Spread</td>'
                     f'<td colspan="2" style="text-align:center">{sp_a:.1f}%</td>'
                     f'<td colspan="2" style="text-align:center">{sp_b:.1f}%</td>'
                     f'<td style="color:{sp_color}">{sp_d:+.1f}%</td></tr>'
                     f'</table></div></section>')

    return f"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>INAV Flight Comparison</title><style>
:root{{--bg:#0f1117;--cd:#1a1b26;--ca:#1e2030;--bd:#2a2d3e;--tx:#c0caf5;--dm:#7982a9;--bl:#7aa2f7;--gn:#9ece6a;--rd:#f7768e;--yl:#e0af68;--tl:#4ecdc4;--pp:#bb9af7;--or:#ff9e64}}
*{{box-sizing:border-box;margin:0;padding:0}}body{{font-family:'SF Mono','Cascadia Code','JetBrains Mono',monospace;background:var(--bg);color:var(--tx);line-height:1.6}}
.ct{{max-width:1200px;margin:0 auto;padding:24px}}header{{background:linear-gradient(135deg,#1a1b26,#24283b);border-bottom:2px solid var(--pp);padding:24px 0;text-align:center}}
header h1{{font-size:1.5rem;letter-spacing:3px;text-transform:uppercase;color:var(--pp)}}.mt{{color:var(--dm);font-size:.8rem;margin-top:8px}}
section{{margin:32px 0}}h2{{font-size:1rem;color:var(--bl);letter-spacing:2px;text-transform:uppercase;border-bottom:1px solid var(--bd);padding-bottom:8px;margin-bottom:16px}}
.cc{{background:var(--cd);border:1px solid var(--bd);border-radius:8px;padding:16px;margin:12px 0;overflow-x:auto}}
table{{width:100%;border-collapse:collapse;margin:8px 0}}th{{background:var(--ca);color:var(--bl);font-size:.7rem;letter-spacing:1px;text-transform:uppercase;padding:8px 14px;text-align:left;border-bottom:1px solid var(--bd)}}
td{{padding:7px 14px;border-bottom:1px solid var(--bd);font-size:.85rem}}
.delta-up{{color:var(--gn);font-weight:700}}.delta-down{{color:var(--rd);font-weight:700}}.delta-same{{color:var(--dm)}}
footer{{text-align:center;color:var(--dm);font-size:.7rem;padding:24px 0;border-top:1px solid var(--bd);margin-top:40px}}
</style></head><body>
<header><h1>▲ INAV Flight Comparison</h1>
<div class="mt">{label_a} vs {label_b} | {datetime.now().strftime('%Y-%m-%d %H:%M')}</div></header>
<div class="ct">
{warnings_html}
{arrow_html}
<section><h2>Score Comparison</h2><div class="cc">
<table>
<tr><th>Metric</th><th>{label_a}</th><th>{label_b}</th><th>Change</th></tr>
<tr><td style="font-weight:700">Overall</td>{_score_cell(sa.get("overall"))}{_score_cell(sb.get("overall"), True)}<td>{_delta_html("overall")}</td></tr>
<tr><td>Noise</td>{_score_cell(sa.get("noise"))}{_score_cell(sb.get("noise"), True)}<td>{_delta_html("noise")}</td></tr>
<tr><td>PID Response</td>{_score_cell(sa.get("pid"))}{_score_cell(sb.get("pid"), True)}<td>{_delta_html("pid")}</td></tr>
<tr><td>Motor Balance</td>{_score_cell(sa.get("motor"))}{_score_cell(sb.get("motor"), True)}<td>{_delta_html("motor")}</td></tr>
</table></div></section>

<section><h2>Verdict</h2><div class="cc">
<table>
<tr><th></th><th>{label_a}</th><th>{label_b}</th></tr>
<tr><td>Verdict</td><td style="color:var(--dm)">{res_a["plan"].get("verdict_text","?")}</td>
<td style="color:var(--bl);font-weight:600">{res_b["plan"].get("verdict_text","?")}</td></tr>
<tr><td>Duration</td><td>{res_a["data"]["time_s"][-1]:.1f}s</td><td>{res_b["data"]["time_s"][-1]:.1f}s</td></tr>
<tr><td>Actions Remaining</td><td>{len([a for a in res_a["plan"]["actions"] if not a.get("deferred")])}</td>
<td>{len([a for a in res_b["plan"]["actions"] if not a.get("deferred")])}</td></tr>
</table></div></section>

{'<section><h2>Config Changes</h2><div class="cc"><table><tr><th>Setting</th><th>'+label_a+'</th><th>'+label_b+'</th></tr>'+config_rows+'</table></div></section>' if config_rows else ''}

{motor_html}
<section><h2>Noise Spectrum Overlay</h2><div class="cc">{ci("noise")}</div></section>
<section><h2>PID Response Comparison</h2><div class="cc">{ci("pid")}</div></section>

</div><footer>INAV Blackbox Analyzer v{REPORT_VERSION} - Comparison Report</footer></body></html>"""


def _run_comparison(file_a, file_b, args, config_raw):
    """Run comparative analysis on two flight logs."""
    R, B, C, G, Y, RED, DIM = _colors()

    print(f"\n  ▲ {t('banner.compare')} v{REPORT_VERSION}")
    print(f"  Flight A: {file_a}")
    print(f"  Flight B: {file_b}")
    print()

    print(f"  Analyzing flight A...", end=" ", flush=True)
    res_a = _analyze_for_compare(file_a, args, config_raw)
    sa = res_a["plan"]["scores"]
    print(f"score {sa['overall']:.0f}/100")

    print(f"  Analyzing flight B...", end=" ", flush=True)
    res_b = _analyze_for_compare(file_b, args, config_raw)
    sb = res_b["plan"]["scores"]
    print(f"score {sb['overall']:.0f}/100")

    label_a = os.path.splitext(os.path.basename(file_a))[0]
    label_b = os.path.splitext(os.path.basename(file_b))[0]

    # Warn if different craft or frame sizes
    craft_a = res_a["config"].get("craft_name", "")
    craft_b = res_b["config"].get("craft_name", "")
    if craft_a and craft_b and craft_a != craft_b:
        print(f"  {Y}⚠ {t('compare.diff_craft', a=craft_a, b=craft_b)}{R}")
        print(f"  {DIM}  {t('compare.diff_craft_note')}{R}")
    profile_a = res_a["profile"]["name"]
    profile_b = res_b["profile"]["name"]
    if profile_a != profile_b:
        print(f"  {Y}⚠ {t('compare.diff_profile', a=profile_a, b=profile_b)}{R}")
        print(f"  {DIM}  {t('compare.diff_profile_note')}{R}")

    # Terminal comparison
    print(f"\n  {B}{'═' * 60}{R}")
    print(f"  {B}{t('section.comparison').upper()}: {label_a} → {label_b}{R}")
    print(f"  {B}{'═' * 60}{R}")

    metrics = [
        (t("section.overall"), sa.get("overall"), sb.get("overall")),
        (t("section.noise"), sa.get("noise"), sb.get("noise")),
        ("PID", sa.get("pid"), sb.get("pid")),
        (t("label.motor"), sa.get("motor"), sb.get("motor")),
    ]

    print(f"\n  {t('compare.metric'):<16} {'A':>8} {'B':>8} {'Δ':>8}")
    print(f"  {'─' * 44}")

    for name, va, vb in metrics:
        va_str = f"{va:.0f}" if va is not None else "-"
        vb_str = f"{vb:.0f}" if vb is not None else "-"
        if va is not None and vb is not None:
            d = vb - va
            if abs(d) < 0.5:
                delta_str = f"{DIM}→ 0{R}"
            elif d > 0:
                delta_str = f"{G}+{d:.0f}{R}"
            else:
                delta_str = f"{RED}{d:.0f}{R}"
        else:
            delta_str = "-"
        print(f"  {name:<16} {va_str:>8} {vb_str:>8} {delta_str}")

    # PID response comparison
    print(f"\n  {B}{t('section.pid')}:{R}")
    print(f"  {t('label.axis'):<8} {t('label.delay')+' A':>10} {t('label.delay')+' B':>10} {'OS A':>8} {'OS B':>8}")
    print(f"  {'─' * 50}")
    for i in range(3):
        pa = res_a["pid_results"][i]
        pb = res_b["pid_results"][i]
        da = f"{pa['tracking_delay_ms']:.1f}ms" if pa and pa["tracking_delay_ms"] is not None else "-"
        db = f"{pb['tracking_delay_ms']:.1f}ms" if pb and pb["tracking_delay_ms"] is not None else "-"
        oa = f"{pa['avg_overshoot_pct']:.1f}%" if pa and pa["avg_overshoot_pct"] is not None else "-"
        ob = f"{pb['avg_overshoot_pct']:.1f}%" if pb and pb["avg_overshoot_pct"] is not None else "-"
        print(f"  {AXIS_NAMES[i]:<8} {da:>10} {db:>10} {oa:>8} {ob:>8}")

    # Motor balance comparison
    ma_a = res_a["motor_analysis"]
    ma_b = res_b["motor_analysis"]
    if ma_a and ma_b and not ma_a.get("idle_detected") and not ma_b.get("idle_detected"):
        print(f"\n  {B}{t('section.motor_balance')}:{R}")
        print(f"  {'':12} {t('compare.spread')+' A':>10} {t('compare.spread')+' B':>10} {'Δ':>8}")
        print(f"  {'─' * 44}")
        sp_a = ma_a.get("balance_spread_pct", 0)
        sp_b = ma_b.get("balance_spread_pct", 0)
        d = sp_b - sp_a
        dc = G if d < -0.5 else RED if d > 0.5 else DIM
        print(f"  {t('compare.spread'):<12} {sp_a:>9.1f}% {sp_b:>9.1f}% {dc}{d:>+.1f}%{R}")

        sat_a = max(m["saturation_pct"] for m in ma_a["motors"])
        sat_b = max(m["saturation_pct"] for m in ma_b["motors"])
        d = sat_b - sat_a
        dc = G if d < -0.5 else RED if d > 0.5 else DIM
        print(f"  {t('compare.peak_sat'):<12} {sat_a:>9.1f}% {sat_b:>9.1f}% {dc}{d:>+.1f}%{R}")

    overall_delta = (sb.get("overall", 0) or 0) - (sa.get("overall", 0) or 0)
    print(f"\n  {B}{'═' * 60}{R}")
    if overall_delta > 5:
        print(f"  {G}▲ {t('compare.improved_by', delta=f'{overall_delta:.0f}')}{R}")
    elif overall_delta < -5:
        print(f"  {RED}▼ {t('compare.degraded_by', delta=f'{abs(overall_delta):.0f}')}{R}")
    else:
        print(f"  {Y}→ {t('compare.score_same')}{R}")

    # Generate HTML report
    if not args.no_html:
        print(f"\n  {t('compare.generating')}")
        charts = {}
        charts["noise"] = _create_comparison_noise_chart(
            res_a["noise_results"], res_b["noise_results"], label_a, label_b)
        charts["pid"] = _create_comparison_pid_chart(
            res_a["pid_results"], res_b["pid_results"],
            res_a["sr"], res_b["sr"], label_a, label_b)

        html = _generate_comparison_html(res_a, res_b, charts, label_a, label_b)
        on = args.output or f"{label_a}_vs_{label_b}_compare.html"
        op = os.path.join(os.path.dirname(file_a) or ".", on)
        with open(op, "w", encoding="utf-8") as f:
            f.write(html)
        print(f"  ✓ {t('report.html_saved', path=op)}")

    print()


# ─── Replay Mode ─────────────────────────────────────────────────────────────

def _downsample(arr, target_points=5000):
    """Downsample array to target number of points using decimation."""
    if len(arr) <= target_points:
        return arr.tolist() if hasattr(arr, 'tolist') else list(arr)
    step = max(1, len(arr) // target_points)
    return arr[::step].tolist() if hasattr(arr, 'tolist') else list(arr[::step])


def _compute_spectrogram(gyro, sr, nperseg=256, noverlap=None):
    """Compute spectrogram for noise heatmap over time.

    Returns: (times, freqs, power_db) where power_db is a 2D array [freq x time].
    """
    if noverlap is None:
        noverlap = nperseg // 2

    step = nperseg - noverlap
    n_segments = max(1, (len(gyro) - nperseg) // step + 1)

    # Limit segments for HTML size
    max_segments = 400
    if n_segments > max_segments:
        step = max(1, (len(gyro) - nperseg) // max_segments)
        n_segments = min(max_segments, (len(gyro) - nperseg) // step + 1)

    freqs = np.fft.rfftfreq(nperseg, 1.0 / sr)
    # Only keep up to 500Hz
    freq_mask = freqs <= 500
    freqs = freqs[freq_mask]

    window = np.hanning(nperseg)
    power = np.zeros((len(freqs), n_segments))
    times = np.zeros(n_segments)

    for i in range(n_segments):
        start = i * step
        end = start + nperseg
        if end > len(gyro):
            break
        segment = gyro[start:end] * window
        fft_vals = np.fft.rfft(segment)
        psd = np.abs(fft_vals[:len(freqs)]) ** 2
        psd[psd < 1e-12] = 1e-12
        power[:, i] = 10 * np.log10(psd)
        times[i] = (start + nperseg / 2) / sr

    return times.tolist(), freqs.tolist(), power.tolist()


def _extract_flight_modes(data, sr):
    """Extract flight mode transitions from slow frames.

    Returns list of {start_s, end_s, modes} for the flight mode overlay bar.
    INAV stores flight modes as a bitmask in slow frames.
    """
    slow_frames = data.get("_slow_frames", [])
    if not slow_frames:
        return []

    # INAV flight mode IDs (from src/main/fc/runtime_config.h)
    MODE_NAMES = {
        0: "ARM", 1: "ANGLE", 2: "HORIZON", 3: "NAV ALTHOLD",
        4: "HEADING HOLD", 5: "HEADFREE", 6: "HEAD ADJ",
        7: "NAV RTH", 8: "NAV POSHOLD", 9: "MANUAL",
        10: "BEEPER", 11: "NAV LAUNCH",
        12: "OSD SW", 28: "NAV CRUISE",
        29: "NAV COURSE HOLD", 45: "ANGLE HOLD",
    }

    n_rows = data.get("n_rows", len(data.get("time_s", [])))
    transitions = []

    for frame_idx, fields in slow_frames:
        # Look for flightModeFlags in the slow frame
        mode_flags = fields.get("flightModeFlags", None)
        if mode_flags is None:
            # Try alternative field names
            for k in fields:
                if "flight" in k.lower() and "mode" in k.lower():
                    mode_flags = fields[k]
                    break

        if mode_flags is not None:
            # Decode bitmask
            try:
                flags = int(mode_flags)
            except (ValueError, TypeError):
                continue

            active_modes = []
            for bit, name in MODE_NAMES.items():
                if flags & (1 << bit):
                    active_modes.append(name)

            t_s = frame_idx / sr if sr > 0 else 0
            if frame_idx < n_rows:
                t_s = data["time_s"][min(frame_idx, len(data["time_s"]) - 1)]

            transitions.append({
                "time_s": float(t_s),
                "modes": active_modes,
                "flags": flags,
            })

    if not transitions:
        return []

    # Convert transitions to segments
    segments = []
    for i, tr in enumerate(transitions):
        end_t = transitions[i + 1]["time_s"] if i + 1 < len(transitions) else data["time_s"][-1]
        label = ", ".join(tr["modes"]) if tr["modes"] else "DISARMED"
        segments.append({
            "start_s": round(tr["time_s"], 3),
            "end_s": round(end_t, 3),
            "label": label,
        })

    return segments


def _generate_replay_html(config, data, sr, noise_results=None):
    """Generate interactive HTML replay with Plotly.js time-series,
    spectrogram waterfall, synced axes, and flight mode overlay.
    """
    craft = config.get("craft_name", "Unknown")
    fw = config.get("firmware_revision", "")
    duration = data["time_s"][-1]

    # Prepare data series — keep more points for Plotly WebGL
    max_pts = 20000
    time_ds = _downsample(data["time_s"], max_pts)
    step = max(1, len(data["time_s"]) // max_pts)

    # Gyro and setpoint
    gyro_data = {}
    sp_data = {}
    for ax in ["roll", "pitch", "yaw"]:
        gk = f"gyro_{ax}"
        sk = f"setpoint_{ax}"
        if gk in data:
            gyro_data[ax] = _downsample(data[gk], max_pts)
        if sk in data:
            sp_data[ax] = _downsample(data[sk], max_pts)

    # Motors
    motor_data = {}
    for i in range(8):
        mk = f"motor{i}"
        if mk in data:
            raw = data[mk]
            if hasattr(raw, '__len__') and len(raw) > 0:
                raw_max = np.max(raw)
                if raw_max > 100:
                    raw_min = np.min(raw[raw > 0]) if np.any(raw > 0) else 0
                    rng = raw_max - raw_min if raw_max > raw_min else 1
                    motor_data[f"M{i}"] = _downsample(
                        (raw - raw_min) / rng * 100, max_pts)
                else:
                    motor_data[f"M{i}"] = _downsample(raw, max_pts)

    # Throttle
    throttle = None
    if "throttle" in data:
        throttle = _downsample(data["throttle"], max_pts)

    # Spectrogram waterfall — compute on roll gyro (most representative)
    spectrogram = None
    gyro_key = None
    for ax in ["roll", "pitch"]:
        if f"gyro_{ax}" in data:
            gyro_key = f"gyro_{ax}"
            break

    if gyro_key is not None:
        raw_gyro = data[gyro_key]
        if len(raw_gyro) > 512:
            spec_times, spec_freqs, spec_power = _compute_spectrogram(
                raw_gyro, sr, nperseg=256)
            spectrogram = {
                "times": spec_times,
                "freqs": spec_freqs,
                "power": spec_power,
            }

    # Flight mode overlay
    flight_modes = _extract_flight_modes(data, sr)

    import json as _json
    payload = _json.dumps({
        "time": time_ds,
        "gyro": gyro_data,
        "setpoint": sp_data,
        "motors": motor_data,
        "throttle": throttle,
        "spectrogram": spectrogram,
        "flight_modes": flight_modes,
        "duration": duration,
    }, separators=(',', ':'))

    axis_colors = {"roll": "#FF6B6B", "pitch": "#4ECDC4", "yaw": "#FFD93D"}
    motor_colors = ["#FF6B6B", "#4ECDC4", "#FFD93D", "#A78BFA",
                    "#FF9E64", "#9ECE6A", "#7AA2F7", "#BB9AF7"]

    # Mode colors for the overlay bar
    mode_colors = {
        "ARM": "#3b4261", "ANGLE": "#7aa2f7", "HORIZON": "#bb9af7",
        "NAV ALTHOLD": "#9ece6a", "NAV RTH": "#f7768e",
        "NAV POSHOLD": "#4ecdc4", "MANUAL": "#565f89",
        "NAV CRUISE": "#e0af68", "NAV COURSE HOLD": "#e0af68",
        "NAV LAUNCH": "#ff9e64", "HEADING HOLD": "#73daca",
    }

    return f"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>INAV Flight Replay - {craft}</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.27.1/plotly.min.js"></script>
<style>
:root{{--bg:#0f1117;--cd:#1a1b26;--ca:#1e2030;--bd:#2a2d3e;--tx:#c0caf5;--dm:#7982a9;--bl:#7aa2f7}}
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:'SF Mono','Cascadia Code',monospace;background:var(--bg);color:var(--tx);line-height:1.6}}
.ct{{max-width:1400px;margin:0 auto;padding:16px}}
header{{background:linear-gradient(135deg,#1a1b26,#24283b);border-bottom:2px solid var(--bl);padding:20px 0;text-align:center}}
header h1{{font-size:1.3rem;letter-spacing:3px;text-transform:uppercase;color:var(--bl)}}
.mt{{color:var(--dm);font-size:.75rem;margin-top:6px}}
.panel{{background:var(--cd);border:1px solid var(--bd);border-radius:8px;padding:8px;margin:8px 0}}
.panel h3{{font-size:.8rem;color:var(--bl);letter-spacing:2px;text-transform:uppercase;margin:4px 8px}}
.mode-bar{{display:flex;height:24px;border-radius:4px;overflow:hidden;margin:8px 0;background:var(--ca);border:1px solid var(--bd)}}
.mode-seg{{display:flex;align-items:center;justify-content:center;font-size:.6rem;font-weight:600;letter-spacing:.5px;overflow:hidden;white-space:nowrap;text-overflow:ellipsis;color:#fff;opacity:0.9}}
.mode-legend{{display:flex;flex-wrap:wrap;gap:6px;margin:4px 0;font-size:.65rem}}
.mode-legend span{{padding:2px 6px;border-radius:3px;opacity:0.9}}
footer{{text-align:center;color:var(--dm);font-size:.7rem;padding:16px 0;border-top:1px solid var(--bd);margin-top:24px}}
</style></head><body>
<header><h1>▲ INAV Flight Replay</h1>
<div class="mt">{craft} | {fw} | {duration:.1f}s | {sr:.0f}Hz | {datetime.now().strftime('%Y-%m-%d %H:%M')}</div></header>
<div class="ct">

<div class="panel"><h3>Flight Modes</h3><div id="modeBar" class="mode-bar"></div><div id="modeLegend" class="mode-legend"></div></div>

<div class="panel"><h3>Gyro vs Setpoint — Roll</h3><div id="plotRoll" style="height:220px"></div></div>
<div class="panel"><h3>Gyro vs Setpoint — Pitch</h3><div id="plotPitch" style="height:220px"></div></div>
<div class="panel"><h3>Gyro vs Setpoint — Yaw</h3><div id="plotYaw" style="height:220px"></div></div>
<div class="panel"><h3>Motor Output (%)</h3><div id="plotMotors" style="height:220px"></div></div>
{'<div class="panel"><h3>Noise Spectrogram (Waterfall)</h3><div id="plotSpectro" style="height:280px"></div></div>' if spectrogram else ''}
{'<div class="panel"><h3>Throttle</h3><div id="plotThrottle" style="height:160px"></div></div>' if throttle else ''}

</div>
<footer>INAV Blackbox Analyzer v{REPORT_VERSION} - Interactive Replay (zoom any panel, all sync)</footer>

<script>
const D = {payload};
const COLORS = {_json.dumps(axis_colors)};
const MCOLORS = {_json.dumps(motor_colors)};
const MODE_COLORS = {_json.dumps(mode_colors)};

const LAYOUT_BASE = {{
    paper_bgcolor: '#1a1b26', plot_bgcolor: '#1a1b26',
    font: {{ family: "'SF Mono','Cascadia Code',monospace", size: 10, color: '#7982a9' }},
    margin: {{ l: 50, r: 16, t: 8, b: 32 }},
    xaxis: {{ color: '#565f89', gridcolor: '#24283b', zeroline: false,
              range: [0, Math.min(5, D.duration)] }},
    yaxis: {{ color: '#565f89', gridcolor: '#24283b', zeroline: false }},
    legend: {{ font: {{ size: 9 }}, bgcolor: 'rgba(26,27,38,0.8)', bordercolor: '#2a2d3e', borderwidth: 1,
               x: 1, xanchor: 'right', y: 1, yanchor: 'top' }},
    hovermode: 'x unified',
}};

const allPlotIds = [];

function makePlot(divId, traces, yLabel, extra) {{
    const layout = JSON.parse(JSON.stringify(LAYOUT_BASE));
    layout.yaxis.title = {{ text: yLabel, font: {{ size: 10 }} }};
    if (extra) Object.assign(layout, extra);
    Plotly.newPlot(divId, traces, layout, {{
        responsive: true,
        displayModeBar: true,
        modeBarButtonsToRemove: ['sendDataToCloud','lasso2d','select2d'],
        displaylogo: false,
    }});
    allPlotIds.push(divId);
}}

// ── Gyro/Setpoint panels ──
['roll','pitch','yaw'].forEach(ax => {{
    const traces = [];
    if (D.setpoint[ax]) traces.push({{
        x: D.time, y: D.setpoint[ax], type: 'scattergl', mode: 'lines',
        name: 'Setpoint', line: {{ color: '#565f89', width: 1.5 }}
    }});
    if (D.gyro[ax]) traces.push({{
        x: D.time, y: D.gyro[ax], type: 'scattergl', mode: 'lines',
        name: 'Gyro', line: {{ color: COLORS[ax], width: 1.2 }}
    }});
    if (traces.length) makePlot('plot'+ax.charAt(0).toUpperCase()+ax.slice(1), traces, 'deg/s');
}});

// ── Motor panel ──
const mTraces = Object.entries(D.motors).map(([name,vals],i) => ({{
    x: D.time, y: vals, type: 'scattergl', mode: 'lines',
    name: name, line: {{ color: MCOLORS[i%MCOLORS.length], width: 1 }}
}}));
if (mTraces.length) makePlot('plotMotors', mTraces, '%');

// ── Spectrogram waterfall ──
if (D.spectrogram && document.getElementById('plotSpectro')) {{
    const spec = D.spectrogram;
    const trace = {{
        x: spec.times, y: spec.freqs, z: spec.power,
        type: 'heatmap',
        colorscale: [
            [0, '#0f1117'], [0.15, '#1a1b26'], [0.3, '#24283b'],
            [0.45, '#7aa2f7'], [0.6, '#4ecdc4'], [0.75, '#e0af68'],
            [0.9, '#ff9e64'], [1.0, '#f7768e']
        ],
        colorbar: {{ title: 'dB', titlefont: {{ size: 9 }}, tickfont: {{ size: 8 }},
                     len: 0.8, thickness: 12 }},
        hovertemplate: '%{{x:.2f}}s<br>%{{y:.0f}}Hz<br>%{{z:.1f}}dB<extra></extra>',
    }};
    const layout = JSON.parse(JSON.stringify(LAYOUT_BASE));
    layout.yaxis.title = {{ text: 'Frequency (Hz)', font: {{ size: 10 }} }};
    layout.yaxis.range = [0, 500];
    Plotly.newPlot('plotSpectro', [trace], layout, {{
        responsive: true, displayModeBar: true, displaylogo: false,
        modeBarButtonsToRemove: ['sendDataToCloud','lasso2d','select2d'],
    }});
    allPlotIds.push('plotSpectro');
}}

// ── Throttle panel ──
if (D.throttle && document.getElementById('plotThrottle')) {{
    makePlot('plotThrottle', [{{
        x: D.time, y: D.throttle, type: 'scattergl', mode: 'lines',
        name: 'Throttle', line: {{ color: '#7aa2f7', width: 1.2 }},
        fill: 'tozeroy', fillcolor: 'rgba(122,162,247,0.08)',
    }}], 'µs');
}}

// ── Synced x-axis across all panels ──
allPlotIds.forEach(srcId => {{
    document.getElementById(srcId).on('plotly_relayout', function(ed) {{
        if (ed['xaxis.range[0]'] !== undefined && ed['xaxis.range[1]'] !== undefined) {{
            const xRange = [ed['xaxis.range[0]'], ed['xaxis.range[1]']];
            allPlotIds.forEach(tgtId => {{
                if (tgtId !== srcId) {{
                    Plotly.relayout(tgtId, {{ 'xaxis.range': xRange }});
                }}
            }});
        }}
        if (ed['xaxis.autorange']) {{
            allPlotIds.forEach(tgtId => {{
                if (tgtId !== srcId) {{
                    Plotly.relayout(tgtId, {{ 'xaxis.autorange': true }});
                }}
            }});
        }}
    }});
}});

// ── Flight mode overlay bar ──
(function() {{
    const bar = document.getElementById('modeBar');
    const legend = document.getElementById('modeLegend');
    if (!D.flight_modes || !D.flight_modes.length) {{
        bar.innerHTML = '<div class="mode-seg" style="flex:1;background:var(--ca);color:var(--dm)">No flight mode data (S-frames)</div>';
        return;
    }}
    const total = D.duration || 1;
    const seen = new Set();
    D.flight_modes.forEach(seg => {{
        const pct = ((seg.end_s - seg.start_s) / total * 100).toFixed(2);
        if (parseFloat(pct) < 0.3) return;
        // Pick color based on most significant mode
        const modes = seg.label.split(', ');
        let color = '#3b4261';
        for (const m of modes) {{
            if (MODE_COLORS[m]) {{ color = MODE_COLORS[m]; break; }}
        }}
        const el = document.createElement('div');
        el.className = 'mode-seg';
        el.style.flex = '0 0 ' + pct + '%';
        el.style.background = color;
        el.title = seg.label + ' (' + seg.start_s.toFixed(1) + 's - ' + seg.end_s.toFixed(1) + 's)';
        // Only show label if segment is wide enough
        if (parseFloat(pct) > 5) el.textContent = modes[modes.length-1];
        bar.appendChild(el);
        modes.forEach(m => seen.add(m));
    }});
    // Legend
    seen.forEach(m => {{
        const s = document.createElement('span');
        s.style.background = MODE_COLORS[m] || '#3b4261';
        s.style.color = '#fff';
        s.textContent = m;
        legend.appendChild(s);
    }});
}})();
</script></body></html>"""


def _run_replay(logfile, args, config_raw):
    """Generate interactive replay HTML for a single flight."""
    R, B, C, G, Y, RED, DIM = _colors()

    print(f"\n  ▲ {t('banner.replay')} v{REPORT_VERSION}")
    print(f"  Loading: {logfile}")

    raw_params = parse_headers_from_bbl(logfile)
    config = extract_fc_config(raw_params)
    if config_raw:
        merge_diff_into_config(config, config_raw)

    ext = os.path.splitext(logfile)[1].lower()
    is_blackbox = ext in (".bbl", ".bfl", ".bbs")
    if is_blackbox:
        data = decode_blackbox_native(logfile, raw_params, quiet=False)
    else:
        data = parse_csv_log(logfile)

    sr = data["sample_rate"]
    craft = config.get("craft_name", "Unknown")
    duration = data['time_s'][-1]
    print(f"  {data['n_rows']:,} rows | {sr:.0f}Hz | {duration:.1f}s")

    # Show data availability
    parts = []
    n_slow = len(data.get("_slow_frames", []))
    if n_slow:
        parts.append(f"{n_slow} flight mode frames")
    for ax in ["roll", "pitch", "yaw"]:
        if f"gyro_{ax}" in data:
            parts.append("gyro")
            break
    motor_count = sum(1 for i in range(8) if f"motor{i}" in data)
    if motor_count:
        parts.append(f"{motor_count} motors")
    if parts:
        print(f"  Data: {', '.join(parts)}")

    # Quick noise analysis for spectrogram reference
    print(f"  Computing spectrogram...", end=" ", flush=True)
    noise_results = [analyze_noise(data, ax, f"gyro_{ax.lower()}", sr) for ax in AXIS_NAMES]
    print("done")

    print(f"  Generating Plotly.js replay (WebGL)...")
    html = _generate_replay_html(config, data, sr, noise_results)

    on = args.output or os.path.splitext(os.path.basename(logfile))[0] + "_replay.html"
    op = os.path.join(os.path.dirname(logfile) or ".", on)
    with open(op, "w", encoding="utf-8") as f:
        f.write(html)

    size_kb = os.path.getsize(op) / 1024
    print(f"\n  ✓ Replay: {op} ({size_kb:.0f}KB)")
    print(f"    Panels: gyro×3, motors, spectrogram waterfall, throttle")
    print(f"    Features: synced zoom/pan, flight mode overlay, WebGL rendering")
    print()


def _analyze_single_log(logfile, args, config_raw=None, summary_only=False):
    """Analyze a single blackbox log file.
    
    Args:
        logfile: Path to log file
        args: Command line arguments
        config_raw: Optional CLI diff text
        summary_only: If True, skip verbose output/reports but still analyze
                      and store in DB. Returns a summary dict.
    
    Returns:
        dict with summary info if summary_only=True, else None
    """

    # ── Parse headers FIRST (needed for auto-detection) ──
    raw_params = {}
    ext = os.path.splitext(logfile)[1].lower()

    is_blackbox = ext in (".bbl", ".bfl", ".bbs")
    if ext in (".txt", ".TXT") and not is_blackbox:
        try:
            with open(logfile, "rb") as f:
                first_line = f.readline().decode("utf-8", errors="ignore").strip()
                if first_line.startswith("H Product:Blackbox") or first_line.startswith("H Field I name:"):
                    is_blackbox = True
        except:
            pass

    if is_blackbox:
        raw_params = parse_headers_from_bbl(logfile)
    else:
        try:
            with open(logfile, "r", errors="ignore") as f:
                for line in f:
                    if line.strip().startswith("H "):
                        raw_params = parse_headers_from_bbl(logfile)
                        break
                    elif line.strip() and not line.strip().startswith("#"):
                        break
        except:
            pass

    config = extract_fc_config(raw_params)

    # ── Merge CLI diff if available ──
    if config_raw:
        n_merged = merge_diff_into_config(config, config_raw)
        mismatches = config.get("_diff_mismatches", [])
        if n_merged > 0 or mismatches:
            parts = [f"{n_merged} new settings from CLI diff"]
            if mismatches:
                parts.append(f"{len(mismatches)} changed since flight")
            print(f"  Merged: {', '.join(parts)}")

    # ── Auto-detect platform from field names ──
    field_names_str = raw_params.get("Field I name", "")
    if field_names_str:
        all_fields = [f.strip().lower() for f in field_names_str.split(",")]
        motor_fields = [f for f in all_fields if re.match(r"motor\[\d+\]", f)]
        servo_fields = [f for f in all_fields if re.match(r"servo\[\d+\]", f)]
        n_motors = len(motor_fields)
        n_servos = len(servo_fields)
    else:
        n_motors = 4  # assume quad
        n_servos = 0

    # Determine platform type
    if n_motors == 3 or (n_motors == 4 and n_servos >= 1):
        platform_type = "Tricopter"
    elif n_motors == 4 and n_servos == 0:
        platform_type = "Quadcopter"
    elif n_motors == 6:
        platform_type = "Hexacopter"
    elif n_motors == 8:
        platform_type = "Octocopter"
    else:
        platform_type = f"{n_motors}-motor"

    # ── Auto-detect frame size from craft_name ──
    craft = config.get("craft_name", "")
    detected_frame = None
    if craft:
        # Look for number that could be frame/prop size (e.g., "NAZGUL 10", "Mark4 7", "Source One V5")
        # Match patterns like "10", "7", "5" that are likely inches, not version numbers
        m = re.search(r'\b(\d{1,2})(?:\s*(?:inch|in|"|' + r"'" + r'))?(?:\s|$)', craft, re.I)
        if m:
            candidate = int(m.group(1))
            if 3 <= candidate <= 15:  # plausible frame size range
                detected_frame = candidate

    frame_inches = args.frame
    prop_inches = args.props
    n_blades = args.blades

    # If only one of frame/props specified, use it for both
    if frame_inches and not prop_inches:
        prop_inches = frame_inches
    elif prop_inches and not frame_inches:
        frame_inches = prop_inches

    # Auto-detect logic
    frame_source = "user"
    if frame_inches is None and detected_frame is not None:
        frame_inches = detected_frame
        prop_inches = prop_inches or detected_frame
        frame_source = "auto"
    elif frame_inches is not None and detected_frame is not None and frame_inches != detected_frame:
        frame_source = "conflict"

    profile = get_frame_profile(frame_inches, prop_inches, n_blades)

    # ── Estimate battery from vbatref ──
    vbatref = raw_params.get("vbatref", "")
    detected_cells = args.cells
    if not detected_cells and vbatref:
        try:
            vref_v = int(vbatref) / 100.0
            if vref_v > 0:
                detected_cells = round(vref_v / 4.2)
                if detected_cells < 1 or detected_cells > 14:
                    detected_cells = None
        except:
            pass

    # ── Show comprehensive banner ──
    nav_mode = getattr(args, 'nav', False)  # kept for backward compat, ignored
    if not summary_only:
        print(f"\n  ▲ {t('banner.analyzer')} v{REPORT_VERSION}")
        print(f"  {t('banner.loading', file=logfile)}")

        fw_rev = config.get("firmware_revision", "")
        fw_date = config.get("firmware_date", "")

        # Aircraft identification banner
        print(f"\n  {'─'*66}")
        if craft:
            print(f"  {t('banner.craft')}:  {craft}")
        if fw_rev:
            fw_str = fw_rev
            if fw_date:
                fw_str += f" ({fw_date})"
            print(f"  {t('banner.firmware')}:  {fw_str}")
        print(f"  {t('banner.platform')}:  {platform_type} ({n_motors} motors{f', {n_servos} servos' if n_servos else ''})")

        frame_str = f"{frame_inches}\"" if frame_inches else "5\" (default)"
        prop_str = f"{prop_inches}\"×{n_blades}-blade" if prop_inches else f"5\"×{n_blades}-blade (default)"
        if frame_source == "auto":
            frame_str += f" (detected from craft name)"
        elif frame_source == "conflict":
            frame_str += f" (user override)"
        print(f"  {t('banner.frame')}:     {frame_str}")
        print(f"  {t('banner.props')}:     {prop_str}")

        if detected_cells:
            cell_str = f"{detected_cells}S"
            if not args.cells and vbatref:
                cell_str += f" (detected from vbatref={int(vbatref)/100:.1f}V)"
            print(f"  {t('banner.battery')}:   {cell_str}")

        if args.kv:
            print(f"  Motors:    {args.kv}KV")

        # Profile and thresholds
        print(f"  Profile:   {profile['name']} ({profile['class']} class)")
        if (frame_inches or 5) >= 8:
            print(f"    Delay: <{profile['ok_delay_ms']}ms | OS: <{profile['ok_overshoot']}% | "
                  f"Filters: {profile['gyro_lpf_range'][0]}-{profile['gyro_lpf_range'][1]}Hz")

            # Warnings
            if frame_source == "conflict":
                print(f"\n  Warning: FRAME SIZE CONFLICT: Craft name \"{craft}\" suggests {detected_frame}\" "
                      f"but --frame {args.frame} was specified.")
                print(f"    Using {args.frame}\" as requested. If this is wrong, the analyzer will use "
                      f"incorrect thresholds.")
            elif frame_source != "auto" and frame_inches is None:
                if detected_frame is None and craft:
                    print(f"\n  Warning: Could not detect frame size from craft name \"{craft}\".")
                    print(f"    Using 5\" defaults. Specify --frame N for accurate thresholds.")
                elif not craft:
                    print(f"\n  Warning: No craft name in log headers. Using 5\" defaults.")
                    print(f"    Specify --frame N for accurate thresholds.")

        print(f"  {'─'*66}")

    # ── Check field readiness ──
    if raw_params and not summary_only:
        field_check = check_blackbox_readiness_from_headers(raw_params, nav_mode=False)
        if not field_check["ok"]:
            _print_readiness(field_check, source="headers")
        elif field_check["items"]:
            R2, B2, C2, G2, Y2, RED2, DIM2 = _colors()
            for item in field_check["items"]:
                if item["level"] == "OK":
                    print(f"  {G2}✓ {item['text']}{R2}")
                elif item["level"] == "INFO":
                    print(f"  {DIM2}ℹ {item['text']}{R2}")

    # ── Decode data ──
    if is_blackbox:
        if not summary_only:
            print(f"\n  {t('terminal.parsing_headers', n=len(raw_params))}")
            print(f"  {t('terminal.decoding')}")
        data = decode_blackbox_native(logfile, raw_params, quiet=summary_only)
    else:
        if not summary_only:
            print(f"\n  {t('terminal.parsing_csv')}")
        data = parse_csv_log(logfile)

    sr = data["sample_rate"]
    if not summary_only:
        print(f"  {data['n_rows']:,} rows | {sr:.0f}Hz | {data['time_s'][-1]:.1f}s")

        # Automatic log quality check
        quality = assess_log_quality(data, config, logfile)
        if quality["grade"] == "UNUSABLE":
            print_log_quality(quality)
            print(f"  Log is not usable for analysis. Run with --check-log for details.")
            dur = float(data["time_s"][-1]) if "time_s" in data and len(data["time_s"]) > 0 else 0
            return {"verdict": "UNUSABLE", "score": 0, "duration": dur} if summary_only else None
        elif quality["grade"] == "MARGINAL":
            R2, B2, C2, G2, Y2, RED2, DIM2 = _colors()
            print(f"  {Y2}⚠ Log quality: MARGINAL{R2} — ", end="")
            warn_msgs = [i["message"] for i in quality["issues"]]
            print(f"{'; '.join(warn_msgs[:2])}{R2}")
    else:
        quality = assess_log_quality(data, config, logfile)
        if not quality["usable"]:
            dur = float(data["time_s"][-1]) if "time_s" in data and len(data["time_s"]) > 0 else 0
            return {"verdict": "UNUSABLE", "score": 0, "duration": dur,
                    "idle": False, "config_key": ""}

    if not summary_only:
        # Show nav field summary if nav data is present
        avail_check = detect_nav_fields(data)
        if avail_check.get("has_any"):
            nav_fields = [k for k in data if k.startswith("nav_") or k.startswith("att_") or k == "baro_alt"]
            gps_count = len(data.get("_gps_frames", []))
            slow_count = len(data.get("_slow_frames", []))
            parts = []
            if nav_fields:
                parts.append(f"{len(nav_fields)} nav fields")
            if gps_count:
                parts.append(f"{gps_count} GPS frames")
            if slow_count:
                parts.append(f"{slow_count} slow frames")
            if parts:
                print(f"  Nav data: {', '.join(parts)}")

    if not summary_only:
        if config_has_pid(config):
            for ax in ["roll","pitch","yaw"]:
                ff = config.get(f'{ax}_ff', '')
                ff_str = f" FF={ff}" if ff else ""
                print(f"  {ax.capitalize()} PID: P={config.get(f'{ax}_p','?')} I={config.get(f'{ax}_i','?')} D={config.get(f'{ax}_d','?')}{ff_str}")
        else:
            print(f"  {t('terminal.no_pid_values')}")

    if not summary_only and config_has_filters(config):
        filt_parts = []
        for k, l in [("gyro_lowpass_hz","Gyro LPF"),("dterm_lpf_hz","D-term LPF"),("yaw_lpf_hz","Yaw LPF")]:
            v = config.get(k)
            if v is not None and v != 0:
                filt_parts.append(f"{l}={v}Hz")
        dyn_en = config.get("dyn_notch_enabled")
        if dyn_en and dyn_en not in (0, "0", "OFF"):
            dyn_min = config.get("dyn_notch_min_hz", "?")
            dyn_q = config.get("dyn_notch_q", "?")
            filt_parts.append(f"DynNotch=ON(min={dyn_min}Hz,Q={dyn_q})")
        rpm_en = config.get("rpm_filter_enabled")
        if rpm_en and rpm_en not in (0, "0", "OFF"):
            filt_parts.append("RPM=ON")
        elif rpm_en is not None:
            filt_parts.append("RPM=OFF")
        if filt_parts:
            print(f"  Filters: {', '.join(filt_parts)}")

    if not summary_only and config.get("_diff_merged"):
        diff_extras = []
        if config.get("motor_poles"):
            diff_extras.append(f"MotorPoles={config['motor_poles']}")
        if config.get("motor_idle") is not None:
            diff_extras.append(f"Idle={config['motor_idle']}")
        if config.get("antigravity_gain") is not None:
            diff_extras.append(f"AntiGrav={config['antigravity_gain']}")
        if config.get("nav_alt_p") is not None:
            diff_extras.append(f"NavAltP={config['nav_alt_p']}")
        if config.get("level_p") is not None:
            diff_extras.append(f"LevelP={config['level_p']}")
        if diff_extras:
            print(f"  Diff extras: {', '.join(diff_extras)}")
        print(f"  {config['_diff_settings_count']} settings from CLI diff "
              f"({config.get('_diff_unmapped_count', 0)} additional stored)")

    # ── RPM prediction (tuning mode only) ──
    rpm_range = None
    prop_harmonics = None
    phase_lag = None
    rpm_range = estimate_rpm_range(args.kv, detected_cells or args.cells)
    if rpm_range:
        cells_used = detected_cells or args.cells
        if not summary_only:
            print(f"  RPM estimate: {rpm_range[0]:,}-{rpm_range[1]:,} ({args.kv}KV x {cells_used}S)")
        prop_harmonics = estimate_prop_harmonics(rpm_range, n_blades)
        if not summary_only:
            for h in prop_harmonics:
                print(f"    {h['label']}: {h['min_hz']:.0f}-{h['max_hz']:.0f} Hz ({n_blades}-blade)")

    # ── Phase lag estimation ──
    if config_has_filters(config):
        # Estimate at a frequency relevant to this frame class
        sig_freq = (profile["noise_band_mid"][0] + profile["noise_band_mid"][1]) / 4
        phase_lag = estimate_total_phase_lag(config, profile, sig_freq)
        if not summary_only and phase_lag["total_degrees"] > 20:
            print(f"  Filter phase lag: {phase_lag['total_degrees']:.0f}deg ({phase_lag['total_ms']:.1f}ms) at {sig_freq:.0f}Hz")

    if not summary_only:
        print("  Analyzing...")

    # ── Always run core analysis (PID, noise, motors) ──
    hover_osc = detect_hover_oscillation(data, sr, profile)
    noise_results = [analyze_noise(data, ax, f"gyro_{ax.lower()}", sr) for ax in AXIS_NAMES]
    noise_fp = fingerprint_noise(noise_results, config, prop_harmonics)
    pid_results = [analyze_pid_response(data, i, sr) for i in range(3)]
    motor_analysis = analyze_motors(data, sr, config)
    dterm_results = analyze_dterm_noise(data, sr)

    # ── Accelerometer vibration analysis ──
    accel_vib = analyze_accel_vibration(data, sr, prop_harmonics)
    if not summary_only and accel_vib and accel_vib.get("axes"):
        vib_score = accel_vib.get("score", 100)
        if vib_score < 85:
            print(f"  Vibration: {vib_score}/100 ({accel_vib['overall_rms_g']:.2f}g RMS)")

    # ── Motor response time ──
    motor_response = analyze_motor_response(data, sr)
    if not summary_only and motor_response and motor_response["motor_response_ms"] > 1:
        print(f"  Motor response: {motor_response['motor_response_ms']:.1f}ms")

    plan = generate_action_plan(noise_results, pid_results, motor_analysis, dterm_results,
                                 config, data, profile, phase_lag, motor_response,
                                 rpm_range, prop_harmonics, hover_osc)
    plan["noise_fingerprint"] = noise_fp

    # Enrich filter action reasons with noise source identification
    if noise_fp and noise_fp["peaks"] and noise_fp["dominant_source"] != "clean":
        fp_summary = noise_fp["summary"]
        for action in plan["actions"]:
            if action.get("category") == "Filter" and "noise" in action.get("reason", "").lower():
                action["reason"] += f". Sources: {fp_summary}"

    # ── Tuning recipe ──
    recipe = generate_tuning_recipe(noise_results, noise_fp, config, profile, accel_vib)
    if not summary_only:
        print(f"  Recipe: {recipe['recipe_name']}")

    # ── Power/efficiency analysis ──
    power_results = analyze_power(data, sr, config)

    # ── Propwash scoring ──
    propwash_results = analyze_propwash(data, sr, profile)

    # ── Failsafe event reconstruction ──
    failsafe_results = analyze_failsafe_events(data, sr)

    # ── Auto-detect and run nav analysis if fields are present ──
    nav_perf = None
    nav_results = None
    no_nav = getattr(args, 'no_nav', False)
    avail = detect_nav_fields(data)

    if avail.get("has_any") and not no_nav:
        # Nav controller performance analysis (decel overshoot, poshold, althold)
        if avail.get("has_pos") and avail.get("has_vel"):
            nav_perf = analyze_nav_performance(data, sr, config, profile)
            if not summary_only and nav_perf and nav_perf.get("findings"):
                n_findings = len(nav_perf["findings"])
                nav_score = nav_perf.get("score", 0)
                print(f"  Nav performance: {nav_score}/100 ({n_findings} findings)")

        # Nav sensor health analysis (compass, GPS, baro, estimator)
        nav_results = run_nav_analysis(data, sr, config)

    # ── Summary-only mode: store in DB and return summary ──
    if summary_only:
        summary = {
            "score": plan["scores"]["overall"],
            "duration": data["time_s"][-1] if "time_s" in data and len(data["time_s"]) > 0 else 0,
            "idle": motor_analysis.get("idle_detected", False) if motor_analysis else False,
            "verdict": plan.get("verdict", ""),
            "verdict_short": _verdict_short(plan.get("verdict", "")),
            "n_actions": len(plan.get("actions", [])),
            "config_key": _config_fingerprint(config),
            "logfile": logfile,
        }
        # Still store in DB
        if not args.no_db:
            try:
                try:
                    from inav_toolkit.flight_db import FlightDB
                except ImportError:
                    from inav_flight_db import FlightDB
                db = FlightDB(args.db_path)
                flight_id, is_new = db.store_flight(
                    plan, config, data, hover_osc, motor_analysis,
                    pid_results, noise_results, log_file=logfile, config_raw=config_raw)
                db.close()
                summary["flight_id"] = flight_id
                summary["is_new"] = is_new
            except Exception:
                pass
        return summary

    # ── Determine if interactive mode ──
    interactive = (sys.stdout.isatty() and hasattr(args, 'device') and args.device
                   and not args.no_terminal and plan["verdict"] != "GROUND_ONLY")

    # ── Capture config review text (needed for both modes) ──
    config_review_text = ""
    if config_raw and plan["verdict"] != "GROUND_ONLY":
        config_review_text = _capture(_print_config_review, config_raw, config, frame_inches, plan)

    if not interactive:
        # Sequential output (piped, no device, or --no-terminal)
        if not args.no_terminal:
            print_terminal_report(plan, noise_results, pid_results, motor_analysis, config, data,
                                  show_narrative=not args.no_narrative, profile=profile, noise_fp=noise_fp)
            # Nav sections (auto-detected)
            if nav_perf and nav_perf.get("findings"):
                _print_section_nav(nav_perf)
            if nav_results:
                _print_section_nav_sensors(nav_results)
            if accel_vib and accel_vib.get("axes"):
                _print_section_vibration(accel_vib)
            if propwash_results and propwash_results.get("events"):
                _print_section_propwash(propwash_results)
            if power_results and power_results.get("avg_cell_v") is not None:
                _print_section_power(power_results)
            if recipe:
                _print_section_recipe(recipe)
            if failsafe_results and failsafe_results.get("events"):
                R2, B2, C2, G2, Y2, RED2, DIM2 = _colors()
                print(f"\n  {B2}{'─'*66}{R2}")
                print(f"  {B2}FAILSAFE EVENTS:{R2}")
                for f in failsafe_results["findings"]:
                    print(f"  {DIM2}ℹ {f['text']}{R2}")
            if config_review_text:
                print(config_review_text, end="")

    # ── Generate charts (slow, do before DB) ──
    charts = {}
    if not args.no_html and plan["verdict"] != "GROUND_ONLY":
        print("  Generating charts...")
        vn = [n for n in noise_results if n]
        if vn: charts["noise"] = create_noise_chart(vn)
        charts["pid"] = create_pid_response_chart(pid_results, sr)
        if motor_analysis: charts["motor"] = create_motor_chart(motor_analysis, data["time_s"])
        if dterm_results: charts["dterm"] = create_dterm_chart(dterm_results)

    # ── Save state with profile ──
    config["_profile_name"] = profile["name"]
    config["_profile_class"] = profile["class"]
    if frame_inches: config["_frame_inches"] = frame_inches
    if prop_inches: config["_prop_inches"] = prop_inches
    config["_n_blades"] = n_blades
    config["_cell_count"] = detected_cells or args.cells
    if args.kv: config["_motor_kv"] = args.kv
    config["_n_motors"] = n_motors
    config["_platform_type"] = platform_type

    sp = save_state(logfile, plan, config, data)
    print(f"  ✓ {t('report.state_saved', path=sp)}")

    # ── Store in flight database ──
    progression_text = ""
    flight_diff = None
    prog_data = None
    if not args.no_db:
        try:
            try:
                from inav_toolkit.flight_db import FlightDB
            except ImportError:
                from inav_flight_db import FlightDB
            db = FlightDB(args.db_path)
            flight_id, is_new = db.store_flight(
                plan, config, data, hover_osc, motor_analysis,
                pid_results, noise_results, log_file=logfile, config_raw=config_raw)
            craft = config.get("craft_name", "unknown")
            n_flights = db.get_flight_count(craft)
            if is_new:
                print(f"  ✓ Database: flight #{flight_id} for {craft} ({n_flights} total)")
            else:
                print(f"  ✓ Database: already stored as flight #{flight_id} (skipped duplicate)")

            # Capture progression for interactive menu
            if n_flights >= 2:
                prog = db.get_progression(craft)
                prog_data = prog
                if prog["changes"]:
                    R, B, C, G, Y, RED, DIM = _colors()
                    trend_icon = {"improving": f"{G}↗ Improving", "degrading": f"{RED}↘ Degrading",
                                  "stable": f"{Y}→ Stable"}.get(prog["trend"], "")
                    prog_lines = [f"\n  {B}Progression:{R} {trend_icon}{R}"]
                    for ch in prog["changes"]:
                        prog_lines.append(f"    {ch}")
                    progression_text = "\n".join(prog_lines)
                    if not interactive:
                        _print_section_history(prog)
                elif not interactive and len(prog.get("flights", [])) >= 2:
                    _print_section_history(prog)

                # Flight-to-flight diff
                flight_diff = db.get_flight_diff(craft, flight_id)
                if flight_diff and flight_diff.get("has_previous") and not interactive:
                    _print_section_diff(flight_diff)

            db.close()
        except Exception as e:
            print(f"  ⚠ Database: {e}")

    # ── HTML report (assembled after DB so we have history data) ──
    if not args.no_html and plan["verdict"] != "GROUND_ONLY":
        # Generate preflight checklist for report
        pf_data = None
        if config_raw:
            pf_data = preflight_checklist(config_raw, frame_inches=frame_inches)
            if not pf_data.get("items"):
                pf_data = None

        html = generate_html_report(plan, noise_results, pid_results, motor_analysis,
                                     dterm_results, config, data, charts,
                                     nav_results=nav_results, nav_perf=nav_perf,
                                     flight_diff=flight_diff, prog_data=prog_data,
                                     preflight=pf_data, accel_vib=accel_vib,
                                     recipe=recipe, power_results=power_results,
                                     propwash_results=propwash_results,
                                     failsafe_results=failsafe_results)
        on = args.output or os.path.splitext(os.path.basename(logfile))[0] + "_report.html"
        op = os.path.join(os.path.dirname(logfile) or ".", on)
        with open(op, "w", encoding="utf-8") as f: f.write(html)
        print(f"\n  ✓ {t('report.html_saved', path=op)}")

        # Auto-open in browser
        if not getattr(args, 'no_browser', False):
            try:
                import webbrowser
                webbrowser.open(f"file://{os.path.abspath(op)}")
            except Exception:
                pass

    # ── Markdown report ──
    if getattr(args, 'report', None) in ('md', 'markdown') and not summary_only:
        md = generate_markdown_report(plan, config, data, noise_results, pid_results,
                                      motor_analysis, profile, quality)
        md_name = os.path.splitext(os.path.basename(logfile))[0] + "_report.md"
        md_path = os.path.join(os.path.dirname(logfile) or ".", md_name)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(md)
        print(f"  ✓ {t('report.markdown_saved', path=md_path)}")

    # ── Interactive menu (device mode only) ──
    if interactive:
        print()
        _interactive_menu(plan, pid_results, noise_fp, motor_analysis, config, data,
                          profile, config_review_text, progression_text,
                          nav_perf=nav_perf, nav_results=nav_results,
                          flight_diff=flight_diff, prog_data=prog_data,
                          accel_vib=accel_vib)

        # ── Auto-apply offer (device mode only) ──
        if hasattr(args, 'device') and args.device:
            active_actions = [a for a in plan["actions"] if not a.get("deferred")]
            cli_cmds = generate_cli_commands(active_actions)
            # Add nav PID commands
            if nav_perf and nav_perf.get("nav_actions"):
                for a in nav_perf["nav_actions"]:
                    cli_cmds.insert(-1, a["cli"])  # before 'save'
            if cli_cmds and len(cli_cmds) > 1:  # more than just 'save'
                _offer_auto_apply(cli_cmds, args)
    else:
        print()

if __name__ == "__main__":
    main()
