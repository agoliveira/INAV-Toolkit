#!/usr/bin/env python3
"""
INAV Parameter Analyzer - Analyze `diff all` output for configuration issues.

Parses INAV CLI `diff all` output and checks for:
  - Safety issues (beepers, failsafe, voltage limits)
  - Performance issues (filters, PID settings, motor protocol)
  - Missing recommended settings for the hardware profile
  - Cross-reference with blackbox analysis results (optional)
  - Navigation configuration review

Usage:
    inav-params diff_all.txt [--frame 10] [--blackbox state.json]
"""

import argparse
import json
import os
import re
import sys
import textwrap
from datetime import datetime

VERSION = "2.15.1"


def _enable_ansi_colors():
    """Enable ANSI color support. Returns True if colors are available."""
    if os.environ.get("NO_COLOR") is not None:
        return False
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetStdHandle(-11)
            mode = ctypes.c_ulong()
            kernel32.GetConsoleMode(handle, ctypes.byref(mode))
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
            return True
        except Exception:
            return False
    return True

_ANSI_ENABLED = _enable_ansi_colors()

def _colors():
    """Return (R, B, C, G, Y, RED, DIM) color codes."""
    if _ANSI_ENABLED:
        return ("\033[0m", "\033[1m", "\033[96m", "\033[92m",
                "\033[93m", "\033[91m", "\033[2m")
    return ("", "", "", "", "", "", "")

def _disable_colors():
    global _ANSI_ENABLED
    _ANSI_ENABLED = False

# ─── Severity Levels ─────────────────────────────────────────────────────────

CRITICAL = "CRITICAL"      # Safety risk, could crash or lose the quad
WARNING = "WARNING"        # Likely performance issue or suboptimal config
INFO = "INFO"              # Suggestion, nice-to-have
OK = "OK"                  # Checked and looks good

SEVERITY_ORDER = {CRITICAL: 0, WARNING: 1, INFO: 2, OK: 3}

# ─── INAV Aux Mode IDs (from src/main/fc/rc_modes.h) ────────────────────────

INAV_MODES = {
    0: "ARM", 1: "ANGLE", 2: "HORIZON", 3: "NAV ALTHOLD",
    5: "HEADING HOLD", 10: "NAV POSHOLD", 11: "NAV RTH",
    12: "MANUAL", 13: "NAV WP", 28: "NAV LAUNCH", 45: "TURTLE",
    47: "OSD ALT", 48: "NAV COURSE HOLD", 53: "MULTI FUNCTION",
    62: "MIXER PROFILE 2", 63: "MIXER TRANSITION",
}

# Modes that require GPS
GPS_MODES = {10, 11, 13, 48}  # POSHOLD, RTH, WP, COURSE HOLD
# Modes that require baro or althold
ALT_MODES = {3}  # NAV ALTHOLD
# Modes that benefit from compass
COMPASS_MODES = {10, 11, 13, 5}  # POSHOLD, RTH, WP, HEADING HOLD

# Multirotor platform types
MC_PLATFORMS = {"MULTIROTOR", "TRICOPTER"}

# ─── Known Defaults & Limits ─────────────────────────────────────────────────

# INAV 9.x defaults for multirotor (applied_defaults=5 is "multirotor with GPS")
INAV9_MC_DEFAULTS = {
    "looptime": 500,           # 2kHz
    "gyro_main_lpf_hz": 110,
    "gyro_main_lpf_type": "PT1",
    "dterm_lpf_hz": 110,
    "dterm_lpf_type": "PT1",
    "dynamic_gyro_notch_q": 250,
    "dynamic_gyro_notch_min_hz": 80,
    "mc_p_pitch": 44, "mc_i_pitch": 75, "mc_d_pitch": 25,
    "mc_p_roll": 40, "mc_i_roll": 75, "mc_d_roll": 25,
    "mc_p_yaw": 85, "mc_i_yaw": 45, "mc_d_yaw": 0,
    "mc_iterm_relax": "RP",
    "d_boost_min": 1.0, "d_boost_max": 1.0,
    "antigravity_gain": 1.0,
    "motor_pwm_protocol": "DSHOT300",
    "blackbox_rate_denom": 1,
    "failsafe_procedure": "DROP",
    "nav_rth_altitude": 5000,
    "nav_mc_hover_thr": 1500,
}

# Motor pole counts for common motors
COMMON_MOTOR_POLES = {14: "most standard motors", 12: "some smaller motors"}

# ─── Frame-Specific Starting Profiles ────────────────────────────────────────
# Conservative baselines for large INAV multirotors.
# These are intentionally soft - designed to be flyable on first hover,
# then refined with the blackbox analyzer. Too low is sluggish, too high
# oscillates. We err on the sluggish side.
#
# Sources: community experience, INAV defaults scaling, physics of prop inertia.
# Prop inertia scales roughly with diameter^4, so a 10" prop has ~4x the inertia
# of a 7". PID gains must scale down accordingly.

FRAME_PROFILES = {
    # ── 5-inch baseline ─────────────────────────────────────────────────
    # Sources:
    #   - INAV 9 firmware defaults (settings.yaml / pid.c)
    #   - INAV configurator 5" preset (GitHub issue #167)
    #   - INAV Settings.md: dterm_lpf "100 should work best with 5-inch"
    #   - INAV Settings.md: dyn_notch_min "default 150 works best with 5\""
    5: {
        "name": "5-inch racer / freestyle",
        "description": "Standard FPV size. 2306-2207 motors, 4S-6S.",
        "typical_auw": "350-700g",
        "typical_motors": "2207, 2306, 2405",
        "pids": {
            "mc_p_pitch": 44, "mc_i_pitch": 75, "mc_d_pitch": 28,
            "mc_cd_pitch": 60,
            "mc_p_roll": 40, "mc_i_roll": 60, "mc_d_roll": 26,
            "mc_cd_roll": 60,
            "mc_p_yaw": 45, "mc_i_yaw": 80, "mc_d_yaw": 0,
        },
        "filters": {
            "gyro_main_lpf_hz": 110,
            "dterm_lpf_hz": 110,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_min_hz": 150,
            "dynamic_gyro_notch_q": 250,
            "dynamic_gyro_notch_mode": "3D",
        },
        "rates": {
            "roll_rate": 70,
            "pitch_rate": 70,
            "yaw_rate": 60,
        },
        "other": {
            "mc_iterm_relax": "RPY",
            "mc_iterm_relax_cutoff": 15,
            "d_boost_min": 0.8,
            "d_boost_max": 1.2,
            "antigravity_gain": 2.0,
            "antigravity_accelerator": 5.0,
            "tpa_rate": 20,
            "tpa_breakpoint": 1350,
            "rate_accel_limit_roll_pitch": 0,
            "rate_accel_limit_yaw": 0,
        },
        "notes": [
            "Close to INAV 9 defaults. Fine starting point for 5-inch builds.",
            "Tune from here using blackbox - most 5-inch quads converge quickly.",
        ],
    },
    # ── 7-inch ──────────────────────────────────────────────────────────
    # Sources:
    #   - INAV Settings.md: dterm_lpf "80 seems like a gold spot for 7-inch"
    #   - INAV Settings.md: dyn_notch_min "values around 100 work fine on 7\""
    #   - Pawel Spychalski's 7" builds (INAV core dev, community baseline)
    #   - INAV defaults designed for 5-7", minimal changes needed at 7"
    7: {
        "name": "7-inch long-range",
        "description": "Common for long-range builds. 2807-3007 motors, 4S-6S.",
        "typical_auw": "800-1400g",
        "typical_motors": "2807, 2907, 3007",
        "pids": {
            "mc_p_pitch": 44, "mc_i_pitch": 75, "mc_d_pitch": 25,
            "mc_cd_pitch": 60,
            "mc_p_roll": 40, "mc_i_roll": 60, "mc_d_roll": 23,
            "mc_cd_roll": 60,
            "mc_p_yaw": 45, "mc_i_yaw": 80, "mc_d_yaw": 0,
        },
        "filters": {
            "gyro_main_lpf_hz": 90,
            "dterm_lpf_hz": 80,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_min_hz": 100,
            "dynamic_gyro_notch_q": 250,
            "dynamic_gyro_notch_mode": "3D",
        },
        "rates": {
            "roll_rate": 50,
            "pitch_rate": 50,
            "yaw_rate": 40,
        },
        "other": {
            "mc_iterm_relax": "RPY",
            "mc_iterm_relax_cutoff": 12,
            "d_boost_min": 0.85,
            "d_boost_max": 1.15,
            "antigravity_gain": 2.0,
            "antigravity_accelerator": 5.0,
            "tpa_rate": 20,
            "tpa_breakpoint": 1350,
            "rate_accel_limit_roll_pitch": 0,
            "rate_accel_limit_yaw": 0,
        },
        "notes": [
            "INAV defaults are designed for 5-7\". Only minor changes needed.",
            "Dterm LPF at 80Hz per INAV dev guidance. Dynamic notch min at 100Hz.",
            "If using triblades, reduce P by 10-15% - they grip harder and respond faster.",
        ],
    },
    # ── 10-inch ─────────────────────────────────────────────────────────
    # Sources:
    #   - INAV Settings.md: dyn_notch_min "60-70" for 10", dterm_lpf ~60-70Hz
    #   - GitHub #9765 (Ivan): starting PIDs 55/75/35 for 7"+, filters -10Hz from 7"
    #   - IntoFPV forum: P:D ratio ~1:1, iterm_relax_cutoff <10 (5 recommended)
    #   - Community consensus: frame stiffness 7.5mm arm min, RPM filters critical
    # Conservative starting point biased toward safe first flight.
    10: {
        "name": "10-inch long-range / cruiser",
        "description": "Long-range with large Li-ion packs. 3507-4010 motors, 4S-6S.",
        "typical_auw": "1500-2800g",
        "typical_motors": "3507, 3508, 4008, 4010",
        "pids": {
            "mc_p_pitch": 40, "mc_i_pitch": 60, "mc_d_pitch": 30,
            "mc_cd_pitch": 40,
            "mc_p_roll": 38, "mc_i_roll": 55, "mc_d_roll": 28,
            "mc_cd_roll": 40,
            "mc_p_yaw": 40, "mc_i_yaw": 60, "mc_d_yaw": 0,
        },
        "filters": {
            "gyro_main_lpf_hz": 70,
            "dterm_lpf_hz": 60,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_min_hz": 65,
            "dynamic_gyro_notch_q": 250,
            "dynamic_gyro_notch_mode": "3D",
        },
        "rates": {
            "roll_rate": 36,
            "pitch_rate": 36,
            "yaw_rate": 30,
        },
        "other": {
            "mc_iterm_relax": "RPY",
            "mc_iterm_relax_cutoff": 8,
            "d_boost_min": 0.80,
            "d_boost_max": 1.20,
            "antigravity_gain": 2.0,
            "antigravity_accelerator": 5.0,
            "tpa_rate": 20,
            "tpa_breakpoint": 1250,
            "rate_accel_limit_roll_pitch": 1000,
            "rate_accel_limit_yaw": 500,
        },
        "notes": [
            "P:D ratio close to 1:1 on 10\" (unlike 5\" where P >> D).",
            "Lower iterm_relax_cutoff (8) prevents bounce-back on this heavier airframe.",
            "Rate acceleration limits prevent sudden moves that stress the frame.",
            "Frame stiffness is critical - 7.5mm carbon arms minimum.",
            "If heavy (>2.5kg), reduce P by 10% and raise I by 10% for better position hold.",
            "EZ Tune is an alternative: set ez_filter_hz ~70, lower response/stability.",
        ],
    },
    # ── 12-inch ─────────────────────────────────────────────────────────
    # Sources:
    #   - Extrapolated from 10" community data + physics-based scaling
    #   - GitHub #9765: 16" Tarot flew P=44/I=45/D=36, gyro=50, dterm=40
    #   - Noise frequencies shift down proportionally with prop diameter
    #   - INAV Settings.md: accel limits ~360-500 dps^2 for heavy multirotors
    12: {
        "name": "12-inch heavy lifter",
        "description": "Heavy lift, cine, long endurance. 4510-5010 motors, 6S-12S.",
        "typical_auw": "2500-5000g",
        "typical_motors": "4510, 4515, 5010, 5015",
        "pids": {
            "mc_p_pitch": 32, "mc_i_pitch": 50, "mc_d_pitch": 25,
            "mc_cd_pitch": 30,
            "mc_p_roll": 30, "mc_i_roll": 45, "mc_d_roll": 23,
            "mc_cd_roll": 30,
            "mc_p_yaw": 35, "mc_i_yaw": 50, "mc_d_yaw": 0,
        },
        "filters": {
            "gyro_main_lpf_hz": 55,
            "dterm_lpf_hz": 45,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_min_hz": 45,
            "dynamic_gyro_notch_q": 250,
            "dynamic_gyro_notch_mode": "3D",
        },
        "rates": {
            "roll_rate": 28,
            "pitch_rate": 28,
            "yaw_rate": 22,
        },
        "other": {
            "mc_iterm_relax": "RPY",
            "mc_iterm_relax_cutoff": 5,
            "d_boost_min": 0.75,
            "d_boost_max": 1.25,
            "antigravity_gain": 2.5,
            "antigravity_accelerator": 5.0,
            "tpa_rate": 15,
            "tpa_breakpoint": 1300,
            "rate_accel_limit_roll_pitch": 500,
            "rate_accel_limit_yaw": 250,
        },
        "notes": [
            "Large prop inertia makes overcorrection the main risk, not sluggishness.",
            "Aerodynamic damping from large props reduces the need for D-term.",
            "Higher antigravity helps maintain altitude during throttle changes with heavy payloads.",
            "Acceleration limits are essential - sudden rate demands can flex the frame.",
            "FC vibration isolation is critical. Use soft-mount or TPU dampers.",
            "EZ Tune alternative: set ez_filter_hz ~55, lower response/stability.",
        ],
    },
    # ── 15-inch ─────────────────────────────────────────────────────────
    # Sources:
    #   - GitHub #9765 (Ivan): gyro filters ~50Hz on 16", PIDs 44/45/36 (tuned)
    #   - INAV Settings.md: accel limits 360/180 dps^2 for big heavy multirotors
    #   - Physics: prop inertia ~10x that of 7", motor noise 60-120Hz range
    #   - Conservative starting point - 16" Tarot flew P=44 but was well-tuned
    15: {
        "name": "15-inch heavy lift / cine lifter",
        "description": "Maximum endurance and payload. 5515-7015 motors, 6S-12S.",
        "typical_auw": "4000-10000g",
        "typical_motors": "5515, 6010, 6015, 7015",
        "pids": {
            "mc_p_pitch": 28, "mc_i_pitch": 45, "mc_d_pitch": 22,
            "mc_cd_pitch": 20,
            "mc_p_roll": 26, "mc_i_roll": 40, "mc_d_roll": 20,
            "mc_cd_roll": 20,
            "mc_p_yaw": 30, "mc_i_yaw": 40, "mc_d_yaw": 0,
        },
        "filters": {
            "gyro_main_lpf_hz": 45,
            "dterm_lpf_hz": 35,
            "dterm_lpf_type": "PT3",
            "dynamic_gyro_notch_min_hz": 30,
            "dynamic_gyro_notch_q": 250,
            "dynamic_gyro_notch_mode": "3D",
        },
        "rates": {
            "roll_rate": 20,
            "pitch_rate": 20,
            "yaw_rate": 18,
        },
        "other": {
            "mc_iterm_relax": "RPY",
            "mc_iterm_relax_cutoff": 5,
            "d_boost_min": 0.70,
            "d_boost_max": 1.30,
            "antigravity_gain": 3.0,
            "antigravity_accelerator": 5.0,
            "tpa_rate": 10,
            "tpa_breakpoint": 1300,
            "rate_accel_limit_roll_pitch": 360,
            "rate_accel_limit_yaw": 180,
        },
        "notes": [
            "Prop inertia is enormous. Overcorrection is the main risk, not sluggishness.",
            "Test in calm conditions first. Wind amplifies any PID issues on large frames.",
            "Accel limits (360/180 dps^2) per INAV dev guidance for big heavy multirotors.",
            "FC vibration isolation is critical. Use soft-mount or TPU dampers.",
            "If oscillation persists, lower P before lowering D - unlike smaller quads.",
            "Community data: tuned 16\" flew P=44 - room to increase once vibrations are controlled.",
            "EZ Tune alternative: set ez_filter_hz ~45, lower response/stability significantly.",
        ],
    },
}

# Voltage-specific adjustments (applied on top of frame profile)
VOLTAGE_ADJUSTMENTS = {
    "4S": {
        "description": "4S (14.8-16.8V)",
        "pid_scale": 1.0,       # baseline
        "notes": "Standard voltage. No PID adjustment needed.",
    },
    "6S": {
        "description": "6S (22.2-25.2V)",
        "pid_scale": 0.85,      # motors are punchier, need less P
        "notes": "Higher voltage means faster motor response. PIDs scaled down ~15%.",
    },
    "8S": {
        "description": "8S (29.6-33.6V)",
        "pid_scale": 0.75,
        "notes": "Very fast motor response. PIDs scaled down ~25%. Use care on first flight.",
    },
    "12S": {
        "description": "12S (44.4-50.4V)",
        "pid_scale": 0.65,
        "notes": "Extreme voltage. Very conservative PIDs recommended. Motors respond almost instantly.",
    },
}


# ─── Setup Mode: Generate Starting Config ────────────────────────────────────

def generate_setup_config(frame_inches, voltage="4S", use_case="longrange"):
    """Generate conservative starting configuration for a given frame size."""
    if frame_inches not in FRAME_PROFILES:
        # Find nearest
        available = sorted(FRAME_PROFILES.keys())
        nearest = min(available, key=lambda x: abs(x - frame_inches))
        print(f"  Warning: No profile for {frame_inches}\". Using nearest: {nearest}\"")
        frame_inches = nearest

    profile = FRAME_PROFILES[frame_inches]
    v_adj = VOLTAGE_ADJUSTMENTS.get(voltage, VOLTAGE_ADJUSTMENTS["4S"])
    scale = v_adj["pid_scale"]

    # Scale PIDs by voltage
    pids = {}
    for k, v in profile["pids"].items():
        if v == 0:
            # Respect explicit zeros (e.g. yaw D = 0 on multirotors)
            pids[k] = 0
        elif k.startswith("mc_p_") or k.startswith("mc_d_") or k.startswith("mc_cd_"):
            pids[k] = max(5, round(v * scale))
        elif k.startswith("mc_i_"):
            # I-term scales less with voltage - it's about steady-state
            pids[k] = max(20, round(v * (1.0 + (scale - 1.0) * 0.3)))
        else:
            pids[k] = v

    config = {
        "frame": frame_inches,
        "voltage": voltage,
        "profile": profile,
        "v_adj": v_adj,
        "pids": pids,
        "filters": dict(profile["filters"]),
        "rates": dict(profile.get("rates", {})),
        "other": dict(profile["other"]),
    }

    return config


def print_setup_report(config):
    """Print the setup configuration as a readable report with CLI commands."""
    R, B, C, G, Y, _RED, DIM = _colors()

    profile = config["profile"]
    frame = config["frame"]
    voltage = config["voltage"]

    print(f"\n{B}{C}{'='*70}{R}")
    print(f"{B}{C}  INAV Starting Configuration - {frame}-inch ({voltage}){R}")
    print(f"{B}{C}{'='*70}{R}")
    print(f"  {DIM}{profile['description']}{R}")
    print(f"  {DIM}Typical AUW: {profile['typical_auw']} | Motors: {profile['typical_motors']}{R}")

    if config["v_adj"]["pid_scale"] != 1.0:
        pct = (1.0 - config["v_adj"]["pid_scale"]) * 100
        print(f"  {Y}Voltage adjustment: {voltage} -> PIDs reduced ~{pct:.0f}%{R}")

    print(f"\n{B}{C}{'-'*70}{R}")
    print(f"  {B}IMPORTANT:{R}")
    print(f"  {DIM}These are CONSERVATIVE starting values - safe for first hover.{R}")
    print(f"  {DIM}The quad may feel sluggish. That's intentional.{R}")
    print(f"  {DIM}Fly, log blackbox data, then use the blackbox analyzer to refine.{R}")
    print(f"{B}{C}{'-'*70}{R}")

    # PID table
    pids = config["pids"]
    has_cd = any(k.startswith("mc_cd_") for k in pids)
    if has_cd:
        print(f"\n  {B}STARTING PIDs:{R}")
        print(f"             {'P':>5}  {'I':>5}  {'D':>5}  {'CD':>5}")
        for axis in ("roll", "pitch", "yaw"):
            p = pids.get(f"mc_p_{axis}", "-")
            i = pids.get(f"mc_i_{axis}", "-")
            d = pids.get(f"mc_d_{axis}", "-")
            cd = pids.get(f"mc_cd_{axis}", "-")
            print(f"    {axis.capitalize():6}   {p:>5}  {i:>5}  {d:>5}  {cd:>5}")
    else:
        print(f"\n  {B}STARTING PIDs:{R}")
        print(f"             {'P':>5}  {'I':>5}  {'D':>5}")
        for axis in ("roll", "pitch", "yaw"):
            p = pids.get(f"mc_p_{axis}", "-")
            i = pids.get(f"mc_i_{axis}", "-")
            d = pids.get(f"mc_d_{axis}", "-")
            print(f"    {axis.capitalize():6}   {p:>5}  {i:>5}  {d:>5}")

    # Filters
    f = config["filters"]
    print(f"\n  {B}FILTERS:{R}")
    print(f"    Gyro LPF:           {f['gyro_main_lpf_hz']}Hz")
    dterm_lpf = f.get("dterm_lpf_hz")
    dterm_type = f.get("dterm_lpf_type", "")
    if dterm_lpf:
        type_str = f" ({dterm_type})" if dterm_type else ""
        print(f"    D-term LPF:         {dterm_lpf}Hz{type_str}")
    print(f"    Dynamic notch:      {f['dynamic_gyro_notch_mode']} (Q={f['dynamic_gyro_notch_q']}, min={f['dynamic_gyro_notch_min_hz']}Hz)")

    # Rates
    rates = config.get("rates", {})
    if rates:
        rr = rates.get("roll_rate", "?")
        pr = rates.get("pitch_rate", "?")
        yr = rates.get("yaw_rate", "?")
        # INAV rates are in deca-degrees/sec, so rate 36 = 360 deg/s
        print(f"\n  {B}RATES:{R}")
        print(f"    Roll:  {rr} ({rr*10} deg/s)  |  Pitch:  {pr} ({pr*10} deg/s)  |  Yaw:  {yr} ({yr*10} deg/s)")

    # Other settings
    print(f"\n  {B}RECOMMENDED SETTINGS:{R}")
    o = config["other"]
    relax_cutoff = o.get("mc_iterm_relax_cutoff")
    cutoff_str = f" (cutoff: {relax_cutoff})" if relax_cutoff else ""
    print(f"    I-term relax:       {o['mc_iterm_relax']}{cutoff_str}")
    print(f"    D-boost:            {o['d_boost_min']:.2f} - {o['d_boost_max']:.2f}")
    print(f"    Antigravity:        {o['antigravity_gain']:.1f} (accel: {o['antigravity_accelerator']:.1f})")
    print(f"    TPA:                {o['tpa_rate']}% above {o['tpa_breakpoint']}")

    accel_rp = o.get("rate_accel_limit_roll_pitch", 0)
    accel_y = o.get("rate_accel_limit_yaw", 0)
    if accel_rp > 0 or accel_y > 0:
        rp_str = f"{accel_rp} dps^2" if accel_rp > 0 else "OFF"
        y_str = f"{accel_y} dps^2" if accel_y > 0 else "OFF"
        print(f"    Accel limits:       Roll/Pitch: {rp_str}  |  Yaw: {y_str}")

    # Notes
    if profile["notes"]:
        print(f"\n  {B}NOTES:{R}")
        for note in profile["notes"]:
            print(f"    {DIM}* {note}{R}")

    if config["v_adj"]["notes"]:
        print(f"    {DIM}* {config['v_adj']['notes']}{R}")

    # CLI commands
    print(f"\n{B}{C}{'-'*70}{R}")
    print(f"  {B}INAV CLI - paste into Configurator CLI tab:{R}")
    print(f"{B}{C}{'-'*70}{R}")
    print()

    all_settings = {}
    all_settings.update(config["pids"])
    all_settings.update(config["filters"])
    all_settings.update(config.get("rates", {}))
    all_settings.update(config["other"])

    for k, v in all_settings.items():
        if isinstance(v, float):
            print(f"    {G}set {k} = {v:.3f}{R}")
        else:
            print(f"    {G}set {k} = {v}{R}")
    print(f"    {G}save{R}")

    print(f"\n{B}{C}{'='*70}{R}\n")


def print_setup_json(config):
    """Output setup config as JSON."""
    output = {
        "frame_inches": config["frame"],
        "voltage": config["voltage"],
        "profile_name": config["profile"]["name"],
        "description": config["profile"]["description"],
        "pids": config["pids"],
        "filters": config["filters"],
        "rates": config.get("rates", {}),
        "other": config["other"],
        "notes": config["profile"]["notes"],
        "voltage_notes": config["v_adj"]["notes"],
    }
    print(json.dumps(output, indent=2))


# ─── Parser ──────────────────────────────────────────────────────────────────

def parse_diff_all(text):
    """Parse INAV `diff all` output into structured data."""
    result = {
        "version": None,
        "board": None,
        "build_date": None,
        "git_hash": None,
        "master": {},           # global settings
        "control_profiles": {},  # {1: {settings}, 2: {settings}, ...}
        "mixer_profiles": {},
        "battery_profiles": {},
        "active_control_profile": 1,
        "active_mixer_profile": 1,
        "active_battery_profile": 1,
        "features": [],
        "features_disabled": [],
        "beepers_disabled": [],
        "beepers_enabled": [],
        "blackbox_enabled": [],
        "blackbox_disabled": [],
        "serial_ports": {},
        "aux_modes": [],
        "motor_mix": [],
        "raw_text": text,
    }

    current_section = "master"
    current_profile_type = None
    current_profile_num = 1

    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            # Check for version comment
            m = re.match(r"#\s*INAV/(\S+)\s+([\d.]+)\s+(.*?)(?:\s*/\s*(.*))?$", line)
            if m:
                result["board"] = m.group(1)
                result["version"] = m.group(2)
                rest = m.group(3)
                # Extract date and git hash
                dm = re.search(r"(\w+ \d+ \d{4})", rest)
                if dm:
                    result["build_date"] = dm.group(1)
                gm = re.search(r"\(([0-9a-f]+)\)", rest)
                if gm:
                    result["git_hash"] = gm.group(1)
            continue

        # Feature lines
        if line.startswith("feature "):
            feat = line[8:].strip()
            if feat.startswith("-"):
                result["features_disabled"].append(feat[1:])
            else:
                result["features"].append(feat)
            continue

        # Beeper lines
        if line.startswith("beeper "):
            beep = line[7:].strip()
            if beep.startswith("-"):
                result["beepers_disabled"].append(beep[1:])
            else:
                result["beepers_enabled"].append(beep)
            continue

        # Blackbox field selection
        if line.startswith("blackbox "):
            bb = line[9:].strip()
            if bb.startswith("-"):
                result["blackbox_disabled"].append(bb[1:])
            else:
                result["blackbox_enabled"].append(bb)
            continue

        # Serial port config
        m = re.match(r"serial\s+(\d+)\s+(.*)", line)
        if m:
            result["serial_ports"][int(m.group(1))] = m.group(2).strip()
            continue

        # Aux mode
        m = re.match(r"aux\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", line)
        if m:
            result["aux_modes"].append({
                "index": int(m.group(1)),
                "mode_id": int(m.group(2)),
                "channel": int(m.group(3)),
                "range_low": int(m.group(4)),
                "range_high": int(m.group(5)),
            })
            continue

        # Motor mix
        m = re.match(r"mmix\s+(\d+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)", line)
        if m:
            result["motor_mix"].append({
                "index": int(m.group(1)),
                "throttle": float(m.group(2)),
                "roll": float(m.group(3)),
                "pitch": float(m.group(4)),
                "yaw": float(m.group(5)),
            })
            continue

        # Profile switches
        m = re.match(r"(control_profile|mixer_profile|battery_profile)\s+(\d+)", line)
        if m:
            current_profile_type = m.group(1)
            current_profile_num = int(m.group(2))
            section_map = {
                "control_profile": "control_profiles",
                "mixer_profile": "mixer_profiles",
                "battery_profile": "battery_profiles",
            }
            current_section = section_map[current_profile_type]
            if current_profile_num not in result[current_section]:
                result[current_section][current_profile_num] = {}
            continue

        # Channel map
        m = re.match(r"map\s+(\w+)", line)
        if m:
            result["master"]["channel_map"] = m.group(1)
            continue

        # Set commands
        m = re.match(r"set\s+(\S+)\s*=\s*(.*)", line)
        if m:
            key = m.group(1).strip()
            val = m.group(2).strip()
            # Try to parse numeric values
            parsed_val = _parse_value(val)

            if current_section == "master":
                result["master"][key] = parsed_val
            else:
                result[current_section][current_profile_num][key] = parsed_val
            continue

        # Active profile restore
        if line.startswith("control_profile") and "restore" not in line:
            m = re.match(r"control_profile\s+(\d+)", line)
            if m:
                result["active_control_profile"] = int(m.group(1))
        if line.startswith("mixer_profile") and "restore" not in line:
            m = re.match(r"mixer_profile\s+(\d+)", line)
            if m:
                result["active_mixer_profile"] = int(m.group(1))
        if line.startswith("battery_profile") and "restore" not in line:
            m = re.match(r"battery_profile\s+(\d+)", line)
            if m:
                result["active_battery_profile"] = int(m.group(1))

    return result


def _parse_value(val):
    """Parse a value string into int, float, or string."""
    if val.upper() in ("ON", "TRUE", "YES"):
        return True
    if val.upper() in ("OFF", "FALSE", "NO"):
        return False
    try:
        return int(val)
    except ValueError:
        pass
    try:
        return float(val)
    except ValueError:
        pass
    return val


def get_active_control(parsed):
    """Get the active control profile settings merged with master."""
    n = parsed["active_control_profile"]
    profile = parsed["control_profiles"].get(n, {})
    return profile


def get_active_battery(parsed):
    """Get the active battery profile settings."""
    n = parsed["active_battery_profile"]
    return parsed["battery_profiles"].get(n, {})


def get_active_mixer(parsed):
    """Get the active mixer profile settings."""
    n = parsed["active_mixer_profile"]
    return parsed["mixer_profiles"].get(n, {})


def get_setting(parsed, key, default=None):
    """Get a setting value, checking active profile first then master."""
    profile = get_active_control(parsed)
    if key in profile:
        return profile[key]
    if key in parsed["master"]:
        return parsed["master"][key]
    return default


# ─── Rule Engine ─────────────────────────────────────────────────────────────

class Finding:
    def __init__(self, severity, category, title, detail, setting=None,
                 current=None, recommended=None, cli_fix=None):
        self.severity = severity
        self.category = category
        self.title = title
        self.detail = detail
        self.setting = setting
        self.current = current
        self.recommended = recommended
        self.cli_fix = cli_fix

    def __repr__(self):
        return f"<{self.severity} {self.category}: {self.title}>"


def run_all_checks(parsed, frame_inches=None, blackbox_state=None):
    """Run all configuration checks and return list of Findings."""
    findings = []

    findings.extend(check_safety(parsed))
    findings.extend(check_motors_protocol(parsed))
    findings.extend(check_filters(parsed, frame_inches))
    findings.extend(check_pid_config(parsed, frame_inches))
    findings.extend(check_navigation(parsed))
    findings.extend(check_gps(parsed))
    findings.extend(check_blackbox(parsed))
    findings.extend(check_battery(parsed))
    findings.extend(check_rx(parsed))
    findings.extend(check_general(parsed))

    if blackbox_state:
        findings.extend(check_crossref_blackbox(parsed, blackbox_state))

    # Sort by severity
    findings.sort(key=lambda f: SEVERITY_ORDER.get(f.severity, 99))
    return findings


# ─── Safety Checks ───────────────────────────────────────────────────────────

def check_safety(parsed):
    findings = []

    # Beeper configuration
    critical_beepers = ["BAT_CRIT_LOW", "BAT_LOW", "RX_LOST", "RX_LOST_LANDING", "HW_FAILURE"]
    disabled = parsed["beepers_disabled"]
    missing_critical = [b for b in critical_beepers if b in disabled]

    if missing_critical:
        findings.append(Finding(
            CRITICAL, "Safety", "Critical beeper warnings disabled",
            f"These beepers are disabled: {', '.join(missing_critical)}. "
            f"You will get no audible warning for low battery, signal loss, or hardware failure. "
            f"This is extremely dangerous - a silent low battery can cause a crash with no warning.",
            setting="beeper",
            current=f"{len(missing_critical)} critical beepers off",
            recommended="Enable at minimum BAT_CRIT_LOW, BAT_LOW, RX_LOST",
            cli_fix="\n".join(f"beeper {b}" for b in missing_critical)))

    all_disabled = len(disabled) > 20  # practically all beepers off
    if all_disabled:
        findings.append(Finding(
            WARNING, "Safety", "Almost all beepers disabled",
            f"{len(disabled)} beepers are disabled. While some are cosmetic (ARMING, SYSTEM_INIT), "
            f"consider enabling at least the safety-related ones.",
            setting="beeper",
            current=f"{len(disabled)} disabled"))

    # Failsafe
    fs_proc = get_setting(parsed, "failsafe_procedure", "DROP")
    if fs_proc == "DROP" or fs_proc == 0:
        findings.append(Finding(
            WARNING, "Safety", "Failsafe set to DROP",
            "On signal loss, the quad will disarm and drop from the sky. "
            "For a GPS-equipped quad, RTH failsafe is much safer.",
            setting="failsafe_procedure",
            current="DROP",
            recommended="RTH",
            cli_fix="set failsafe_procedure = RTH"))
    elif fs_proc == "RTH" or fs_proc == 2:
        findings.append(Finding(
            OK, "Safety", "Failsafe set to RTH",
            "Good - the quad will attempt to return home on signal loss.",
            setting="failsafe_procedure",
            current="RTH"))

    fs_min_dist = get_setting(parsed, "failsafe_min_distance", 0)
    fs_min_proc = get_setting(parsed, "failsafe_min_distance_procedure", "DROP")
    if fs_min_dist > 0 and (fs_min_proc == "LAND" or fs_min_proc == 1):
        findings.append(Finding(
            OK, "Safety", f"Failsafe min distance: {fs_min_dist/100:.0f}m → LAND",
            f"If within {fs_min_dist/100:.0f}m when signal is lost, the quad will land instead of RTH. Good.",
            setting="failsafe_min_distance"))
    elif fs_min_dist == 0 and (fs_proc == "RTH" or fs_proc == 2):
        findings.append(Finding(
            INFO, "Safety", "No failsafe minimum distance set",
            "Consider setting failsafe_min_distance so the quad lands instead of RTH when close to home. "
            "RTH from 5 meters away can be unpredictable.",
            setting="failsafe_min_distance",
            recommended="set failsafe_min_distance = 1000",
            cli_fix="set failsafe_min_distance = 1000\nset failsafe_min_distance_procedure = LAND"))

    # DSHOT beeper as alternative
    dshot_beeper = get_setting(parsed, "dshot_beeper_enabled", True)
    motor_protocol = get_setting(parsed, "motor_pwm_protocol", "")
    if isinstance(motor_protocol, str) and "DSHOT" in motor_protocol.upper():
        if dshot_beeper is False:
            if all_disabled or missing_critical:
                findings.append(Finding(
                    WARNING, "Safety", "DSHOT beeper also disabled",
                    "With physical beeper warnings off AND DSHOT beeper disabled, "
                    "you have no way to find the quad if it lands in tall grass. "
                    "DSHOT beeper uses the motors to beep - it's your last-resort finder.",
                    setting="dshot_beeper_enabled",
                    current="OFF",
                    recommended="ON",
                    cli_fix="set dshot_beeper_enabled = ON"))

    return findings


# ─── Motor & Protocol Checks ────────────────────────────────────────────────

def check_motors_protocol(parsed):
    findings = []
    protocol = get_setting(parsed, "motor_pwm_protocol", "")
    protocol_str = str(protocol).upper()

    is_dshot = "DSHOT" in protocol_str
    if is_dshot:
        findings.append(Finding(
            OK, "Motors", f"Motor protocol: {protocol}",
            "DSHOT digital protocol - good for consistent motor response.",
            setting="motor_pwm_protocol",
            current=str(protocol)))
    else:
        findings.append(Finding(
            WARNING, "Motors", f"Motor protocol: {protocol}",
            "Not using DSHOT. Digital protocols (DSHOT300/600) give more consistent "
            "motor timing and enable features like DSHOT beeper.",
            setting="motor_pwm_protocol",
            current=str(protocol),
            recommended="DSHOT300 or DSHOT600",
            cli_fix="set motor_pwm_protocol = DSHOT300"))

    # RPM filter - INAV uses ESC telemetry wire (not bidirectional DSHOT)
    rpm_filter = get_setting(parsed, "rpm_gyro_filter_enabled", None)

    # Check if ESC telemetry is configured on any serial port
    # Serial function 4096 = ESC telemetry (ESC_SENSOR)
    has_esc_telemetry = any(
        "4096" in conf for conf in parsed["serial_ports"].values()
    )

    if rpm_filter is True and not has_esc_telemetry:
        findings.append(Finding(
            CRITICAL, "Motors", "RPM filter enabled but no ESC telemetry port configured",
            "INAV's RPM filter requires ESC telemetry data via a dedicated wire from the ESC "
            "to a UART RX pin. No serial port is configured for ESC telemetry (function 4096). "
            "Without RPM data, the filter has nothing to work with and may cause instability.",
            setting="rpm_gyro_filter_enabled",
            current="ON",
            recommended="Connect ESC telemetry wire and configure port, or disable RPM filter",
            cli_fix="set rpm_gyro_filter_enabled = OFF"))
    elif rpm_filter is True and has_esc_telemetry:
        findings.append(Finding(
            OK, "Motors", "RPM filter enabled with ESC telemetry",
            "RPM filter is active and ESC telemetry port is configured. "
            "This provides precise motor noise tracking.",
            setting="rpm_gyro_filter_enabled",
            current="ON"))

    # Motor stop on low
    motorstop = get_setting(parsed, "motorstop_on_low", True)
    mixer = get_active_mixer(parsed)
    motorstop = mixer.get("motorstop_on_low", motorstop)
    if motorstop is False:
        findings.append(Finding(
            OK, "Motors", "Motor stop on low throttle: OFF",
            "Motors keep spinning at idle - good for flight stability.",
            setting="motorstop_on_low"))

    # Throttle idle
    battery = get_active_battery(parsed)
    idle = battery.get("throttle_idle", get_setting(parsed, "throttle_idle", 5.0))
    if isinstance(idle, (int, float)):
        if idle < 3.0:
            findings.append(Finding(
                WARNING, "Motors", f"Throttle idle very low: {idle}%",
                "Very low idle may cause desync on some ESCs, especially with high cell count. "
                "5-8% is typical for multirotor.",
                setting="throttle_idle",
                current=f"{idle}%",
                recommended="5.0",
                cli_fix="set throttle_idle = 5.000"))
        elif idle > 10.0:
            findings.append(Finding(
                INFO, "Motors", f"Throttle idle high: {idle}%",
                "High idle wastes battery but can help with prop wash handling. "
                "5-8% is typical.",
                setting="throttle_idle",
                current=f"{idle}%"))

    return findings


# ─── Filter Checks ───────────────────────────────────────────────────────────

def check_filters(parsed, frame_inches=None):
    findings = []

    gyro_lpf = get_setting(parsed, "gyro_main_lpf_hz", 110)
    dyn_notch_q = get_setting(parsed, "dynamic_gyro_notch_q", 250)
    dyn_notch_mode = get_setting(parsed, "dynamic_gyro_notch_mode", "3D")
    dyn_notch_min = get_setting(parsed, "dynamic_gyro_notch_min_hz", 80)
    kalman_q = get_setting(parsed, "setpoint_kalman_q", 100)

    # EZ Tune detection
    # ez_enabled controls whether EZ Tune actively computes PIDs.
    # ez_ parameters may exist in diff even when EZ Tune is disabled (residual from past use).
    profile = get_active_control(parsed)
    ez_enabled = profile.get("ez_enabled", get_setting(parsed, "ez_enabled", None))
    ez_filter = profile.get("ez_filter_hz", get_setting(parsed, "ez_filter_hz", None))

    # EZ Tune is active only if ez_enabled is explicitly ON
    # If ez_enabled is not in the diff, it's at default (OFF)
    using_ez = ez_enabled is True

    if using_ez:
        findings.append(Finding(
            WARNING, "Filters", f"EZ Tune active (ez_filter_hz = {ez_filter})",
            "EZ Tune computes PID and filter values from its own parameters (ez_response, "
            "ez_damping, etc). Manual PID changes via CLI or blackbox analyzer recommendations "
            "will be OVERWRITTEN on reboot. To apply blackbox tuning: either adjust EZ Tune "
            "parameters (ez_response, ez_damping, ez_stability), or disable EZ Tune "
            "and set PIDs manually.",
            setting="ez_enabled",
            current="ON",
            recommended="Disable to use manual PIDs",
            cli_fix="set ez_enabled = OFF"))
    elif ez_filter is not None and ez_enabled is None:
        # ez_ params exist but ez_enabled not in diff (default OFF) - just informational
        findings.append(Finding(
            INFO, "Filters", "EZ Tune parameters present but EZ Tune appears disabled",
            f"Residual EZ Tune parameters found (ez_filter_hz={ez_filter}) but ez_enabled "
            f"is not set (default OFF). These values are not being used. If you want to clean "
            f"up, you can leave them - they have no effect while EZ Tune is disabled.",
            setting="ez_filter_hz",
            current=f"{ez_filter} (inactive)"))

    # Gyro LPF
    if isinstance(gyro_lpf, (int, float)):
        if frame_inches and frame_inches >= 8:
            if gyro_lpf > 80:
                findings.append(Finding(
                    WARNING, "Filters", f"Gyro LPF at {gyro_lpf}Hz - high for {frame_inches}-inch",
                    f"Large props generate noise at lower frequencies. For {frame_inches}-inch, "
                    f"gyro LPF between 40-80Hz is typical. Higher values let more motor noise "
                    f"reach the PID controller.",
                    setting="gyro_main_lpf_hz",
                    current=str(gyro_lpf),
                    recommended="40-80",
                    cli_fix=f"set gyro_main_lpf_hz = 65"))
            else:
                findings.append(Finding(
                    OK, "Filters", f"Gyro LPF at {gyro_lpf}Hz - reasonable for {frame_inches}-inch",
                    "Filter cutoff is in the expected range for this prop size.",
                    setting="gyro_main_lpf_hz",
                    current=str(gyro_lpf)))
        elif frame_inches and frame_inches <= 5:
            if gyro_lpf < 80:
                findings.append(Finding(
                    INFO, "Filters", f"Gyro LPF at {gyro_lpf}Hz - conservative for {frame_inches}-inch",
                    "This is quite low for a small quad. You might be losing responsiveness. "
                    "90-150Hz is typical for 5-inch unless you have vibration issues.",
                    setting="gyro_main_lpf_hz",
                    current=str(gyro_lpf)))

    # Dynamic notch
    if dyn_notch_mode and str(dyn_notch_mode).upper() not in ("OFF", "0", "FALSE"):
        findings.append(Finding(
            OK, "Filters", f"Dynamic notch filter: {dyn_notch_mode} (Q={dyn_notch_q}, min={dyn_notch_min}Hz)",
            "Dynamic notch tracks motor noise peaks in real-time.",
            setting="dynamic_gyro_notch_mode",
            current=f"{dyn_notch_mode}"))

        if isinstance(dyn_notch_min, (int, float)) and frame_inches is not None:
            # INAV docs: 150 for 5", 100 for 7", 60-70 for 10"
            expected_max = {5: 180, 7: 120, 10: 80, 12: 60, 15: 45}
            nearest_size = min(expected_max.keys(), key=lambda x: abs(x - frame_inches))
            max_thresh = expected_max[nearest_size]
            if frame_inches >= 8 and dyn_notch_min > max_thresh:
                rec = max_thresh - 10
                findings.append(Finding(
                    INFO, "Filters", f"Dynamic notch min_hz = {dyn_notch_min}Hz - may miss low-freq noise",
                    f"Large props can produce harmonics below {dyn_notch_min}Hz. "
                    f"INAV docs recommend lower values for {frame_inches}-inch. "
                    f"Consider lowering to {rec}-{max_thresh}Hz.",
                    setting="dynamic_gyro_notch_min_hz",
                    current=str(dyn_notch_min),
                    recommended=f"{rec}-{max_thresh}",
                    cli_fix=f"set dynamic_gyro_notch_min_hz = {rec}"))
    else:
        findings.append(Finding(
            WARNING, "Filters", "Dynamic notch filter is OFF",
            "The dynamic notch filter tracks motor noise harmonics in real-time. "
            "Without it, you rely entirely on the lowpass filter which also adds phase lag. "
            "Strongly recommended for all INAV multirotor builds.",
            setting="dynamic_gyro_notch_mode",
            current="OFF",
            recommended="3D",
            cli_fix="set dynamic_gyro_notch_mode = 3D"))

    # D-term LPF
    dterm_lpf = get_setting(parsed, "dterm_lpf_hz", 110)
    if isinstance(dterm_lpf, (int, float)) and frame_inches:
        # Expected ranges per frame size from INAV dev guidance and community data
        expected = {5: (90, 150), 7: (70, 100), 10: (45, 75), 12: (35, 60), 15: (25, 45)}
        nearest = min(expected.keys(), key=lambda x: abs(x - frame_inches))
        low, high = expected[nearest]
        if dterm_lpf > high + 20:
            findings.append(Finding(
                WARNING, "Filters", f"D-term LPF at {dterm_lpf}Hz - high for {frame_inches}-inch",
                f"Large props generate lower-frequency noise that feeds through D-term. "
                f"For {frame_inches}-inch, {low}-{high}Hz is typical. "
                f"High D-term filtering lets motor noise through and causes hot motors.",
                setting="dterm_lpf_hz",
                current=str(dterm_lpf),
                recommended=f"{low}-{high}",
                cli_fix=f"set dterm_lpf_hz = {(low + high) // 2}"))
        elif dterm_lpf < low - 15:
            findings.append(Finding(
                INFO, "Filters", f"D-term LPF at {dterm_lpf}Hz - conservative for {frame_inches}-inch",
                f"This adds more phase lag than needed. For {frame_inches}-inch, {low}-{high}Hz "
                f"is typical. Higher values reduce delay and improve prop wash handling.",
                setting="dterm_lpf_hz",
                current=str(dterm_lpf)))

    # Setpoint Kalman
    if isinstance(kalman_q, (int, float)):
        if kalman_q > 300:
            findings.append(Finding(
                INFO, "Filters", f"Setpoint Kalman Q={kalman_q} - very aggressive",
                "High Kalman Q means less smoothing on the setpoint. "
                "This can make the quad feel very snappy but also amplify RC noise.",
                setting="setpoint_kalman_q",
                current=str(kalman_q)))

    return findings


# ─── PID Configuration Checks ───────────────────────────────────────────────

def check_pid_config(parsed, frame_inches=None):
    findings = []
    profile = get_active_control(parsed)
    pnum = parsed["active_control_profile"]

    # Check if EZ Tune is active in this profile
    ez_active = profile.get("ez_enabled", get_setting(parsed, "ez_enabled", None)) is True
    ez_note = " EZ Tune does NOT control this setting - it needs to be set manually." if ez_active else ""

    # I-term relax
    iterm_relax = profile.get("mc_iterm_relax", get_setting(parsed, "mc_iterm_relax", "RP"))
    other_profiles_have_rpy = False
    for n, p in parsed["control_profiles"].items():
        if n != pnum and p.get("mc_iterm_relax") == "RPY":
            other_profiles_have_rpy = True
            break

    if str(iterm_relax).upper() == "RP" and other_profiles_have_rpy:
        findings.append(Finding(
            WARNING, "PID", f"Active profile {pnum} uses iterm_relax=RP, other profiles use RPY",
            "I-term relax on yaw (RPY) reduces I-term windup during fast yaw moves, "
            "preventing yaw bounce-back. Your other profiles have RPY enabled but the active one doesn't. "
            f"This looks like a configuration oversight.{ez_note}",
            setting="mc_iterm_relax",
            current="RP",
            recommended="RPY",
            cli_fix="set mc_iterm_relax = RPY"))
    elif str(iterm_relax).upper() == "RP":
        findings.append(Finding(
            INFO, "PID", "I-term relax is RP (roll/pitch only)",
            "Consider RPY if you experience yaw bounce-back on fast rotations.",
            setting="mc_iterm_relax",
            current="RP",
            recommended="RPY"))

    # I-term relax cutoff - critical for large quads
    relax_cutoff = profile.get("mc_iterm_relax_cutoff",
                               get_setting(parsed, "mc_iterm_relax_cutoff", 15))
    if isinstance(relax_cutoff, (int, float)) and frame_inches and frame_inches >= 9:
        if relax_cutoff > 10:
            recommended_cutoff = 8 if frame_inches <= 10 else 5
            findings.append(Finding(
                WARNING, "PID",
                f"I-term relax cutoff = {relax_cutoff} - high for {frame_inches}-inch",
                f"Large quads need lower iterm_relax_cutoff to prevent bounce-back. "
                f"The default of 15 works for 5\", but {frame_inches}\" builds "
                f"need {recommended_cutoff} or lower. High cutoff lets I-term build "
                f"during maneuvers, causing overshoot on heavy airframes.{ez_note}",
                setting="mc_iterm_relax_cutoff",
                current=str(relax_cutoff),
                recommended=str(recommended_cutoff),
                cli_fix=f"set mc_iterm_relax_cutoff = {recommended_cutoff}"))

    # D-boost
    d_boost_min = profile.get("d_boost_min", get_setting(parsed, "d_boost_min", 1.0))
    d_boost_max = profile.get("d_boost_max", get_setting(parsed, "d_boost_max", 1.0))
    other_profiles_have_dboost = False
    for n, p in parsed["control_profiles"].items():
        if n != pnum and (p.get("d_boost_min", 1.0) != 1.0 or p.get("d_boost_max", 1.0) != 1.0):
            other_profiles_have_dboost = True
            break

    if d_boost_min == 1.0 and d_boost_max == 1.0 and other_profiles_have_dboost:
        findings.append(Finding(
            WARNING, "PID", f"Active profile {pnum} has no D-boost, other profiles do",
            "D-boost dynamically adjusts D-term based on setpoint changes - it gives more D during "
            "fast maneuvers (reduces overshoot) and less during straight flight (less noise). "
            "Your other profiles have it configured but the active one uses defaults. "
            f"Likely a configuration oversight.{ez_note}",
            setting="d_boost_min / d_boost_max",
            current="1.0 / 1.0 (disabled)",
            recommended="0.8 / 1.2 (like your other profiles)",
            cli_fix="set d_boost_min = 0.800\nset d_boost_max = 1.200"))

    # Antigravity
    antigrav = profile.get("antigravity_gain", get_setting(parsed, "antigravity_gain", 1.0))
    other_have_antigrav = any(
        p.get("antigravity_gain", 1.0) != 1.0
        for n, p in parsed["control_profiles"].items() if n != pnum
    )

    if antigrav == 1.0 and other_have_antigrav:
        findings.append(Finding(
            WARNING, "PID", f"Active profile {pnum} has no antigravity, other profiles do",
            "Antigravity boosts I-term during rapid throttle changes to counteract the "
            "altitude drop/gain when you punch or chop throttle. Your other profiles have it "
            f"but the active one doesn't.{ez_note}",
            setting="antigravity_gain",
            current="1.0 (disabled)",
            recommended="2.0 (like your other profiles)",
            cli_fix="set antigravity_gain = 2.000\nset antigravity_accelerator = 5.000"))

    # TPA
    tpa_rate = profile.get("tpa_rate", get_setting(parsed, "tpa_rate", 0))
    tpa_bp = profile.get("tpa_breakpoint", get_setting(parsed, "tpa_breakpoint", 1500))
    if isinstance(tpa_rate, (int, float)) and tpa_rate > 0:
        if isinstance(tpa_bp, (int, float)) and tpa_bp < 1300:
            findings.append(Finding(
                INFO, "PID", f"TPA breakpoint at {tpa_bp} - quite low",
                f"TPA starts reducing PID gains at throttle {tpa_bp}. This means gains start "
                f"dropping early. For aggressive flying, 1350-1500 is more common.",
                setting="tpa_breakpoint",
                current=str(tpa_bp),
                recommended="1350-1500"))

    # Rate acceleration limits - important for large quads
    if frame_inches and frame_inches >= 9:
        accel_rp = get_setting(parsed, "rate_accel_limit_roll_pitch", 0)
        accel_yaw = get_setting(parsed, "rate_accel_limit_yaw", 0)
        if isinstance(accel_rp, (int, float)) and accel_rp == 0:
            rec = 500 if frame_inches <= 10 else 360
            findings.append(Finding(
                INFO, "PID",
                f"No roll/pitch acceleration limit set for {frame_inches}-inch quad",
                f"Large quads benefit from rate acceleration limits to prevent sudden "
                f"rate demands that stress the frame and overwhelm the motors. "
                f"INAV devs recommend ~360 dps^2 for big heavy multirotors. "
                f"A value around {rec} is a good starting point for {frame_inches}\".{ez_note}",
                setting="rate_accel_limit_roll_pitch",
                current="0 (unlimited)",
                recommended=str(rec),
                cli_fix=f"set rate_accel_limit_roll_pitch = {rec}"))

    # EZ Tune cross-check (only relevant if EZ Tune is actually active)
    if ez_active:
        ez_damping = profile.get("ez_damping", get_setting(parsed, "ez_damping", 100))
        if isinstance(ez_damping, (int, float)) and ez_damping > 120:
            findings.append(Finding(
                INFO, "PID", f"EZ Tune damping = {ez_damping} (high)",
                "High EZ damping means more D-term. This can help with overshoot but "
                "also amplifies noise. If blackbox shows clean gyro, this is fine.",
                setting="ez_damping",
                current=str(ez_damping)))

    return findings


# ─── Navigation Checks ───────────────────────────────────────────────────────

def check_navigation(parsed):
    findings = []

    rth_alt = get_setting(parsed, "nav_rth_altitude", 5000)
    hover_thr = get_setting(parsed, "nav_mc_hover_thr", 1500)
    profile = get_active_control(parsed)

    # Safehome
    has_safehome = any("safehome" in line.lower() and "set" in line.lower()
                       for line in parsed["raw_text"].splitlines()
                       if not line.strip().startswith("#"))
    if not has_safehome:
        findings.append(Finding(
            INFO, "Navigation", "No safehome configured",
            "Safehome lets you define alternative landing points for RTH. "
            "If you fly at a regular location, setting a safehome ensures the quad "
            "returns to a safe landing spot even if the home point drifted.",
            setting="safehome",
            recommended="Configure via Configurator Mission tab"))

    # RTH altitude
    if isinstance(rth_alt, (int, float)):
        alt_m = rth_alt / 100
        if alt_m > 100:
            findings.append(Finding(
                WARNING, "Navigation", f"RTH altitude: {alt_m:.0f}m - very high",
                "High RTH altitude means longer return time and more battery used. "
                "Also may exceed legal altitude limits in many countries (120m/400ft).",
                setting="nav_rth_altitude",
                current=f"{alt_m:.0f}m",
                recommended="30-60m"))
        elif alt_m < 15:
            findings.append(Finding(
                WARNING, "Navigation", f"RTH altitude: {alt_m:.0f}m - quite low",
                "Low RTH altitude risks collision with trees, buildings, or terrain. "
                "30-50m is typical for safe RTH.",
                setting="nav_rth_altitude",
                current=f"{alt_m:.0f}m",
                recommended="30-50m"))
        else:
            findings.append(Finding(
                OK, "Navigation", f"RTH altitude: {alt_m:.0f}m",
                "Reasonable RTH altitude.",
                setting="nav_rth_altitude",
                current=f"{alt_m:.0f}m"))

    # Hover throttle
    battery = get_active_battery(parsed)
    hover = battery.get("nav_mc_hover_thr", hover_thr)
    if isinstance(hover, (int, float)):
        if hover > 1600:
            findings.append(Finding(
                WARNING, "Navigation", f"Hover throttle: {hover} - high",
                "Hover throttle above 1600 means the quad is heavy relative to motor power. "
                "Altitude hold and nav may be sluggish because there's limited headroom for correction.",
                setting="nav_mc_hover_thr",
                current=str(hover),
                recommended="1200-1500"))
        elif hover < 1100:
            findings.append(Finding(
                INFO, "Navigation", f"Hover throttle: {hover} - very low",
                "This suggests very powerful motors relative to weight. "
                "Altitude hold may be twitchy. Consider lowering PID gains for nav.",
                setting="nav_mc_hover_thr",
                current=str(hover)))

    # Nav PID values from active profile
    pos_p = profile.get("nav_mc_pos_xy_p", get_setting(parsed, "nav_mc_pos_xy_p", None))
    heading_p = profile.get("nav_mc_heading_p", get_setting(parsed, "nav_mc_heading_p", None))

    if pos_p is not None and isinstance(pos_p, (int, float)):
        if pos_p > 50:
            findings.append(Finding(
                WARNING, "Navigation", f"Position hold P = {pos_p} - aggressive",
                "High position P gain can cause oscillation (salad bowling) in position hold and RTH. "
                "The quad overcorrects, overshoots, and oscillates around the target position.",
                setting="nav_mc_pos_xy_p",
                current=str(pos_p),
                recommended="20-35",
                cli_fix=f"set nav_mc_pos_xy_p = 30"))
        elif pos_p < 15:
            findings.append(Finding(
                INFO, "Navigation", f"Position hold P = {pos_p} - conservative",
                "Low position P may result in slow corrections and drifting in wind.",
                setting="nav_mc_pos_xy_p",
                current=str(pos_p)))

    if heading_p is not None and isinstance(heading_p, (int, float)):
        if heading_p > 60:
            findings.append(Finding(
                INFO, "Navigation", f"Nav heading P = {heading_p} - high",
                "High heading P during navigation can cause yaw oscillation. "
                "Default is 60, consider lowering if you see heading wobble during RTH.",
                setting="nav_mc_heading_p",
                current=str(heading_p)))

    return findings


# ─── GPS Checks ──────────────────────────────────────────────────────────────

def check_gps(parsed):
    findings = []

    has_gps = "GPS" in parsed["features"]
    if not has_gps:
        if any(get_setting(parsed, k) for k in ["nav_rth_altitude", "failsafe_procedure"]
               if get_setting(parsed, k) in ("RTH", 2)):
            findings.append(Finding(
                CRITICAL, "GPS", "GPS feature not enabled but RTH failsafe configured",
                "Failsafe is set to RTH but GPS is not enabled. The quad cannot navigate home.",
                setting="feature GPS",
                current="disabled",
                recommended="enabled",
                cli_fix="feature GPS"))
        return findings

    findings.append(Finding(
        OK, "GPS", "GPS enabled",
        "GPS feature is active.",
        setting="feature GPS"))

    # Multi-constellation
    galileo = get_setting(parsed, "gps_ublox_use_galileo", False)
    beidou = get_setting(parsed, "gps_ublox_use_beidou", False)
    glonass = get_setting(parsed, "gps_ublox_use_glonass", False)

    constellations = ["GPS"]  # always on
    if galileo: constellations.append("Galileo")
    if beidou: constellations.append("BeiDou")
    if glonass: constellations.append("GLONASS")

    if len(constellations) >= 3:
        findings.append(Finding(
            OK, "GPS", f"Multi-constellation: {', '.join(constellations)}",
            "Multiple GNSS constellations improve fix quality and satellite count.",
            setting="gps_ublox_use_*"))
    elif len(constellations) == 1:
        findings.append(Finding(
            INFO, "GPS", "Only GPS constellation enabled",
            "Enabling Galileo and/or GLONASS improves fix quality with more visible satellites.",
            setting="gps_ublox_use_galileo",
            recommended="ON",
            cli_fix="set gps_ublox_use_galileo = ON"))

    # Compass
    mag_hw = get_setting(parsed, "mag_hardware", None)
    align_mag = get_setting(parsed, "align_mag", "DEFAULT")
    if mag_hw and str(mag_hw).upper() not in ("NONE", "0", "FALSE"):
        findings.append(Finding(
            OK, "GPS", f"Compass: {mag_hw} (alignment: {align_mag})",
            "Compass is configured. Critical for accurate heading in GPS modes.",
            setting="mag_hardware"))

        # Compass calibration quality check
        mag_gains = []
        for axis in ("x", "y", "z"):
            g = get_setting(parsed, f"maggain_{axis}", None)
            if g is not None and isinstance(g, (int, float)):
                mag_gains.append(g)
        if len(mag_gains) == 3:
            spread = (max(mag_gains) - min(mag_gains)) / max(max(mag_gains), 1) * 100
            if spread > 40:
                findings.append(Finding(
                    WARNING, "GPS",
                    f"Compass calibration may be poor (gain spread: {spread:.0f}%)",
                    f"Mag gains: X={mag_gains[0]}, Y={mag_gains[1]}, Z={mag_gains[2]}. "
                    f"A spread above 30-40% suggests the compass calibration was done near "
                    f"metallic objects or with motor interference. Poor compass cal causes "
                    f"toilet-bowling in position hold and erratic RTH heading. "
                    f"Recalibrate outdoors, away from metal, with motors off.",
                    setting="maggain_x/y/z",
                    current=f"X={mag_gains[0]} Y={mag_gains[1]} Z={mag_gains[2]}",
                    recommended="Recalibrate compass outdoors"))
            elif spread > 25:
                findings.append(Finding(
                    INFO, "GPS",
                    f"Compass gain spread: {spread:.0f}% - borderline",
                    f"Mag gains: X={mag_gains[0]}, Y={mag_gains[1]}, Z={mag_gains[2]}. "
                    f"This is borderline acceptable. If you see heading drift in poshold "
                    f"or toilet-bowling, recalibrate the compass.",
                    setting="maggain_x/y/z"))

    elif has_gps:
        findings.append(Finding(
            WARNING, "GPS", "No compass configured",
            "GPS navigation without a compass relies on GPS heading, which only works "
            "while moving. Hover position hold and slow-speed RTH will have poor heading accuracy.",
            setting="mag_hardware",
            recommended="Install and configure a compass"))

    return findings


# ─── Blackbox Checks ─────────────────────────────────────────────────────────

def check_blackbox(parsed):
    findings = []

    bb_denom = get_setting(parsed, "blackbox_rate_denom", 1)
    if isinstance(bb_denom, (int, float)) and bb_denom > 1:
        d = int(bb_denom)
        rate_desc = "half" if d == 2 else f"1/{d}"
        findings.append(Finding(
            INFO, "Blackbox", f"Blackbox logging at {rate_desc} rate",
            f"Recording every {d}{'nd' if d==2 else 'rd' if d==3 else 'th'} sample. "
            f"This saves flash space but reduces analysis resolution. "
            f"For PID tuning, 1/1 (full rate) gives the best data. "
            f"For long flights, 1/2 is a reasonable compromise.",
            setting="blackbox_rate_denom",
            current=str(bb_denom),
            recommended="1 for tuning, 2 for long flights"))

    # Check if essential fields are being logged
    disabled = parsed["blackbox_disabled"]
    essential_for_tuning = ["GYRO_RAW", "MOTORS", "RC_COMMAND"]
    missing = [f for f in essential_for_tuning if f in disabled]
    if missing:
        findings.append(Finding(
            WARNING, "Blackbox", f"Essential blackbox fields disabled: {', '.join(missing)}",
            "These fields are needed for PID tuning analysis. "
            "Without them, the blackbox analyzer cannot give accurate recommendations.",
            setting="blackbox",
            current=f"{', '.join(missing)} disabled",
            recommended="Enable all for tuning",
            cli_fix="\n".join(f"blackbox {f}" for f in missing)))

    # Navigation fields for nav analysis
    nav_disabled = [f for f in ["NAV_ACC", "NAV_POS"] if f in disabled]
    if nav_disabled:
        findings.append(Finding(
            INFO, "Blackbox", f"Nav blackbox fields disabled: {', '.join(nav_disabled)}",
            "These fields help analyze navigation performance (position hold, RTH). "
            "Enable them if you want to analyze nav tuning.",
            setting="blackbox",
            current=f"{', '.join(nav_disabled)} disabled"))

    return findings


# ─── Battery Checks ──────────────────────────────────────────────────────────

def check_battery(parsed):
    findings = []
    battery = get_active_battery(parsed)

    min_cell = battery.get("vbat_min_cell_voltage",
                           get_setting(parsed, "vbat_min_cell_voltage", 330))
    warn_cell = battery.get("vbat_warning_cell_voltage",
                            get_setting(parsed, "vbat_warning_cell_voltage", 350))
    capacity = battery.get("battery_capacity", get_setting(parsed, "battery_capacity", 0))
    cap_warn = battery.get("battery_capacity_warning",
                           get_setting(parsed, "battery_capacity_warning", 0))
    cap_crit = battery.get("battery_capacity_critical",
                           get_setting(parsed, "battery_capacity_critical", 0))

    if isinstance(min_cell, (int, float)):
        min_v = min_cell / 100 if min_cell > 100 else min_cell
        # Heuristic: large capacity (>7000mAh) + low min voltage often = Li-ion pack
        likely_lion = (isinstance(capacity, (int, float)) and capacity >= 7000
                       and min_v < 3.0)
        if min_v < 3.0:
            if likely_lion:
                findings.append(Finding(
                    INFO, "Battery", f"Minimum cell voltage: {min_v:.2f}V (likely Li-ion)",
                    f"With {capacity}mAh capacity and {min_v:.2f}V minimum, this looks like a Li-ion "
                    f"setup (18650/21700). This voltage is appropriate for Li-ion. "
                    f"If you switch to LiPo, raise this to 3.3V.",
                    setting="vbat_min_cell_voltage",
                    current=f"{min_v:.2f}V"))
            else:
                findings.append(Finding(
                    WARNING, "Battery", f"Minimum cell voltage: {min_v:.2f}V - low for LiPo",
                    "If using LiPo batteries, going below 3.0V per cell causes permanent damage. "
                    "3.3V is the recommended minimum for LiPo. If you're running Li-ion cells "
                    "(18650/21700), 2.5-2.8V is normal - you can ignore this warning.",
                    setting="vbat_min_cell_voltage",
                    current=f"{min_v:.2f}V",
                    recommended="330 (3.30V) for LiPo, 270 (2.70V) for Li-ion",
                    cli_fix="set vbat_min_cell_voltage = 330"))
        elif min_v < 3.2:
            findings.append(Finding(
                INFO, "Battery", f"Minimum cell voltage: {min_v:.2f}V - low side",
                "Below 3.2V accelerates LiPo wear. 3.3-3.5V is safer.",
                setting="vbat_min_cell_voltage",
                current=f"{min_v:.2f}V",
                recommended="330-350"))

    if isinstance(capacity, (int, float)) and capacity > 0:
        findings.append(Finding(
            OK, "Battery", f"Battery capacity: {capacity}mAh",
            "Capacity-based monitoring is configured.",
            setting="battery_capacity",
            current=f"{capacity}mAh"))

        if isinstance(cap_warn, (int, float)) and cap_warn > 0:
            warn_pct = cap_warn / capacity * 100
            findings.append(Finding(
                OK if warn_pct >= 30 else INFO, "Battery",
                f"Warning at {cap_warn}mAh remaining ({warn_pct:.0f}%)",
                f"Battery warning triggers at {warn_pct:.0f}% remaining capacity.",
                setting="battery_capacity_warning"))
    elif capacity == 0:
        findings.append(Finding(
            INFO, "Battery", "No battery capacity set",
            "Capacity-based monitoring gives more accurate remaining flight time "
            "than voltage alone. Set battery_capacity to your pack's mAh rating.",
            setting="battery_capacity",
            recommended="Set to your battery mAh"))

    return findings


# ─── Receiver Checks ─────────────────────────────────────────────────────────

def check_rx(parsed):
    findings = []

    rx_provider = get_setting(parsed, "serialrx_provider", None)
    if rx_provider:
        findings.append(Finding(
            OK, "Receiver", f"Serial RX: {rx_provider}",
            f"Using {rx_provider} protocol.",
            setting="serialrx_provider",
            current=str(rx_provider)))

    channel_map = parsed["master"].get("channel_map", "AETR")
    if channel_map != "AETR" and channel_map != "TAER":
        findings.append(Finding(
            INFO, "Receiver", f"Channel map: {channel_map}",
            "Non-standard channel map. Make sure your transmitter matches.",
            setting="map",
            current=channel_map))

    return findings


# ─── General Checks ──────────────────────────────────────────────────────────

def check_general(parsed):
    findings = []

    # Craft name
    name = get_setting(parsed, "name", "")
    if not name:
        findings.append(Finding(
            INFO, "General", "No craft name set",
            "Setting a name helps identify logs and OSD display.",
            setting="name",
            cli_fix='set name = MY_QUAD'))

    # I2C speed
    i2c = get_setting(parsed, "i2c_speed", "400KHZ")
    if str(i2c).upper() == "800KHZ":
        findings.append(Finding(
            OK, "General", "I2C bus at 800KHz",
            "Fast I2C reduces sensor read latency.",
            setting="i2c_speed",
            current="800KHZ"))

    # Airmode
    airmode = get_setting(parsed, "airmode_type", "STICK_CENTER")
    if str(airmode).upper() == "THROTTLE_THRESHOLD":
        findings.append(Finding(
            OK, "General", "Airmode: throttle threshold",
            "Airmode activates above throttle threshold - good for preventing accidental "
            "motor spin on the ground.",
            setting="airmode_type",
            current="THROTTLE_THRESHOLD"))

    return findings


# ─── Blackbox Cross-Reference ────────────────────────────────────────────────

def check_crossref_blackbox(parsed, bb_state):
    """Cross-reference config with blackbox analysis results."""
    findings = []

    if not isinstance(bb_state, dict):
        return findings

    actions = bb_state.get("actions", [])
    config = bb_state.get("config", {})
    scores = bb_state.get("scores", {})

    overall = scores.get("overall", 0)
    noise_score = scores.get("noise")
    pid_score = scores.get("pid")

    if overall > 0:
        findings.append(Finding(
            OK if overall >= 75 else WARNING if overall >= 50 else CRITICAL,
            "Blackbox", f"Last tune quality: {overall:.0f}/100",
            f"Noise: {noise_score:.0f}, PID: {pid_score:.0f}, Motors: {scores.get('motor', 0):.0f}" if noise_score else "",
            current=f"{overall:.0f}/100"))

    # Check if EZ Tune is fighting manual PID changes
    ez_active = get_setting(parsed, "ez_enabled", None) is True
    if ez_active:
        bb_roll_p = config.get("roll_p")
        diff_roll_p = get_active_control(parsed).get("mc_p_roll")
        if bb_roll_p and diff_roll_p and bb_roll_p != diff_roll_p:
            findings.append(Finding(
                WARNING, "Blackbox", "PID values differ between diff and blackbox",
                f"Roll P in diff: {diff_roll_p}, in last blackbox: {bb_roll_p}. "
                f"With EZ Tune active, manually set PIDs will be overwritten on reboot. "
                f"Either tune through EZ Tune parameters or disable EZ Tune (set ez_enabled = OFF).",
                setting="ez_enabled"))

    return findings


# ─── Terminal Output ─────────────────────────────────────────────────────────

# ─── Pre-Flight Sanity Check ─────────────────────────────────────────────────
# A "will this crash on first flight?" validator.
# Different from the tuning-focused analysis above — this catches
# dangerous misconfigurations that can destroy hardware or hurt people.

class SanityItem:
    """A single pre-flight check result."""
    FAIL = "FAIL"
    WARN = "WARN"
    ASK = "ASK"
    PASS = "PASS"

    def __init__(self, status, category, message, detail=None, question=None,
                 recommendation=None, cli_fix=None):
        self.status = status
        self.category = category
        self.message = message
        self.detail = detail or ""
        self.question = question       # Interactive confirmation prompt
        self.recommendation = recommendation
        self.cli_fix = cli_fix
        self.user_confirmed = None     # None=not asked, True/False=answer

    def __repr__(self):
        return f"<{self.status} {self.category}: {self.message}>"


def _ask_pilot(question, default_yes=False):
    """Ask the pilot a yes/no question interactively."""
    suffix = "[Y/n]" if default_yes else "[y/N]"
    try:
        answer = input(f"    ? {question} {suffix} ").strip().lower()
        if not answer:
            return default_yes
        return answer in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return default_yes


def _get_assigned_modes(parsed):
    """Return set of mode IDs that have aux channel assignments."""
    return {m["mode_id"] for m in parsed["aux_modes"]
            if m["range_low"] < m["range_high"]}


def _has_gps_uart(parsed):
    """Check if any serial port has GPS function enabled."""
    for _port_id, config in parsed["serial_ports"].items():
        parts = config.split()
        if len(parts) >= 1:
            try:
                func_mask = int(parts[0])
                if func_mask & 2:  # GPS function bit
                    return True
            except ValueError:
                pass
    return False


def _has_feature(parsed, feat_name):
    """Check if a feature is enabled (present in features list)."""
    return feat_name.upper() in [f.upper() for f in parsed["features"]]


def _has_feature_disabled(parsed, feat_name):
    """Check if a feature is explicitly disabled."""
    return feat_name.upper() in [f.upper() for f in parsed.get("features_disabled", [])]


def _detect_frame_from_name(parsed):
    """Try to detect frame size from craft name."""
    name = get_setting(parsed, "name", "")
    if not name:
        return None
    name_upper = str(name).upper()
    for size in [15, 12, 10, 7, 5]:
        if str(size) in name_upper:
            return size
    return None


def _detect_motor_count(parsed):
    """Detect motor count from motor_mix or platform type."""
    if parsed["motor_mix"]:
        return max(m["index"] for m in parsed["motor_mix"]) + 1
    # Infer from platform_type
    platform = get_setting(parsed, "platform_type", "")
    platform_map = {
        "QUADX": 4, "QUAD": 4, "QUADP": 4,
        "HEX6": 6, "HEXX": 6, "HEXH": 6, "HEX6X": 6,
        "OCTOX8": 8, "OCTOFLATX": 8, "OCTOFLATP": 8,
        "Y6": 6,
        "TRI": 3, "TRICOPTER": 3,
    }
    mixer = get_setting(parsed, "mixer_preset", "")
    if isinstance(mixer, str):
        for key, count in platform_map.items():
            if key in mixer.upper():
                return count
    return None


def run_sanity_check(parsed, frame_inches=None, interactive=True):
    """Run pre-flight sanity checks. Returns list of SanityItems."""
    items = []

    # Auto-detect frame size if not provided
    if frame_inches is None:
        frame_inches = _detect_frame_from_name(parsed)

    assigned_modes = _get_assigned_modes(parsed)
    has_gps = _has_gps_uart(parsed)
    motor_count = _detect_motor_count(parsed)
    profile = get_active_control(parsed)

    # ── 1. ARMING ────────────────────────────────────────────────────────
    if 0 not in assigned_modes:
        items.append(SanityItem(
            SanityItem.FAIL, "Arming",
            "No ARM switch assigned",
            "Without an ARM mode on an aux channel, the aircraft cannot be armed "
            "(or worse, may arm unexpectedly if using stick commands).",
            recommendation="Assign ARM to an AUX channel in the Modes tab"))
    else:
        arm_modes = [m for m in parsed["aux_modes"] if m["mode_id"] == 0]
        for am in arm_modes:
            rng = am["range_high"] - am["range_low"]
            if rng > 800:
                items.append(SanityItem(
                    SanityItem.WARN, "Arming",
                    f"ARM range is very wide ({am['range_low']}-{am['range_high']})",
                    "A wide ARM range increases the risk of accidental arming.",
                    recommendation="Use a narrow range like 1800-2100"))
            else:
                items.append(SanityItem(
                    SanityItem.PASS, "Arming",
                    "ARM switch assigned"))

    # ── 2. MOTOR OUTPUT ──────────────────────────────────────────────────
    output_enabled = get_setting(parsed, "motor_pwm_protocol", "")
    # In INAV, motor output is disabled by default until you enable it in Outputs tab.
    # The feature MOTOR_STOP / output is controlled differently.
    # If we see no motor protocol at all or DISABLED, that's a problem.
    if output_enabled == "DISABLED" or output_enabled == "" or output_enabled == 0:
        items.append(SanityItem(
            SanityItem.FAIL, "Motor Output",
            "Motor output appears DISABLED",
            "INAV disables motor output by default as a safety measure. "
            "Motors will not spin until you enable output in the Outputs tab.",
            recommendation="In Configurator: Outputs tab → enable motor output, select DSHOT protocol"))

    # ── 3. MOTOR DIRECTION ───────────────────────────────────────────────
    motor_inverted = get_setting(parsed, "motor_direction_inverted", False)
    if motor_inverted:
        items.append(SanityItem(
            SanityItem.ASK, "Motors",
            "Motor direction is INVERTED (props-out configuration)",
            "motor_direction_inverted = ON means the FC expects reversed motor spin "
            "(props-out). If your motors spin the normal way (props-in), the quad "
            "will yaw violently and flip on takeoff.",
            question="Are you running props-out (reversed) motor direction?",
            recommendation="If props-in, set motor_direction_inverted = OFF",
            cli_fix="set motor_direction_inverted = OFF"))
    else:
        items.append(SanityItem(
            SanityItem.PASS, "Motors",
            "Motor direction: standard (props-in)"))

    # ── 4. FAILSAFE ──────────────────────────────────────────────────────
    fs_proc = get_setting(parsed, "failsafe_procedure", "DROP")
    if fs_proc == "DROP" or fs_proc == 0:
        items.append(SanityItem(
            SanityItem.FAIL, "Failsafe",
            "Failsafe procedure is DROP",
            "On signal loss, the aircraft will disarm and freefall from any altitude. "
            "This is the default but extremely dangerous for any altitude flight.",
            recommendation="Set failsafe to RTH if you have GPS, or LAND",
            cli_fix="set failsafe_procedure = RTH"))
    elif (fs_proc == "RTH" or fs_proc == 2) and not has_gps:
        items.append(SanityItem(
            SanityItem.FAIL, "Failsafe",
            "Failsafe set to RTH but no GPS configured",
            "RTH failsafe requires GPS. Without it, the aircraft will fall back "
            "to emergency landing which may not work correctly.",
            recommendation="Configure GPS on a UART, or change failsafe to LAND"))
    elif fs_proc == "RTH" or fs_proc == 2:
        items.append(SanityItem(
            SanityItem.PASS, "Failsafe",
            "Failsafe set to RTH with GPS available"))

    # Failsafe throttle + airmode
    airmode = get_setting(parsed, "airmode_type", "")
    fs_throttle = get_setting(parsed, "failsafe_throttle", 1000)
    if isinstance(fs_throttle, (int, float)) and fs_throttle <= 1050:
        if airmode and str(airmode).upper() != "STICK_CENTER":
            items.append(SanityItem(
                SanityItem.WARN, "Failsafe",
                f"Low failsafe throttle ({fs_throttle}) with airmode enabled",
                "Airmode keeps PID corrections active even at zero throttle. "
                "Combined with low failsafe throttle, this can cause the aircraft "
                "to tumble during emergency landing instead of descending smoothly.",
                recommendation="Consider raising failsafe_throttle to hover level"))

    # ── 5. BATTERY ───────────────────────────────────────────────────────
    batt_profile = get_active_battery(parsed)
    vmin = batt_profile.get("bat_voltage_cell_min", get_setting(parsed, "bat_voltage_cell_min", 330))
    vmax = batt_profile.get("bat_voltage_cell_max", get_setting(parsed, "bat_voltage_cell_max", 420))
    vwarn = batt_profile.get("bat_voltage_cell_warning", get_setting(parsed, "bat_voltage_cell_warning", 350))

    if isinstance(vmin, (int, float)) and isinstance(vmax, (int, float)):
        if vmin >= vmax:
            items.append(SanityItem(
                SanityItem.FAIL, "Battery",
                f"Cell min voltage ({vmin/100:.2f}V) >= max ({vmax/100:.2f}V)",
                "Battery voltage limits are inverted or equal. "
                "The aircraft will think the battery is always critical."))
        elif vmin < 250 or vmin > 380:
            items.append(SanityItem(
                SanityItem.WARN, "Battery",
                f"Unusual cell min voltage: {vmin/100:.2f}V",
                "Normal range is 2.80-3.50V for LiPo, 2.50-2.80V for Li-ion.",
                question="Is this voltage correct for your battery type?"))
        else:
            items.append(SanityItem(
                SanityItem.PASS, "Battery",
                f"Battery limits: {vmin/100:.2f}V min, {vmax/100:.2f}V max"))

    # No battery monitoring
    if _has_feature_disabled(parsed, "VBAT"):
        items.append(SanityItem(
            SanityItem.FAIL, "Battery",
            "Battery voltage monitoring is DISABLED",
            "Without voltage monitoring, you will have no low battery warning "
            "and no battery-related failsafe. The aircraft will fly until "
            "the battery cuts out, causing an uncontrolled crash.",
            recommendation="Enable VBAT feature",
            cli_fix="feature VBAT"))

    # ── 6. GPS & NAVIGATION ──────────────────────────────────────────────
    nav_modes_assigned = assigned_modes & GPS_MODES
    nav_mode_names = [INAV_MODES.get(m, f"Mode {m}") for m in nav_modes_assigned]

    if nav_modes_assigned and not has_gps:
        items.append(SanityItem(
            SanityItem.FAIL, "Navigation",
            f"Nav modes assigned but no GPS configured: {', '.join(nav_mode_names)}",
            "These modes require GPS to function. Without GPS, activating them "
            "will cause unpredictable behavior or no effect at all.",
            recommendation="Configure GPS on a UART in the Ports tab"))
    elif nav_modes_assigned and has_gps:
        items.append(SanityItem(
            SanityItem.PASS, "Navigation",
            f"GPS configured for nav modes: {', '.join(nav_mode_names)}"))

    if has_gps and not nav_modes_assigned and 11 not in assigned_modes:
        items.append(SanityItem(
            SanityItem.WARN, "Navigation",
            "GPS configured but no NAV RTH mode assigned",
            "You have GPS hardware configured but no return-to-home switch. "
            "RTH is valuable as an emergency recovery option.",
            recommendation="Assign NAV RTH to an AUX channel"))

    # ALTHOLD without baro
    alt_modes_assigned = assigned_modes & ALT_MODES
    # Check for baro - it's usually a hardware feature, not always in diff
    baro_hardware = get_setting(parsed, "baro_hardware", "")
    if alt_modes_assigned and baro_hardware and str(baro_hardware).upper() == "NONE":
        items.append(SanityItem(
            SanityItem.FAIL, "Navigation",
            "NAV ALTHOLD assigned but barometer is disabled",
            "Altitude hold requires a barometer. Without it, altitude control "
            "will not work and the aircraft may climb or descend uncontrollably.",
            recommendation="Enable barometer hardware or remove ALTHOLD mode"))

    # POSHOLD/RTH without compass
    compass_modes_assigned = assigned_modes & COMPASS_MODES
    compass_mode_names = [INAV_MODES.get(m, f"Mode {m}") for m in compass_modes_assigned]
    mag_hardware = get_setting(parsed, "mag_hardware", "")
    mag_disabled = (isinstance(mag_hardware, str) and mag_hardware.upper() == "NONE") or mag_hardware == 0
    if compass_modes_assigned and mag_disabled:
        items.append(SanityItem(
            SanityItem.WARN, "Navigation",
            f"Compass disabled but nav modes assigned: {', '.join(compass_mode_names)}",
            "INAV can fly nav modes without compass using GPS-derived heading, "
            "but position hold performance is reduced and toilet-bowl patterns "
            "are more likely, especially at low speed.",
            question="Are you intentionally flying without compass?"))

    # Hover throttle at default
    hover_thr = get_setting(parsed, "nav_mc_hover_thr", 1500)
    if isinstance(hover_thr, (int, float)) and hover_thr == 1500:
        if nav_modes_assigned:
            items.append(SanityItem(
                SanityItem.ASK, "Navigation",
                "Hover throttle is at default (1500)",
                "The default hover throttle may not match your aircraft. "
                "If too low, the aircraft will sink in altitude hold. "
                "If too high, it will climb.",
                question="Have you calibrated the hover throttle for this aircraft?",
                recommendation="Hover the aircraft, note the throttle value, "
                "set nav_mc_hover_thr to that value"))

    # ── 7. PIDS ──────────────────────────────────────────────────────────
    p_roll = get_setting(parsed, "mc_p_roll", profile.get("mc_p_roll", 40))
    p_pitch = get_setting(parsed, "mc_p_pitch", profile.get("mc_p_pitch", 44))
    d_roll = get_setting(parsed, "mc_d_roll", profile.get("mc_d_roll", 25))
    d_pitch = get_setting(parsed, "mc_d_pitch", profile.get("mc_d_pitch", 25))

    # Zero P = no stabilization
    if isinstance(p_roll, (int, float)) and isinstance(p_pitch, (int, float)):
        if p_roll == 0 or p_pitch == 0:
            items.append(SanityItem(
                SanityItem.FAIL, "PIDs",
                f"P-term is ZERO (roll={p_roll}, pitch={p_pitch})",
                "With P=0 the aircraft has no stabilization on that axis. "
                "It will be completely uncontrollable.",
                recommendation="Set P values to at least 20"))

    # Extremely high PIDs
    if isinstance(p_roll, (int, float)) and isinstance(p_pitch, (int, float)):
        if p_roll > 100 or p_pitch > 100:
            items.append(SanityItem(
                SanityItem.FAIL, "PIDs",
                f"P-term is extremely high (roll={p_roll}, pitch={p_pitch})",
                "P values above 100 will cause violent oscillation on any airframe. "
                "The aircraft will shake itself apart on takeoff.",
                recommendation="Reduce P to a sane starting point for your frame size"))

    # PID mismatch with detected frame size
    if frame_inches and frame_inches in FRAME_PROFILES:
        fp = FRAME_PROFILES[frame_inches]
        expected_p = fp["pids"].get("mc_p_roll", 40)

        if isinstance(p_roll, (int, float)):
            # Check if PIDs are way off for this frame size
            # A 10" frame with 5" PIDs (40-44) is too aggressive
            # A 5" frame with 15" PIDs (26) is too sluggish
            if frame_inches >= 10 and p_roll >= 44:
                # Looks like 5"/7" defaults on a big frame
                items.append(SanityItem(
                    SanityItem.ASK, "PIDs",
                    f"PIDs look aggressive for a {frame_inches}-inch frame "
                    f"(P_roll={p_roll}, expected ~{expected_p})",
                    f"These PID values are typical for a 5-7 inch quad. On a "
                    f"{frame_inches}-inch frame, this will likely cause violent "
                    f"oscillation due to higher prop inertia.",
                    question=f"Is this actually a {frame_inches}-inch frame?",
                    recommendation=f"Use inav-params --setup {frame_inches} for starting PIDs",
                    cli_fix=f"# Run: inav-params --setup {frame_inches}"))
            elif frame_inches <= 5 and p_roll < 25:
                items.append(SanityItem(
                    SanityItem.ASK, "PIDs",
                    f"PIDs look very low for a {frame_inches}-inch frame "
                    f"(P_roll={p_roll}, expected ~{expected_p})",
                    "Very low PIDs on a small frame will make it feel mushy and unresponsive.",
                    question=f"Is this actually a {frame_inches}-inch frame?"))

    if isinstance(p_roll, (int, float)) and isinstance(p_pitch, (int, float)):
        if not (p_roll == 0 or p_pitch == 0) and not (p_roll > 100 or p_pitch > 100):
            items.append(SanityItem(
                SanityItem.PASS, "PIDs",
                f"PIDs in reasonable range (P_roll={p_roll}, P_pitch={p_pitch})"))

    # ── 8. RATES ─────────────────────────────────────────────────────────
    roll_rate = get_setting(parsed, "roll_rate",
                            profile.get("roll_rate", 40))
    pitch_rate = get_setting(parsed, "pitch_rate",
                             profile.get("pitch_rate", 40))
    yaw_rate = get_setting(parsed, "yaw_rate",
                           profile.get("yaw_rate", 30))

    for axis, rate in [("Roll", roll_rate), ("Pitch", pitch_rate), ("Yaw", yaw_rate)]:
        if isinstance(rate, (int, float)):
            if rate == 0:
                items.append(SanityItem(
                    SanityItem.FAIL, "Rates",
                    f"{axis} rate is ZERO",
                    f"With rate=0, the aircraft cannot rotate on the {axis.lower()} axis. "
                    "It will be uncontrollable.",
                    recommendation=f"Set {axis.lower()}_rate to at least 20"))
            elif rate > 120:  # 1200 dps — extreme
                items.append(SanityItem(
                    SanityItem.WARN, "Rates",
                    f"{axis} rate is extremely high ({rate} = {rate*10}dps)",
                    "Very high rates make the aircraft extremely twitchy and hard to control. "
                    "Most pilots use 200-700dps for multirotors.",
                    question=f"Is {rate*10}dps {axis.lower()} rate intentional?"))

    # ── 9. FILTERS ───────────────────────────────────────────────────────
    gyro_lpf = get_setting(parsed, "gyro_main_lpf_hz", 110)
    dterm_lpf = get_setting(parsed, "dterm_lpf_hz", 110)
    looptime = get_setting(parsed, "looptime", 500)

    if isinstance(looptime, (int, float)) and looptime > 0:
        sample_rate = 1_000_000 / looptime
        nyquist = sample_rate / 2

        if isinstance(gyro_lpf, (int, float)):
            if gyro_lpf == 0:
                items.append(SanityItem(
                    SanityItem.WARN, "Filters",
                    "Gyro LPF is DISABLED (0 Hz)",
                    "Running with no gyro low-pass filter passes all noise directly "
                    "to the PID controller. Unless you have very clean motors and "
                    "a well-built frame, this will cause motor heating and oscillation.",
                    question="Are you intentionally running without gyro LPF?"))
            elif gyro_lpf > nyquist:
                items.append(SanityItem(
                    SanityItem.FAIL, "Filters",
                    f"Gyro LPF ({gyro_lpf}Hz) is above Nyquist ({nyquist:.0f}Hz)",
                    "The filter cutoff is higher than what the sample rate can represent. "
                    "This provides no filtering and may cause aliasing artifacts.",
                    recommendation=f"Lower gyro_main_lpf_hz below {nyquist:.0f}"))

        if isinstance(dterm_lpf, (int, float)):
            if dterm_lpf == 0:
                items.append(SanityItem(
                    SanityItem.WARN, "Filters",
                    "D-term LPF is DISABLED (0 Hz)",
                    "D-term amplifies noise by design. Without filtering, motor "
                    "heating and high-frequency oscillation are very likely.",
                    question="Are you intentionally running without D-term LPF?"))

    if isinstance(gyro_lpf, (int, float)) and gyro_lpf > 0:
        if frame_inches and frame_inches in FRAME_PROFILES:
            expected_lpf = FRAME_PROFILES[frame_inches]["filters"]["gyro_main_lpf_hz"]
            if gyro_lpf > expected_lpf * 2:
                items.append(SanityItem(
                    SanityItem.WARN, "Filters",
                    f"Gyro LPF ({gyro_lpf}Hz) seems high for {frame_inches}-inch "
                    f"(expected ~{expected_lpf}Hz)",
                    "Higher filter cutoff passes more noise. Large frames need lower cutoffs "
                    "because they vibrate at lower frequencies."))
        else:
            items.append(SanityItem(
                SanityItem.PASS, "Filters",
                f"Gyro LPF at {gyro_lpf}Hz"))

    # ── 10. RECEIVER ─────────────────────────────────────────────────────
    rx_type = get_setting(parsed, "serialrx_provider", None)
    has_rx_uart = False
    for _port_id, config in parsed["serial_ports"].items():
        parts = config.split()
        if len(parts) >= 1:
            try:
                func_mask = int(parts[0])
                if func_mask & 64:  # Serial RX function bit
                    has_rx_uart = True
            except ValueError:
                pass

    if not has_rx_uart and rx_type is not None:
        # Might be using SPI RX or other non-serial receiver
        items.append(SanityItem(
            SanityItem.PASS, "Receiver",
            f"Receiver protocol: {rx_type}"))
    elif has_rx_uart:
        items.append(SanityItem(
            SanityItem.PASS, "Receiver",
            f"Serial receiver configured" + (f" ({rx_type})" if rx_type else "")))
    else:
        items.append(SanityItem(
            SanityItem.WARN, "Receiver",
            "No serial receiver UART detected in diff",
            "This may be normal if using SPI RX (like on AIO boards) or "
            "if the receiver is on default UART2 (not shown in diff)."))

    # ── 11. BOARD ALIGNMENT ──────────────────────────────────────────────
    align_roll = get_setting(parsed, "align_board_roll", 0)
    align_pitch = get_setting(parsed, "align_board_pitch", 0)
    align_yaw = get_setting(parsed, "align_board_yaw", 0)

    has_alignment = False
    for axis_name, val in [("roll", align_roll), ("pitch", align_pitch), ("yaw", align_yaw)]:
        if isinstance(val, (int, float)) and val != 0:
            has_alignment = True
            degrees = val / 10.0  # INAV stores in decidegrees
            if abs(degrees) > 0 and abs(degrees) not in (90, 180, 270):
                items.append(SanityItem(
                    SanityItem.ASK, "Board Alignment",
                    f"Board alignment {axis_name} = {degrees:.1f} degrees",
                    "Non-standard board alignment. Common values are 0, 90, 180, 270. "
                    "Wrong alignment will cause the FC to fight you — stabilization "
                    "will push the aircraft the wrong way.",
                    question=f"Is your FC rotated {degrees:.1f} degrees on {axis_name}?"))
            elif abs(degrees) in (90, 180, 270):
                items.append(SanityItem(
                    SanityItem.PASS, "Board Alignment",
                    f"Board rotated {degrees:.0f}° on {axis_name}"))

    if not has_alignment:
        items.append(SanityItem(
            SanityItem.PASS, "Board Alignment",
            "Standard orientation (no rotation)"))

    # ── 12. OSD / VIDEO ──────────────────────────────────────────────────
    osd_video = get_setting(parsed, "osd_video_system", "")
    if isinstance(osd_video, str) and osd_video.upper() not in ("", "AUTO"):
        # Has specific video system configured
        items.append(SanityItem(
            SanityItem.PASS, "OSD",
            f"OSD video system: {osd_video}"))

    # ── 13. BEEPER SAFETY ────────────────────────────────────────────────
    critical_beepers = ["BAT_CRIT_LOW", "BAT_LOW", "RX_LOST", "RX_LOST_LANDING"]
    disabled = parsed.get("beepers_disabled", [])
    missing_critical = [b for b in critical_beepers if b in disabled]
    if missing_critical:
        items.append(SanityItem(
            SanityItem.WARN, "Safety",
            f"Critical beepers disabled: {', '.join(missing_critical)}",
            "You will have no audible warning for low battery or signal loss.",
            recommendation="Enable critical beepers",
            cli_fix="\n".join(f"beeper {b}" for b in missing_critical)))
    else:
        items.append(SanityItem(
            SanityItem.PASS, "Safety",
            "Critical beepers enabled"))

    # ── 14. TRICOPTER SERVO ──────────────────────────────────────────────
    platform = get_setting(parsed, "platform_type", "")
    if isinstance(platform, str) and "TRI" in platform.upper():
        # Check servo direction
        items.append(SanityItem(
            SanityItem.ASK, "Platform",
            "Tricopter detected — verify tail servo direction",
            "Tricopters need the tail servo to push in the correct direction "
            "for yaw authority. An inverted servo will cause uncontrollable yaw.",
            question="Have you verified the tail servo moves correctly for yaw "
            "in the Configurator Outputs tab?"))

    # ── 15. HEX/OCTO MOTOR ORDER ────────────────────────────────────────
    if motor_count and motor_count >= 6:
        platform_name = "Hexacopter" if motor_count == 6 else "Octocopter"
        items.append(SanityItem(
            SanityItem.ASK, "Platform",
            f"{platform_name} detected ({motor_count} motors)",
            f"With {motor_count} motors, correct motor order and spin direction "
            "are critical. Verify in the Configurator Outputs tab that each motor "
            "matches the diagram.",
            question=f"Have you verified all {motor_count} motor positions and spin directions?"))

    # ── 16. FLIGHT MODES ─────────────────────────────────────────────────
    has_stabilized = assigned_modes & {1, 2}  # ANGLE or HORIZON
    if not has_stabilized and 12 not in assigned_modes:
        # No stabilized mode and no manual mode — only ARM
        items.append(SanityItem(
            SanityItem.WARN, "Modes",
            "No flight mode switch assigned (ANGLE/HORIZON)",
            "Without a flight mode on a switch, the aircraft defaults to "
            "ACRO/rate mode which requires advanced piloting skills. "
            "Most pilots want at least ANGLE mode for recovery.",
            question="Are you an experienced ACRO pilot?"))
    elif has_stabilized:
        mode_names = [INAV_MODES.get(m, "") for m in assigned_modes & {1, 2}]
        items.append(SanityItem(
            SanityItem.PASS, "Modes",
            f"Stabilized mode available: {', '.join(mode_names)}"))

    return items


def print_sanity_report(items, parsed, interactive=True):
    """Print the interactive pre-flight sanity check report."""
    R, B, C, G, Y, RED, DIM = _colors()

    print(f"\n  {B}{'=' * 55}{R}")
    print(f"  {B}  INAV Pre-Flight Sanity Check v{VERSION}{R}")
    print(f"  {B}{'=' * 55}{R}")

    # Aircraft identification
    name = get_setting(parsed, "name", "")
    board = parsed.get("board", "")
    version = parsed.get("version", "")
    platform = get_setting(parsed, "platform_type", "")

    print(f"\n  {B}AIRCRAFT{R}")
    if name:
        print(f"    Craft name:  {C}{name}{R}")
    if board:
        print(f"    Board:       {board}")
    if version:
        print(f"    Firmware:    INAV {version}")
    if platform:
        print(f"    Platform:    {platform}")
    print()

    # Group items by category
    categories = []
    seen = set()
    for item in items:
        if item.category not in seen:
            categories.append(item.category)
            seen.add(item.category)

    n_fail = 0
    n_warn = 0
    n_ask_unconfirmed = 0
    n_pass = 0

    for category in categories:
        cat_items = [i for i in items if i.category == category]
        print(f"  {B}{category}{R}")

        for item in cat_items:
            if item.status == SanityItem.PASS:
                print(f"    {G}✓{R} {item.message}")
                n_pass += 1

            elif item.status == SanityItem.FAIL:
                print(f"    {RED}✗ FAIL:{R} {item.message}")
                if item.detail:
                    for line in textwrap.wrap(item.detail, 68):
                        print(f"      {DIM}{line}{R}")
                if item.recommendation:
                    print(f"      {Y}→ {item.recommendation}{R}")
                if item.cli_fix:
                    print(f"      {C}CLI: {item.cli_fix}{R}")
                n_fail += 1

            elif item.status == SanityItem.WARN:
                print(f"    {Y}⚠ WARNING:{R} {item.message}")
                if item.detail:
                    for line in textwrap.wrap(item.detail, 68):
                        print(f"      {DIM}{line}{R}")
                if item.question and interactive:
                    confirmed = _ask_pilot(item.question)
                    item.user_confirmed = confirmed
                    if confirmed:
                        print(f"      {G}→ Acknowledged by pilot{R}")
                    else:
                        print(f"      {RED}→ Pilot says NO — review this before flying{R}")
                        n_fail += 1
                        continue
                elif item.recommendation:
                    print(f"      {Y}→ {item.recommendation}{R}")
                if item.cli_fix:
                    print(f"      {C}CLI: {item.cli_fix}{R}")
                n_warn += 1

            elif item.status == SanityItem.ASK:
                print(f"    {C}? CONFIRM:{R} {item.message}")
                if item.detail:
                    for line in textwrap.wrap(item.detail, 68):
                        print(f"      {DIM}{line}{R}")
                if item.question and interactive:
                    confirmed = _ask_pilot(item.question)
                    item.user_confirmed = confirmed
                    if confirmed:
                        print(f"      {G}→ Confirmed{R}")
                        n_pass += 1
                    else:
                        print(f"      {RED}→ NOT confirmed — fix this before flying{R}")
                        if item.recommendation:
                            print(f"      {Y}→ {item.recommendation}{R}")
                        if item.cli_fix:
                            print(f"      {C}CLI: {item.cli_fix}{R}")
                        n_fail += 1
                else:
                    # Non-interactive: treat ASK as warning
                    if item.recommendation:
                        print(f"      {Y}→ {item.recommendation}{R}")
                    n_ask_unconfirmed += 1

        print()

    # ── VERDICT ──────────────────────────────────────────────────────────
    print(f"  {B}{'=' * 55}{R}")
    print(f"  {B}  PRE-FLIGHT VERDICT{R}")
    print(f"  {B}{'=' * 55}{R}")

    if n_fail > 0:
        print(f"    {RED}✗{R} {n_fail} CRITICAL issue{'s' if n_fail > 1 else ''} — must fix before flying")
    if n_warn > 0:
        print(f"    {Y}⚠{R} {n_warn} WARNING{'s' if n_warn > 1 else ''} — review recommended")
    if n_ask_unconfirmed > 0:
        print(f"    {C}?{R} {n_ask_unconfirmed} item{'s' if n_ask_unconfirmed > 1 else ''} need{'s' if n_ask_unconfirmed == 1 else ''} pilot confirmation (use --check without --no-interactive)")
    if n_pass > 0:
        print(f"    {G}✓{R} {n_pass} check{'s' if n_pass > 1 else ''} passed")

    print()
    if n_fail > 0:
        print(f"    {RED}{B}██ NO-GO ██{R}")
        print(f"    {RED}Fix critical issues and run --check again.{R}")
    elif n_warn > 0 or n_ask_unconfirmed > 0:
        print(f"    {Y}{B}██ CONDITIONAL ██{R}")
        print(f"    {Y}Review warnings before flying.{R}")
    else:
        print(f"    {G}{B}██ GO ██{R}")
        print(f"    {G}All checks passed. Fly safe!{R}")
    print()

    return n_fail


def _pull_diff_from_device(device_path, no_color=False):
    """Connect to FC via MSP and pull diff all. Returns (text, info) or exits."""
    R, B, C, G, Y, RED, DIM = _colors()

    try:
        from inav_toolkit.msp import auto_detect_fc, INAVDevice
    except ImportError:
        from msp import auto_detect_fc, INAVDevice

    print(f"  {B}Connecting to FC...{R}", end=" ", flush=True)

    if device_path == "auto":
        dev, info = auto_detect_fc()
        if not dev:
            print(f"\n  {RED}ERROR: No INAV flight controller found.{R}")
            sys.exit(1)
    else:
        dev = INAVDevice(device_path)
        dev.open()
        info = dev.get_info()

    fc_name = info.get("craft_name", "Unknown")
    fw_ver = info.get("fw_version", "?")
    board = info.get("board_id", "?")
    print(f"{C}{board}{R} — {fc_name} — INAV {fw_ver}")

    print(f"  Pulling configuration...", end=" ", flush=True)
    try:
        diff_text = dev.get_diff_all(timeout=15.0)
        print(f"done ({len(diff_text.splitlines())} lines)")
    except Exception as e:
        print(f"\n  {RED}ERROR: Failed to pull diff: {e}{R}")
        dev.close()
        sys.exit(1)

    dev.close()
    return diff_text


# ─── Reporting ───────────────────────────────────────────────────────────────

def print_report(parsed, findings, frame_inches=None):
    R, B, C, G, Y, RED, DIM = _colors()

    sev_color = {CRITICAL: RED, WARNING: Y, INFO: C, OK: G}
    sev_icon = {CRITICAL: "✗", WARNING: "⚠", INFO: "ℹ", OK: "✓"}

    print(f"\n{B}{C}{'═'*70}{R}")
    print(f"{B}{C}  INAV Parameter Analyzer v{VERSION}{R}")
    print(f"{B}{C}{'═'*70}{R}")

    # Header info
    if parsed["version"]:
        print(f"  {DIM}Firmware: INAV {parsed['version']} | Board: {parsed['board']}{R}")
    name = get_setting(parsed, "name", "")
    if name:
        print(f"  {DIM}Craft: {name}{R}")
    if frame_inches:
        print(f"  {DIM}Profile: {frame_inches}-inch{R}")
    pnum = parsed["active_control_profile"]
    print(f"  {DIM}Active control profile: {pnum}{R}")

    # Summary counts
    counts = {CRITICAL: 0, WARNING: 0, INFO: 0, OK: 0}
    for f in findings:
        counts[f.severity] = counts.get(f.severity, 0) + 1

    print(f"\n  {B}SUMMARY:{R}")
    if counts[CRITICAL]:
        print(f"    {RED}{B}{counts[CRITICAL]} CRITICAL{R} - fix before flying")
    if counts[WARNING]:
        print(f"    {Y}{B}{counts[WARNING]} WARNING{R} - should address")
    if counts[INFO]:
        print(f"    {C}{counts[INFO]} suggestions{R}")
    if counts[OK]:
        print(f"    {G}{counts[OK]} checks passed{R}")

    # Group findings by category
    categories = {}
    for f in findings:
        if f.category not in categories:
            categories[f.category] = []
        categories[f.category].append(f)

    # Print non-OK findings first, grouped by category
    has_issues = any(f.severity != OK for f in findings)
    if has_issues:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}FINDINGS:{R}")
        print(f"{B}{C}{'─'*70}{R}")

        for cat, cat_findings in categories.items():
            issues = [f for f in cat_findings if f.severity != OK]
            if not issues:
                continue
            print(f"\n  {B}{cat}{R}")
            for f in issues:
                sc = sev_color[f.severity]
                icon = sev_icon[f.severity]
                print(f"    {sc}{B}{icon}{R} {B}{f.title}{R}")
                for line in textwrap.wrap(f.detail, width=62):
                    print(f"      {DIM}{line}{R}")
                if f.current:
                    print(f"      {DIM}Current: {f.current}{R}")
                if f.recommended:
                    print(f"      {DIM}Recommended: {f.recommended}{R}")

    # CLI fixes
    fixes = [f for f in findings if f.cli_fix and f.severity in (CRITICAL, WARNING)]
    if fixes:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}SUGGESTED CLI FIXES:{R}")
        print(f"{B}{C}{'─'*70}{R}")
        print()
        for f in fixes:
            print(f"  {DIM}# {f.title}{R}")
            for cmd in f.cli_fix.split("\n"):
                print(f"    {G}{cmd}{R}")
            print()
        print(f"    {G}save{R}")

    # OK items (compact)
    ok_items = [f for f in findings if f.severity == OK]
    if ok_items:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}PASSED:{R}")
        for f in ok_items:
            print(f"    {G}✓{R} {DIM}{f.title}{R}")

    print(f"\n{B}{C}{'═'*70}{R}\n")


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=f"INAV Parameter Analyzer v{VERSION} - Check diff all for issues",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Modes:
              Analysis (default):  inav-params diff_all.txt --frame 10
              Setup:               inav-params --setup 10 --voltage 6S
              Sanity check:        inav-params --check diff_all.txt
              Sanity check (USB):  inav-params --check --device auto
            
            Supported frame sizes: 5, 7, 10, 12, 15 inches
            Supported voltages: 4S, 6S, 8S, 12S
        """))
    parser.add_argument("--version", action="version", version=f"inav-params {VERSION}")
    parser.add_argument("difffile", nargs="?", default=None,
                        help="INAV `diff all` output (.txt file or stdin)")
    parser.add_argument("--frame", type=int, metavar="INCHES",
                        help="Frame size in inches (affects filter/PID recommendations)")
    parser.add_argument("--blackbox", metavar="STATE_JSON",
                        help="Blackbox analyzer state.json for cross-reference")
    parser.add_argument("--json", action="store_true",
                        help="Output findings as JSON")
    parser.add_argument("--setup", type=int, metavar="INCHES",
                        help="Generate starting configuration for a new quad (7/10/12/15)")
    parser.add_argument("--voltage", type=str, metavar="CELLS", default="4S",
                        help="Battery voltage for --setup (4S/6S/8S/12S, default: 4S)")
    parser.add_argument("--check", action="store_true",
                        help="Pre-flight sanity check mode - catches dangerous misconfigurations")
    parser.add_argument("--device", type=str, metavar="PORT", default=None,
                        help="FC serial port for --check mode (use 'auto' to scan)")
    parser.add_argument("--no-interactive", action="store_true",
                        help="Skip interactive questions in --check mode")
    parser.add_argument("--no-color", action="store_true",
                        help="Disable colored terminal output.")
    parser.add_argument("--lang", metavar="LANG",
                        help="Language for output (en, pt_BR, es). "
                             "Auto-detects from INAV_LANG env var or system locale.")
    args = parser.parse_args()

    # Initialize localization
    try:
        from inav_toolkit.i18n import set_locale, detect_locale
    except ImportError:
        try:
            from i18n import set_locale, detect_locale
        except ImportError:
            set_locale = detect_locale = None
    if set_locale and detect_locale:
        lang = getattr(args, 'lang', None) or detect_locale()
        set_locale(lang)

    if args.no_color:
        _disable_colors()

    # ─── Sanity check mode ─────────────────────────────────────────────
    if args.check:
        if args.device:
            text = _pull_diff_from_device(args.device, args.no_color)
        elif args.difffile:
            if args.difffile == "-":
                text = sys.stdin.read()
            else:
                if not os.path.isfile(args.difffile):
                    print(f"ERROR: File not found: {args.difffile}")
                    sys.exit(1)
                with open(args.difffile, "r", errors="replace") as f:
                    text = f.read()
        else:
            parser.error("--check requires a diff file or --device")

        parsed = parse_diff_all(text)
        interactive = not args.no_interactive
        items = run_sanity_check(parsed, frame_inches=args.frame,
                                 interactive=interactive)

        if args.json:
            output = [{
                "status": i.status,
                "category": i.category,
                "message": i.message,
                "detail": i.detail,
                "recommendation": i.recommendation,
                "cli_fix": i.cli_fix,
                "user_confirmed": i.user_confirmed,
            } for i in items]
            print(json.dumps(output, indent=2))
        else:
            n_fail = print_sanity_report(items, parsed, interactive=interactive)
            sys.exit(1 if n_fail > 0 else 0)
        return
    args = parser.parse_args()

    if args.no_color:
        _disable_colors()

    # ─── Setup mode ───────────────────────────────────────────────────────
    if args.setup:
        if not args.json:
            print(f"\n  ▲ INAV Parameter Analyzer v{VERSION}")
            print(f"  Mode: Starting configuration")

        voltage = args.voltage.upper()
        if not voltage.endswith("S"):
            voltage = voltage + "S"

        config = generate_setup_config(args.setup, voltage=voltage)

        if args.json:
            print_setup_json(config)
        else:
            print_setup_report(config)

        # If a diff file was also provided, show what would change
        if args.difffile and os.path.isfile(args.difffile):
            with open(args.difffile, "r", errors="replace") as f:
                text = f.read()
            parsed = parse_diff_all(text)
            profile = get_active_control(parsed)

            R, B, C, G, Y, _RED, DIM = _colors()
            print(f"  {B}COMPARISON WITH CURRENT CONFIG ({args.difffile}):{R}")
            print(f"             {'Current':>8} {'Suggested':>10} {'Change':>8}")

            all_new = {}
            all_new.update(config["pids"])
            all_new.update(config["filters"])
            all_new.update(config["other"])

            for k, new_val in sorted(all_new.items()):
                cur = profile.get(k, get_setting(parsed, k, None))
                if cur is not None and cur != new_val:
                    if isinstance(cur, (int, float)) and isinstance(new_val, (int, float)):
                        diff = new_val - cur
                        sign = "+" if diff > 0 else ""
                        color = G if abs(diff) < abs(cur) * 0.05 else Y
                        print(f"    {k:26} {cur:>8} {new_val:>10} {color}{sign}{diff}{R}")
                    else:
                        print(f"    {k:26} {str(cur):>8} {str(new_val):>10}")

            print()
        return

    # ─── Analysis mode ────────────────────────────────────────────────────
    if not args.difffile:
        parser.error("diff file is required (or use --setup INCHES)")

    # Load diff all
    if args.difffile == "-":
        text = sys.stdin.read()
    else:
        if not os.path.isfile(args.difffile):
            print(f"ERROR: File not found: {args.difffile}")
            sys.exit(1)
        with open(args.difffile, "r", errors="replace") as f:
            text = f.read()

    if not args.json:
        print(f"\n  ▲ INAV Parameter Analyzer v{VERSION}")
        print(f"  Loading: {args.difffile}")

    parsed = parse_diff_all(text)

    if not args.json:
        if parsed["version"]:
            print(f"  Firmware: INAV {parsed['version']} on {parsed['board']}")
        name = get_setting(parsed, "name", "")
        if name:
            print(f"  Craft: {name}")

    # Load blackbox state if provided
    bb_state = None
    if args.blackbox:
        try:
            with open(args.blackbox) as f:
                bb_state = json.load(f)
            if not args.json:
                print(f"  Blackbox state: {args.blackbox}")
        except Exception as e:
            if not args.json:
                print(f"  ⚠ Could not load blackbox state: {e}")

    # Run checks
    findings = run_all_checks(parsed, frame_inches=args.frame, blackbox_state=bb_state)

    if args.json:
        output = [{
            "severity": f.severity,
            "category": f.category,
            "title": f.title,
            "detail": f.detail,
            "setting": f.setting,
            "current": f.current,
            "recommended": f.recommended,
            "cli_fix": f.cli_fix,
        } for f in findings]
        print(json.dumps(output, indent=2))
    else:
        print_report(parsed, findings, frame_inches=args.frame)


if __name__ == "__main__":
    main()
