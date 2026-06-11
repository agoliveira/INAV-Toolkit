#!/usr/bin/env python3
"""
INAV Toolkit — Flight Tools v2.23.0
===================================
Standalone flight utilities built on the blackbox decoder:

  anonymize_log()      — strip identifying data, write shareable CSV (--share)
  analyze_range()      — cruise efficiency and projected range
  analyze_postmortem() — crash forensics on the final seconds (--postmortem)

These are imported by blackbox_analyzer for CLI wiring but have no
side effects on import.
"""

import os
import math
import numpy as np

try:
    from inav_toolkit.i18n import t
except ImportError:
    def t(key, **kw):
        return key.format(**kw) if kw else key

FMODE_FAILSAFE = 1 << 9

# Canonical data-dict key → original blackbox column name.
# Used by the anonymizer so its CSV output round-trips through
# parse_csv_log() and external tools that expect blackbox names.
_REVERSE_MAP = [
    ("loopiter", "loopIteration"),
    ("time", "time"),
    ("gyro_roll", "gyroADC[0]"), ("gyro_pitch", "gyroADC[1]"), ("gyro_yaw", "gyroADC[2]"),
    ("setpoint_roll", "rcCommand[0]"), ("setpoint_pitch", "rcCommand[1]"),
    ("setpoint_yaw", "rcCommand[2]"), ("throttle", "rcCommand[3]"),
    ("rc_roll", "rcData[0]"), ("rc_pitch", "rcData[1]"),
    ("rc_yaw", "rcData[2]"), ("rc_throttle", "rcData[3]"),
    ("axisP_roll", "axisP[0]"), ("axisP_pitch", "axisP[1]"), ("axisP_yaw", "axisP[2]"),
    ("axisI_roll", "axisI[0]"), ("axisI_pitch", "axisI[1]"), ("axisI_yaw", "axisI[2]"),
    ("axisD_roll", "axisD[0]"), ("axisD_pitch", "axisD[1]"), ("axisD_yaw", "axisD[2]"),
    ("motor0", "motor[0]"), ("motor1", "motor[1]"),
    ("motor2", "motor[2]"), ("motor3", "motor[3]"),
    ("att_roll", "attitude[0]"), ("att_pitch", "attitude[1]"), ("att_heading", "attitude[2]"),
    ("acc_x", "accSmooth[0]"), ("acc_y", "accSmooth[1]"), ("acc_z", "accSmooth[2]"),
    ("baro_alt", "BaroAlt"),
    ("nav_pos_n", "navPos[0]"), ("nav_pos_e", "navPos[1]"), ("nav_pos_u", "navPos[2]"),
    ("nav_vel_n", "navVel[0]"), ("nav_vel_e", "navVel[1]"), ("nav_vel_u", "navVel[2]"),
    ("nav_tgt_n", "navTgtPos[0]"), ("nav_tgt_e", "navTgtPos[1]"), ("nav_tgt_u", "navTgtPos[2]"),
    ("nav_state", "navState"), ("nav_flags", "navFlags"),
    ("nav_eph", "navEPH"), ("nav_epv", "navEPV"),
    ("vbat", "vbatLatest"), ("amperage", "amperageLatest"), ("rssi", "rssi"),
]

_M_PER_DEG_LAT = 111320.0


def _extract_gps_points(data, sr):
    """Extract GPS fixes from _gps_frames as a list of dicts.

    Returns [] when no usable fixes. Coordinates in degrees, speed in m/s
    (INAV logs GPS_speed in cm/s), altitude in meters.
    """
    frames = data.get("_gps_frames", [])
    if not frames:
        return []

    def field(fields, *names):
        for n in names:
            if n in fields:
                return fields[n]
        low = {k.lower(): v for k, v in fields.items()}
        for n in names:
            if n.lower() in low:
                return low[n.lower()]
        return None

    points = []
    for idx, fields in frames:
        lat = field(fields, "GPS_coord[0]", "GPS_coord_0", "GPS_lat")
        lon = field(fields, "GPS_coord[1]", "GPS_coord_1", "GPS_lon")
        if lat is None or lon is None:
            continue
        try:
            lat_deg = float(lat) / 1e7
            lon_deg = float(lon) / 1e7
        except (ValueError, TypeError):
            continue
        if abs(lat_deg) < 0.1 and abs(lon_deg) < 0.1:
            continue  # no fix
        try:
            speed_ms = float(field(fields, "GPS_speed", "GPS_ground_speed") or 0) / 100.0
            alt_m = float(field(fields, "GPS_altitude", "GPS_alt") or 0)
            sats = int(field(fields, "GPS_numSat", "GPS_numsat") or 0)
        except (ValueError, TypeError):
            speed_ms, alt_m, sats = 0.0, 0.0, 0
        points.append({
            "idx": int(idx), "t": (idx / sr) if sr > 0 else 0.0,
            "lat": lat_deg, "lon": lon_deg,
            "speed_ms": speed_ms, "alt_m": alt_m, "sats": sats,
        })
    return points


def _latlon_to_ne_m(lat, lon, home_lat, home_lon):
    """Convert lat/lon to north/east meters from home (local tangent plane)."""
    north = (lat - home_lat) * _M_PER_DEG_LAT
    east = (lon - home_lon) * _M_PER_DEG_LAT * math.cos(math.radians(home_lat))
    return north, east


def _haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


# ═══════════════════════════════════════════════════════════════════════════
# Anonymizer (--share)
# ═══════════════════════════════════════════════════════════════════════════

def anonymize_log(data, output_path, keep_name=False, craft_name=""):
    """Write an anonymized, shareable CSV from parsed log data.

    Strips: absolute GPS coordinates (converted to meters-from-home),
    home location, GPS altitude (converted to relative), craft name.
    Keeps: all flight dynamics, tune-relevant fields, timestamps.

    Args:
        data: parsed log dict from decode_blackbox_native()/parse_csv_log()
        output_path: CSV path to write
        keep_name: include craft name as a comment line
        craft_name: craft name from config (only used with keep_name)

    Returns summary dict: {output, n_rows, columns, stripped, kept_name}
    """
    n_rows = int(data.get("n_rows") or len(data.get("time_s", [])))
    if n_rows == 0:
        raise ValueError("no rows in parsed log")

    cols = []  # (header, array)
    for our_key, orig_name in _REVERSE_MAP:
        arr = data.get(our_key)
        if arr is not None and hasattr(arr, "__len__") and len(arr) == n_rows:
            cols.append((orig_name, arr))
    if not any(h == "time" for h, _ in cols):
        ts = data.get("time_s")
        if ts is not None and len(ts) == n_rows:
            cols.insert(0, ("time", np.asarray(ts) * 1e6))

    stripped = []
    gps_points = _extract_gps_points(data, data.get("sample_rate", 0) or 1)
    if gps_points:
        home = gps_points[0]
        north = np.full(n_rows, np.nan)
        east = np.full(n_rows, np.nan)
        rel_alt = np.full(n_rows, np.nan)
        spd = np.full(n_rows, np.nan)
        for p in gps_points:
            i = p["idx"]
            if 0 <= i < n_rows:
                n_m, e_m = _latlon_to_ne_m(p["lat"], p["lon"], home["lat"], home["lon"])
                north[i], east[i] = n_m, e_m
                rel_alt[i] = p["alt_m"] - home["alt_m"]
                spd[i] = p["speed_ms"]
        cols.append(("pos_north_m", north))
        cols.append(("pos_east_m", east))
        cols.append(("rel_alt_m", rel_alt))
        cols.append(("gps_speed_ms", spd))
        stripped.append(f"GPS coordinates ({len(gps_points)} fixes → meters-from-home)")
        stripped.append("home location")
        stripped.append("absolute GPS altitude (→ relative)")
    if craft_name and not keep_name:
        stripped.append(f'craft name ("{craft_name}")')

    with open(output_path, "w", encoding="utf-8", newline="") as f:
        f.write("# anonymized by INAV Toolkit --share — GPS converted to "
                "meters-from-home, identifying headers removed\n")
        if keep_name and craft_name:
            f.write(f"# craft: {craft_name}\n")
        f.write(",".join(h for h, _ in cols) + "\n")
        arrays = [np.asarray(a, dtype=float) for _, a in cols]
        for r in range(n_rows):
            vals = []
            for a in arrays:
                v = a[r]
                if np.isnan(v):
                    vals.append("")
                elif float(v).is_integer():
                    vals.append(str(int(v)))
                else:
                    vals.append(f"{v:.3f}")
            f.write(",".join(vals) + "\n")

    return {"output": output_path, "n_rows": n_rows,
            "columns": [h for h, _ in cols], "stripped": stripped,
            "kept_name": bool(keep_name and craft_name)}


def print_anonymize_summary(summary):
    print(f"\n  {t('share.title')}")
    print(f"  ✓ {summary['output']} ({summary['n_rows']:,} rows, "
          f"{len(summary['columns'])} columns)")
    if summary["stripped"]:
        print(f"  {t('share.stripped')}:")
        for s in summary["stripped"]:
            print(f"    - {s}")
    else:
        print(f"  {t('share.nothing_to_strip')}")
    print(f"  {t('share.note')}\n")


# ═══════════════════════════════════════════════════════════════════════════
# Range / efficiency
# ═══════════════════════════════════════════════════════════════════════════

def analyze_range(data, sr, config=None, capacity_mah=None):
    """Cruise efficiency and projected range from GPS + current data.

    Returns dict or None when prerequisites (GPS fixes + amperage) missing.
    Keys: segments, total_cruise_km, avg_speed_kmh, mah_per_km, wh_per_km,
    capacity_mah, projected_range_km, projected_range_70_km, spread_pct.
    """
    points = _extract_gps_points(data, sr)
    if len(points) < 10 or "amperage" not in data:
        return None

    amp = np.asarray(data["amperage"], dtype=float) / 100.0  # → A
    vbat = None
    if "vbat" in data:
        vbat = np.asarray(data["vbat"], dtype=float) / 100.0  # → V

    if capacity_mah is None and config:
        try:
            capacity_mah = int(config.get("battery_capacity") or 0) or None
        except (ValueError, TypeError):
            capacity_mah = None

    # Cruise segments: speed > 3 m/s sustained ≥ 15 s
    MIN_SPEED, MIN_DUR = 3.0, 15.0
    segments = []
    seg_start = None
    for i, p in enumerate(points):
        cruising = p["speed_ms"] > MIN_SPEED
        if cruising and seg_start is None:
            seg_start = i
        elif not cruising and seg_start is not None:
            if points[i - 1]["t"] - points[seg_start]["t"] >= MIN_DUR:
                segments.append((seg_start, i - 1))
            seg_start = None
    if seg_start is not None and points[-1]["t"] - points[seg_start]["t"] >= MIN_DUR:
        segments.append((seg_start, len(points) - 1))
    if not segments:
        return None

    seg_results = []
    for a, b in segments:
        dist_m = 0.0
        for j in range(a + 1, b + 1):
            dist_m += _haversine_m(points[j - 1]["lat"], points[j - 1]["lon"],
                                   points[j]["lat"], points[j]["lon"])
        dur_s = points[b]["t"] - points[a]["t"]
        if dist_m < 50 or dur_s <= 0:
            continue
        i0 = max(0, points[a]["idx"])
        i1 = min(len(amp), points[b]["idx"] + 1)
        seg_amp = amp[i0:i1]
        seg_amp = seg_amp[~np.isnan(seg_amp)]
        if len(seg_amp) == 0:
            continue
        avg_a = float(np.mean(seg_amp))
        mah = avg_a * (dur_s / 3600.0) * 1000.0
        wh = None
        if vbat is not None:
            seg_v = vbat[i0:i1]
            seg_v = seg_v[~np.isnan(seg_v)]
            if len(seg_v):
                wh = float(np.mean(seg_v)) * avg_a * (dur_s / 3600.0)
        km = dist_m / 1000.0
        seg_results.append({
            "t0": round(points[a]["t"], 1), "t1": round(points[b]["t"], 1),
            "dist_km": round(km, 3), "dur_s": round(dur_s, 1),
            "speed_kmh": round(km / (dur_s / 3600.0), 1),
            "mah_per_km": round(mah / km, 1),
            "wh_per_km": round(wh / km, 2) if wh is not None else None,
        })
    if not seg_results:
        return None

    total_km = sum(s["dist_km"] for s in seg_results)
    total_mah = sum(s["mah_per_km"] * s["dist_km"] for s in seg_results)
    mah_per_km = total_mah / total_km
    wh_vals = [s["wh_per_km"] for s in seg_results if s["wh_per_km"] is not None]
    effs = [s["mah_per_km"] for s in seg_results]
    spread = (max(effs) - min(effs)) / mah_per_km * 100.0 if len(effs) > 1 else 0.0

    result = {
        "segments": seg_results,
        "total_cruise_km": round(total_km, 2),
        "avg_speed_kmh": round(sum(s["speed_kmh"] * s["dist_km"] for s in seg_results) / total_km, 1),
        "mah_per_km": round(mah_per_km, 1),
        "wh_per_km": round(sum(wh_vals) / len(wh_vals), 2) if wh_vals else None,
        "capacity_mah": capacity_mah,
        "projected_range_km": None,
        "projected_range_70_km": None,
        "spread_pct": round(spread, 0),
    }
    if capacity_mah:
        result["projected_range_km"] = round(capacity_mah / mah_per_km, 1)
        result["projected_range_70_km"] = round(capacity_mah * 0.7 / mah_per_km, 1)
    return result


def print_range_report(r):
    print(f"\n  {t('range.title')}")
    print(f"  {t('range.cruise')}: {r['total_cruise_km']:.2f} km @ "
          f"{r['avg_speed_kmh']:.0f} km/h "
          f"({len(r['segments'])} segment{'s' if len(r['segments']) != 1 else ''})")
    eff = f"  {t('range.efficiency')}: {r['mah_per_km']:.0f} mAh/km"
    if r["wh_per_km"] is not None:
        eff += f" | {r['wh_per_km']:.1f} Wh/km"
    if r["spread_pct"] and r["spread_pct"] >= 15:
        eff += f"  ({t('range.spread')}: {r['spread_pct']:.0f}%)"
    print(eff)
    if r["projected_range_km"]:
        print(f"  {t('range.projected', cap=r['capacity_mah'])}: "
              f"{r['projected_range_km']:.1f} km "
              f"({t('range.with_reserve')}: {r['projected_range_70_km']:.1f} km)")
        print(f"  {t('range.caveat')}")


def markdown_range_section(r):
    lines = [f"### {t('range.title')}", ""]
    lines.append(f"- {t('range.cruise')}: **{r['total_cruise_km']:.2f} km** @ "
                 f"{r['avg_speed_kmh']:.0f} km/h")
    eff = f"- {t('range.efficiency')}: **{r['mah_per_km']:.0f} mAh/km**"
    if r["wh_per_km"] is not None:
        eff += f" ({r['wh_per_km']:.1f} Wh/km)"
    lines.append(eff)
    if r["projected_range_km"]:
        lines.append(f"- {t('range.projected', cap=r['capacity_mah'])}: "
                     f"**{r['projected_range_km']:.1f} km** "
                     f"({t('range.with_reserve')}: {r['projected_range_70_km']:.1f} km)")
        lines.append(f"- *{t('range.caveat')}*")
    lines.append("")
    return lines


# ═══════════════════════════════════════════════════════════════════════════
# Postmortem (--postmortem)
# ═══════════════════════════════════════════════════════════════════════════

def detect_abrupt_end(data, sr):
    """Heuristic: log ends mid-flight (high throttle at last frame, no wind-down)."""
    thr = data.get("throttle")
    if thr is None:
        thr = data.get("rc_throttle")
    if thr is None or len(thr) < int(sr):
        return False
    tail = np.asarray(thr[-max(3, int(sr * 0.2)):], dtype=float)
    tail = tail[~np.isnan(tail)]
    if len(tail) == 0:
        return False
    return float(np.mean(tail)) > 1300


def analyze_postmortem(data, sr, config=None, window_s=10.0):
    """Crash forensics on the last `window_s` seconds of a log.

    Runs independent detectors and returns ranked verdicts.
    Verdicts are evidence-based on the *last recorded data* — logs often
    end at power loss, so the true final moments may be missing.

    Returns dict: {window_s, t_end, abrupt_end,
                   verdicts: [{cause, confidence, evidence: [str]}]}
    """
    n = int(data.get("n_rows") or len(data.get("time_s", [])))
    if n < int(sr * 2):
        return None
    w = min(n, max(int(sr * window_s), int(sr * 2)))
    s = n - w  # window start index
    t_end = n / sr

    def win(key):
        arr = data.get(key)
        if arr is None or len(arr) != n:
            return None
        a = np.asarray(arr[s:], dtype=float)
        return a if np.any(~np.isnan(a)) else None

    verdicts = []

    # ── Motor/ESC failure: one output flat while others actively vary ──
    motors = {i: win(f"motor{i}") for i in range(4)}
    motors = {i: m for i, m in motors.items() if m is not None}
    if len(motors) >= 3:
        # Failures start mid-window: judge flatness on the last 60% of it
        tail0 = int(w * 0.4)
        tails = {i: m[tail0:] for i, m in motors.items()}
        ranges = {i: float(np.nanmax(m) - np.nanmin(m)) for i, m in tails.items()}
        means = {i: float(np.nanmean(m)) for i, m in tails.items()}
        active = [i for i, r in ranges.items() if r > 80]
        for i, r in ranges.items():
            if r < 8 and len(active) >= 2:
                others_mean = np.mean([means[j] for j in motors if j != i])
                ev = [f"M{i} output flat (range {r:.0f}) over the last "
                      f"{w / sr:.1f}s while {len(active)} motors varied actively",
                      f"M{i} mean {means[i]:.0f} vs others {others_mean:.0f}"]
                conf = "HIGH" if abs(means[i] - others_mean) > 150 else "MEDIUM"
                verdicts.append({"cause": "motor_esc_failure",
                                 "label": t("pm.motor_failure", motor=i),
                                 "confidence": conf, "evidence": ev})

    # ── Voltage collapse ──
    vb = win("vbat")
    if vb is not None:
        v = vb[~np.isnan(vb)] / 100.0
        if len(v) > int(sr):
            dur = len(v) / sr
            slope = (float(v[-int(sr // 2) or 1:].mean()) - float(v[:int(sr // 2) or 1].mean())) / max(dur, 0.1)
            cells = None
            if config:
                cells = config.get("_cell_count")
            cell_div = cells if cells else max(1, round(float(np.median(v)) / 3.8))
            min_cell = float(np.min(v)) / cell_div
            ev = []
            if slope < -0.4:
                ev.append(f"pack voltage falling {abs(slope):.2f} V/s in the final window")
            if min_cell < 3.0:
                ev.append(f"minimum cell voltage {min_cell:.2f}V")
            if ev:
                conf = "HIGH" if (slope < -0.8 or min_cell < 2.8) else "MEDIUM"
                verdicts.append({"cause": "voltage_collapse",
                                 "label": t("pm.voltage_collapse"),
                                 "confidence": conf, "evidence": ev})

    # ── RX loss / failsafe ──
    rc_keys = ["rc_roll", "rc_pitch", "rc_yaw"]
    rc = [win(k) for k in rc_keys]
    rc = [a for a in rc if a is not None]
    ev = []
    if len(rc) >= 2:
        half = w // 2
        frozen = all(float(np.nanstd(a[half:])) < 0.8 for a in rc)
        moving_before = any(float(np.nanstd(a[:half])) > 2.0 for a in rc)
        if frozen and moving_before:
            ev.append(f"RC inputs frozen for the last {half / sr:.1f}s after prior activity")
    for idx, fields in data.get("_slow_frames", []):
        if idx >= s:
            flags = fields.get("flightModeFlags")
            try:
                if flags is not None and int(flags) & FMODE_FAILSAFE:
                    ev.append(f"FAILSAFE flag active at t={idx / sr:.1f}s")
                    break
            except (ValueError, TypeError):
                pass
    if ev:
        conf = "HIGH" if len(ev) >= 2 else "MEDIUM"
        verdicts.append({"cause": "rx_loss",
                         "label": t("pm.rx_loss"),
                         "confidence": conf, "evidence": ev})

    # ── Impact: gyro saturation right at log end ──
    last_s = max(1, int(sr))
    gyro_peak = 0.0
    for ax in ("gyro_roll", "gyro_pitch", "gyro_yaw"):
        g = data.get(ax)
        if g is not None and len(g) == n:
            tail = np.asarray(g[-last_s:], dtype=float)
            tail = tail[~np.isnan(tail)]
            if len(tail):
                gyro_peak = max(gyro_peak, float(np.max(np.abs(tail))))
    if gyro_peak > 1500:
        verdicts.append({"cause": "impact",
                         "label": t("pm.impact"),
                         "confidence": "HIGH" if gyro_peak > 1900 else "MEDIUM",
                         "evidence": [f"gyro peaked at {gyro_peak:.0f} deg/s within "
                                      f"the final second before the log ends"]})

    # ── Nav divergence: estimator vs GPS position split ──
    pts = [p for p in _extract_gps_points(data, sr) if p["idx"] >= s]
    pn, pe = data.get("nav_pos_n"), data.get("nav_pos_e")
    if len(pts) >= 3 and pn is not None and pe is not None and len(pn) == n:
        all_pts = _extract_gps_points(data, sr)
        home = all_pts[0] if all_pts else None
        if home:
            divs = []
            for p in pts:
                i = p["idx"]
                if i < n and not (np.isnan(pn[i]) or np.isnan(pe[i])):
                    gn, ge = _latlon_to_ne_m(p["lat"], p["lon"], home["lat"], home["lon"])
                    divs.append(math.hypot(gn - pn[i] / 100.0, ge - pe[i] / 100.0))
            if divs and max(divs) > 25 and divs[-1] >= max(divs) * 0.8:
                verdicts.append({"cause": "nav_divergence",
                                 "label": t("pm.nav_divergence"),
                                 "confidence": "MEDIUM",
                                 "evidence": [f"GPS vs estimator position split grew to "
                                              f"{max(divs):.0f}m in the final window"]})

    order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
    verdicts.sort(key=lambda v: order.get(v["confidence"], 3))
    return {"window_s": round(w / sr, 1), "t_end": round(t_end, 1),
            "abrupt_end": detect_abrupt_end(data, sr), "verdicts": verdicts}


def print_postmortem_report(pm, colors=None):
    R = B = G = Y = RED = DIM = ""
    if colors:
        R, B, G, Y, RED, DIM = colors
    print(f"\n  {B}{t('pm.title')}{R}")
    print(f"  {DIM}{t('pm.window', w=pm['window_s'], end=pm['t_end'])}{R}")
    if pm["abrupt_end"]:
        print(f"  {Y}⚠ {t('pm.abrupt')}{R}")
    if not pm["verdicts"]:
        print(f"  {G}✓ {t('pm.clean')}{R}\n")
        return
    print()
    for v in pm["verdicts"]:
        c = RED if v["confidence"] == "HIGH" else Y
        print(f"  {c}▸ {v['label']} — {v['confidence']}{R}")
        for e in v["evidence"]:
            print(f"      {DIM}{e}{R}")
    print(f"\n  {DIM}{t('pm.disclaimer')}{R}\n")
