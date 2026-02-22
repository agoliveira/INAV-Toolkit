# Navigation Analyzer - Detailed Documentation

## Overview

The `--nav` flag activates a separate analysis mode that checks navigation sensor health from any blackbox log. It does not run PID tuning - it evaluates whether your compass, GPS, barometer, and position estimator are healthy enough for autonomous flight modes (althold, poshold, RTH, waypoints).

This works on any flight type including manual/acro. You don't need to fly poshold or RTH to check sensor health.

## Usage

### Recommended: Direct from FC

```bash
# Download blackbox + pull config + nav analysis (preferred)
python3 inav_blackbox_analyzer.py --device auto --nav
```

This is the best workflow. The analyzer connects to the FC, downloads the blackbox log, pulls the full configuration, and runs nav analysis with config cross-referencing. Cross-referenced findings combine flight data with FC settings to produce actionable results - for example, warning that high baro noise combined with aggressive altitude PID gains will cause oscillation.

### From file

```bash
# Nav health analysis
python3 inav_blackbox_analyzer.py flight.bbl --nav

# With explicit diff file for config enrichment
python3 inav_blackbox_analyzer.py flight.bbl --nav --diff diff_all.txt

# Analyze specific flight from multi-log flash dump
python3 inav_blackbox_analyzer.py dataflash.bbl --nav
```

**Auto-discovery:** If a `*_diff.txt` file exists in the same directory as the BBL (saved automatically by `--device` mode), it is loaded without needing `--diff`. To save a diff manually: open the CLI tab in INAV Configurator, type `diff all`, copy the output, save to a text file next to your BBL.

When `--nav` is used, only navigation analysis runs. No PID response, noise spectrum, motor balance, filter phase lag, or tuning recommendations are produced.

For multi-log flash dumps, nav mode skips the flight comparison table and analyzes the last flight directly.

## Prerequisites

**Tune PIDs first.** The nav analyzer checks for PID oscillation before running. If moderate or severe oscillation is detected, it warns that sensor readings are unreliable due to vibration:

```
! PID tuning incomplete - severe oscillation detected (Roll 91 deg/s RMS)
  Nav readings (especially baro and compass) are affected by vibration.
  Run without --nav first to fix PIDs, then re-check nav health.
```

Compass jitter and baro noise measured during PID oscillation reflect frame vibration, not actual sensor problems. Fix tuning first, then validate nav health.

## Blackbox Configuration

For nav analysis, enable these blackbox debug fields in INAV Configurator or CLI:

```
set blackbox_rate_denom = 2
blackbox NAV_POS
blackbox NAV_ACC
blackbox ATTI
blackbox ACC
blackbox MAG
blackbox RC_COMMAND
blackbox MOTORS
save
```

NAV_POS provides navigation position/velocity/target data. ATTI provides heading. ACC provides accelerometer data used by the estimator. MAG enables compass logging. MOTORS and RC_COMMAND enable throttle correlation analysis.

GPS data (satellite count, position) is logged automatically in G-frames when GPS is enabled in INAV.

## Analysis Modules

### Compass Health

**What it checks:**

- **Heading jitter** - Standard deviation of heading rate during steady flight (filtering out intentional turns >30 deg/s). Measured at ~50Hz to match compass update rate, avoiding quantization artifacts from 1000Hz logging. Good: <2 deg/s. Concerning: >5 deg/s.

- **Motor EMI correlation** - Pearson correlation between throttle changes and heading jumps. High correlation (>0.4) indicates electromagnetic interference from motors/ESCs affecting the magnetometer. This is the most common compass problem on multirotors.

- **Heading drift** - Average heading drift rate over the flight. >1 deg/s indicates calibration issues or persistent interference.

**Scoring:**
| Condition | Penalty |
|-----------|---------|
| Jitter > 5 deg/s | -40 |
| Jitter > 2 deg/s | -15 |
| Throttle correlation > 0.4 | -30 |
| Throttle correlation > 0.2 | -10 |
| Drift > 1 deg/s | -15 |

**Common causes of compass problems:**
- Magnetometer too close to power wires or battery leads
- ESC wires running near the GPS/compass module
- Low-quality compass module with poor shielding
- Missing or incorrect compass calibration
- Metal frame components near the compass

### GPS Quality

**What it checks:**

- **EPH (Estimated Position Horizontal)** - Horizontal position accuracy estimate from the navigation filter. Logged in I-frames as `navEPH` in centimeters. Good: <200cm. Poor: >500cm.

- **Satellite count** - Extracted from decoded GPS G-frames. Reports average and minimum count during the flight. Minimum <6 is dangerous for nav modes.

- **Position jumps** - Sudden position changes >2m between consecutive samples. Indicates GPS multipath, signal loss, or receiver glitches.

- **GPS update rate** - Computed from G-frame timestamps. Most GPS modules update at 5-10Hz.

**Scoring:**
| Condition | Penalty |
|-----------|---------|
| EPH avg > 500cm | -40 |
| EPH avg > 300cm | -15 |
| Min sats < 6 | -20 |
| Avg sats < 8 | -10 |
| Position jumps > 5 | -25 |
| Position jumps > 0 | -5 |

### Barometer Quality

**What it checks:**

- **Baro noise** - RMS of detrended barometric altitude. A zero-phase Butterworth low-pass at 0.5Hz removes intentional climbs/descents, leaving only noise. Good: <30cm RMS. Bad: >100cm RMS.

- **Propwash correlation** - Correlation between throttle and baro noise. High correlation (>0.5) means propwash is reaching the baro sensor, causing false altitude readings.

- **Spike detection** - Counts samples exceeding 5 sigma or 1m (whichever is larger). Spikes cause sudden altitude jumps in althold.

**Scoring:**
| Condition | Penalty |
|-----------|---------|
| Noise > 100cm RMS | -40 |
| Noise > 30cm RMS | -15 |
| Spikes > 3 | -15 |
| Throttle correlation > 0.5 | -20 |

**Common causes of baro noise:**
- Missing foam cover over the baro sensor
- Propwash reaching the baro through open frame
- Inadequate sealing around the baro sensor
- High vibration (fix PIDs first)

### Estimator Health

**What it checks:**

- **Baro vs nav altitude correlation** - Pearson correlation between `baro_alt` and `nav_pos_u` (the navigation filter's altitude estimate). Should be >0.95. Low correlation means the estimator is diverging from the barometer, which can cause dangerous altitude errors in althold/RTH.

- **Max divergence** - Peak difference between normalized baro and nav altitude. >1000cm is a serious problem.

**Scoring:**
| Condition | Penalty |
|-----------|---------|
| Correlation < 0.8 | -40 |
| Correlation < 0.95 | -10 |
| Max divergence > 1000cm | -30 |

### Altitude Hold (Phase-Gated)

Only runs when the flight log contains sustained nav-controlled phases (navState > 1 for >5 seconds). Does not run on manual flights to avoid false positives.

**What it checks:**

- **Altitude oscillation** - Peak-to-peak deviation of `nav_pos_u` from `nav_tgt_u`. Good: <50cm. Bad: >200cm.

- **Z velocity noise** - Standard deviation of `nav_vel_u` during hover. Should be near zero.

### Position Hold (Phase-Gated)

Only runs when nav phases are detected. Same gating as altitude hold.

**What it checks:**

- **CEP (Circular Error Probable)** - 50th percentile of horizontal position error from target. Measures how tightly the quad holds position.

- **Max drift** - Maximum distance from target position.

- **Toilet bowl detection** - Two methods:
  1. **Angular rotation**: If the position error vector steadily rotates at >0.1 rad/s with amplitude >50cm, it's toilet-bowling. This is the classic compass-interference drift pattern.
  2. **Spectral analysis**: FFT of position error looking for a dominant oscillation in 0.05-0.5 Hz range with >40% of total spectral power.

**Toilet bowl / salad bowl** is caused by compass heading error that rotates the coordinate frame of the position controller. The quad drifts in a circle because it thinks "north" is slowly rotating. Fix the compass first.

## Nav Score

The overall nav score is a weighted average:

| Check | Weight |
|-------|--------|
| Compass | 30% |
| GPS | 25% |
| Barometer | 25% |
| Estimator | 20% |

Althold and poshold scores are reported separately when available but do not contribute to the overall nav score (since they require specific flight modes).

## Config Cross-Referencing

When FC configuration is available (via `--device` or `--diff`), the analyzer combines flight data with config settings to produce findings that neither source can produce alone:

**Baro noise + altitude PIDs:** High baro noise with aggressive `nav_mc_pos_z_p` or `nav_mc_vel_z_p` will cause altitude oscillation in althold. The analyzer warns and suggests reducing gains or fixing baro foam.

**Compass jitter + mag calibration:** High heading jitter with uneven mag gain spread (>30%) confirms bad calibration. The analyzer recommends recalibrating outdoors.

**Compass jitter + heading P gain:** High jitter amplified by aggressive `nav_mc_heading_p` causes position controller instability. The analyzer suggests fixing the compass first or reducing the gain.

**GPS satellites + GNSS constellations:** Low satellite count with Galileo/Beidou/GLONASS disabled. The analyzer suggests enabling additional constellations with exact CLI commands.

**No compass configured:** `mag_hardware=NONE` means poshold and RTH rely on GPS course only, which is unreliable at low speed and during hover.

**Estimator divergence + baro weight:** Low baro-estimator correlation with low `inav_w_z_baro_p` suggests the estimator isn't trusting the barometer enough.

Without config data, the analyzer still runs all sensor checks and reports scores, but cannot produce these combined findings.

## Output

### Terminal

```
NAV HEALTH
  Compass      100/100  jitter 0.8 deg/s, motor corr 0.05
  GPS          100/100  14 sats avg, EPH 65cm
  Baro          85/100  noise 22cm RMS
  Estimator    100/100  baro correlation 0.987
  Nav Score     96/100
```

### HTML Report

A standalone `_nav_report.html` file with dark theme containing:
- Nav score card
- Per-check scores with details
- Findings with severity indicators
- Tuning prerequisite warning (when applicable)

## Recommended Flight Pattern

For nav health validation:

1. **Hover in manual/angle mode**, 20-30 seconds - gives clean compass and baro readings without nav controller interference
2. **Gentle yaw rotation** (one full 360) - tests compass for dead zones
3. **Throttle punches** (2-3 short ones) - tests for motor EMI on compass and propwash on baro
4. **Altitude changes** (climb 20m, descend) - tests baro tracking and estimator convergence

For poshold/RTH validation (once nav score is good):

1. **Position hold hover**, 30-60 seconds - tests position holding accuracy, detects toilet bowl
2. **Altitude hold**, 30 seconds - tests altitude stability
3. **Fly out 100-200m, trigger RTH** - tests full return navigation
4. **Let it complete landing** - tests landing detection

## Decoder Details

The nav analyzer uses fields from three frame types:

**I/P frames (main loop rate):**
- `navPos[0-2]` - Navigation position estimate (N/E/U, centimeters)
- `navVel[0-2]` - Navigation velocity estimate (N/E/U, cm/s)
- `navTgtPos[0-2]` - Navigation target position
- `navTgtVel[0-2]` - Navigation target velocity
- `navAcc[0-2]` - Navigation accelerometer
- `attitude[0-2]` - Roll, pitch, heading (decidegrees)
- `BaroAlt` - Barometric altitude (centimeters)
- `navState` - Current navigation state machine value
- `navFlags` - Navigation status flags
- `navEPH` / `navEPV` - Position accuracy estimates

**S-frames (slow, ~10Hz):**
- Flight mode flags (ANGLE, HORIZON, NAV_ALTHOLD, NAV_POSHOLD, etc.)

**G-frames (GPS rate, ~5-10Hz):**
- `GPS_numSat` - Satellite count
- `GPS_coord[0-1]` - Latitude/longitude
- `GPS_altitude` - GPS altitude
- `GPS_speed` - Ground speed
- `GPS_ground_course` - Track over ground

## Troubleshooting

**"No navigation fields found in this log"**
Enable NAV_POS and ATTI in blackbox settings. These are not enabled by default.

**All scores 100 but nav modes misbehave**
The analyzer checks sensor health, not nav PID tuning. If sensors are healthy but poshold drifts, the nav PIDs need adjustment (nav_mc_pos_xy_p, nav_mc_vel_xy_pid, etc.).

**Baro noise high but foam is installed**
Check that foam completely seals the sensor with no gaps. Even small openings let propwash through. Also check that PIDs are tuned - frame vibration inflates baro readings.

**Compass jitter high but module is far from motors**
Check ESC wire routing. Current-carrying wires create magnetic fields proportional to current, which change with throttle. Route power wires as far from the compass as possible, and twist positive/negative pairs to cancel their fields.
