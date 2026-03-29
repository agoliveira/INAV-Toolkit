# INAV Toolkit

A Python toolkit for analyzing blackbox logs, validating configurations, and tuning INAV flight controllers. Plug in your FC via USB — it runs a pre-flight safety check, pulls the config, downloads the blackbox, analyzes PID tuning and navigation performance, compares with your previous flight, tells you exactly what to change, and gives you the CLI commands to paste. Every config is archived automatically so you can diff any two sessions.

Built for the INAV long-range community. Tested on 7" to 15" multirotors with GPS navigation, INAV 9.0.x.

## Install / Update

```bash
pip install inav-toolkit
```

Or from source:

```bash
git clone https://github.com/agoliveira/INAV-Toolkit.git
cd INAV-Toolkit
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Or on Debian/Ubuntu, system packages:
```bash
sudo apt install python3-numpy python3-scipy python3-serial
```
Python 3.8+. `pyserial` only needed for direct FC connection.

> **Note:** If using a venv, activate it (`source .venv/bin/activate`) before running any commands below.

### Connect and Analyze

```bash
python3 -m inav_toolkit.blackbox_analyzer --device auto
```

That's it. The tool will:

1. Auto-detect your FC and pull the full config dump
2. Run a **pre-flight safety check** (disabled beepers, failsafe, battery limits)
3. Archive the config to the **backup vault** with timestamp
4. Download the blackbox and split multi-flight logs
5. Run PID tuning, noise, and navigation analysis
6. Compare with your **previous flight** and explain what changed
7. Clean up log files and erase flash
8. Open the **tabbed HTML report** in your browser

Results are presented in an interactive terminal menu:

```
  TUNE QUALITY: █████████░░░░░░░░░░░ 46/100
    Noise:94 | PID:26 | Motors:53

  [1] PID Tuning        R:62%OS  P:ok  Y:N/A
  [2] Noise Analysis    94/100 — 2 sources
  [3] Hover & Stability R:moderate  P:wind  Y:wind
  [4] Nav Performance   45/100 — 4 decel events, CEP 1271cm
  [5] Nav Sensors       Com:70  GPS:100  Bar:70
  [6] Config Review     1 critical, 2 warnings
  [7] Flight History    3 flights ▃▅▇
  [8] vs Previous       ↘ -8 (4 config changes)
  [C] CLI Commands      2 changes
  [A] Show All
  [Q] Quit

  Section (1-8, C, A, Q):
```

Each section shows a one-line status at a glance. Select a number to drill into details. Flight History shows trend sparklines. The CLI Commands section gives you paste-ready commands.

## Tools

### Blackbox Analyzer

Decodes `.bbl` / `.bfl` blackbox logs natively in Python (no `blackbox_decode` needed) and produces actionable tuning and navigation recommendations.

```bash
# Connect to FC — full pipeline (safety check + download + analyze + cleanup + erase)
python3 -m inav_toolkit.blackbox_analyzer --device auto

# Analyze an existing log file
python3 -m inav_toolkit.blackbox_analyzer flight.bbl

# With a saved dump/diff for config comparison
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --config my_dump.txt

# Specify frame size for tailored thresholds
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --frame 10

# Download only, analyze later
python3 -m inav_toolkit.blackbox_analyzer --device auto --download-only

# Keep raw log files (skip auto-cleanup)
python3 -m inav_toolkit.blackbox_analyzer --device auto --keep-logs

# Don't erase flash after download
python3 -m inav_toolkit.blackbox_analyzer --device auto --no-erase

# Don't open browser with HTML report
python3 -m inav_toolkit.blackbox_analyzer --device auto --no-browser

# Skip nav analysis even if nav fields are present
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --no-nav

# Flight progression history
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --history

# List archived configs
python3 -m inav_toolkit.blackbox_analyzer --config-history

# Compare two archived configs
python3 -m inav_toolkit.blackbox_analyzer --config-diff 1 3
```

**What it analyzes (auto-detected from available data):**

PID tuning:
- Step response — overshoot %, tracking delay, settling time per axis
- FeedForward-aware — attributes overshoot between FF and P, recommends FF reduction before P cuts
- Hover oscillation — RMS and peak-to-peak gyro, wind buffeting vs P oscillation on large frames
- Noise spectrum — peak frequencies, source classification (propwash, electrical, motor imbalance)
- Filter recommendations — phase-lag-aware, won't recommend destructive LPF cuts on 10"+ frames
- Motor balance — average load and saturation per motor

Navigation performance (when nav fields are present):
- Deceleration overshoot — measures position error when stopping, oscillation count, settling time
- Position hold quality — CEP, RMS error, max drift, toilet-bowl pattern detection
- Altitude hold quality — vertical error RMS, oscillation detection
- Controller saturation — detects when nav demands exceed platform capability
- Wind vs tuning — correlates position error with wind estimates to separate environmental drift from PID problems

Navigation sensors:
- Compass health — heading jitter, magnetic interference, drift rate
- GPS quality — fix type, satellite count, HDOP, position jumps
- Barometer — noise level, spike detection, throttle correlation
- Position estimator — GPS vs estimator divergence

**What it outputs:**

Terminal:
- Interactive menu with 8 sections (with `--device auto`) or sequential report
- Pre-flight safety checklist with CLI fix commands
- Flight-to-flight diff with causal verdict ("P increase caused more overshoot")
- Trend sparklines showing score progression across sessions

HTML report (tabbed, auto-opens in browser):
- **Overview** — score, actions, CLI commands, pre-flight warnings, config
- **PID Tuning** — step response table and chart
- **Noise & Filters** — gyro spectrum and D-term noise charts
- **Motors** — balance table and chart
- **Nav Performance** — deceleration, poshold, althold metrics
- **Nav Sensors** — compass, GPS, baro, estimator health
- **What-If** — interactive PID/filter simulation with predicted effects
- **History** — flight diff, score progression chart, flight table

Other:
- `state.json` for cross-referencing with the parameter analyzer
- SQLite flight database for progression tracking
- Config vault with timestamped backups of every FC dump

### Config Backup Vault

Every `--device auto` run archives the FC's full `dump all` with a timestamp. Over time you build a complete history of every config you've flown.

```bash
# List archived configs
python3 -m inav_toolkit.blackbox_analyzer --config-history

  CONFIG VAULT: 3 archived configs
    #  Date              Settings       Age  Craft
  ────────────────────────────────────────────────────────────
    1  2026-03-29 13:18       976    2h ago  NAZGUL 10
    2  2026-03-28 17:07       976    1d ago  NAZGUL 10
    3  2026-03-27 10:30       976    2d ago  NAZGUL 10

# Compare any two configs by number
python3 -m inav_toolkit.blackbox_analyzer --config-diff 1 3

  CONFIG DIFF: 12 differences, 964 unchanged
  Changed (10):
    mc_p_roll: 25 → 40  (+60%)
    dterm_lpf_hz: 60 → 110  (+83%)
    failsafe_procedure: RTH → DROP
    ...
```

### Parameter Analyzer

Validates an INAV config for issues. Accepts `dump all` or `diff all` output. Catches problems that blackbox data alone can't detect.

```bash
# Check existing config
python3 -m inav_toolkit.param_analyzer my_dump.txt --frame 10

# Generate starting PIDs for a new build
python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S

# Compare starting PIDs with current config
python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S my_dump.txt

# Cross-reference with blackbox results
python3 -m inav_toolkit.param_analyzer my_dump.txt --blackbox state.json
```

**What it checks:** safety (beepers, failsafe, battery limits), motors (protocol, RPM filter, ESC telemetry), filters (frame-appropriate LPF, dynamic notch), PIDs (cross-profile consistency, FF, antigravity), navigation (frame-aware nav PID thresholds, RTH altitude, deceleration time, position PIDs, safehome), GPS (constellations, compass cal), blackbox (logging rate, essential fields).

**Setup mode** generates conservative starting PIDs including FeedForward for 7/10/12/15" frames at 4S/6S/8S/12S.

### VTOL Configurator

Validates INAV VTOL configurations using mixer_profile switching.

```bash
python3 -m inav_toolkit.vtol_configurator vtol_config.txt
```

Checks MC/FW mixer profiles, tilt servo rules, motor role inference, yaw authority, FW control surfaces, mode assignments, automated RTH transition, and airmode conflicts.

### Guided Wizard

Interactive session manager that walks you through the full workflow.

```bash
python3 -m inav_toolkit.wizard
```

Connects to your FC and offers: tuning session, nav health check, new build safety check, blackbox download, or config restore from backup.

### MSP Module

Direct serial communication with INAV flight controllers. Used internally by the other tools, also usable standalone.

```bash
# Standalone: identify FC and download blackbox
python3 -m inav_toolkit.msp --device auto
```

Handles MSP v2 framing, pipelined dataflash download (4-deep), CLI command/batch, `dump all`/`diff all` config pull, auto-reconnection after USB VCP reset.

## Tuning Workflow

```
1. NEW BUILD
   python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S
   → Conservative starting PIDs, filters, FF, and nav settings
   → Paste CLI commands into INAV Configurator

2. SAFETY CHECK (automatic with --device auto)
   → Pre-flight checklist catches disabled beepers, failsafe issues
   → Flags nav PIDs that are INAV defaults (tuned for 5") on larger frames
   → Warns sternly with fix commands, doesn't block

3. FLY
   → Enable blackbox logging (GYRO_RAW, MOTORS, RC_COMMAND)
   → Hover for 10s, then do deliberate roll/pitch/yaw sweeps
   → Engage POSHOLD and RTH for nav analysis data

4. ANALYZE
   python3 -m inav_toolkit.blackbox_analyzer --device auto
   → Downloads log, pulls full config dump, archives to vault
   → Pre-flight safety check before download
   → Interactive menu with PID, noise, nav, config review sections
   → Compares with previous flight: "P increase caused more overshoot"
   → Shows exactly what to change with CLI commands
   → Opens tabbed HTML report with What-If simulation
   → Auto-cleans log files and erases flash when done

5. APPLY + REPEAT
   → Paste CLI commands, fly again
   → Tool auto-detects which flights use the new config
   → Flight database tracks progression with sparkline trends
   → Config vault archives every session for future reference
```

## Frame Size Profiles

The `--setup` mode provides conservative starting configurations:

| Frame | P Roll | P Pitch | D | FF | Gyro LPF | Dyn Notch Min |
|-------|--------|---------|---|-----|----------|---------------|
| 7" 4S | 35 | 38 | 23 | 60 | 90 Hz | 60 Hz |
| 10" 4S | 25 | 28 | 18 | 40 | 65 Hz | 50 Hz |
| 12" 4S | 20 | 22 | 14 | 30 | 50 Hz | 35 Hz |
| 15" 4S | 15 | 16 | 10 | 20 | 40 Hz | 25 Hz |

Higher voltage (6S/8S/12S) scales P and D down proportionally. Yaw FF also included (slightly higher than roll/pitch since yaw has no D-term).

## Project Structure

```
INAV-Toolkit/
├── inav_toolkit/
│   ├── __init__.py              # Package version
│   ├── blackbox_analyzer.py     # Blackbox analyzer, nav performance, What-If
│   ├── msp.py                   # MSP v2 serial communication
│   ├── flight_db.py             # SQLite flight history + flight diff
│   ├── param_analyzer.py        # Config validator + setup generator
│   ├── vtol_configurator.py     # VTOL mixer validator
│   ├── wizard.py                # Guided session manager
│   ├── autotune.py              # Autotune orchestrator (experimental)
│   ├── i18n.py                  # Internationalization
│   └── locales/                 # en, pt_BR, es
├── docs/
├── tests/
├── README.md
├── CHANGELOG.md
├── LICENSE                      # MIT
└── requirements.txt
```

## INAV Version Support

Developed and tested against **INAV 9.0.x**. The blackbox binary decoder handles the Cleanflight/INAV encoding format natively in Python — no external `blackbox_decode` tool needed. Parameter names are INAV-specific; Betaflight is not currently supported.

## Contributing

This is an active project. Planned: autonomous tuning loop (Pi Zero 2W or ESP32 strapped to the quad).

## License

MIT License. See [LICENSE](LICENSE).

## Acknowledgments

- The INAV development team and community
- QuadMeUp (Pawel Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
- UAV Tech for the spark that gave me the idea to create this
