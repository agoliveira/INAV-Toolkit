# INAV Toolkit

A Python toolkit for analyzing blackbox logs, validating configurations, and tuning INAV flight controllers. Plug in your FC via USB - it pulls the config, downloads the blackbox, analyzes PID tuning and navigation performance, tells you exactly what to change, and gives you the CLI commands to paste.

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

That's it. The tool will auto-detect your FC, pull the full config dump, download the blackbox, split multi-flight logs, run PID tuning and navigation analysis, clean up log files, and erase flash. Results are presented in an interactive menu:

```
  TUNE QUALITY: █████████░░░░░░░░░░░ 46/100
    Noise:94 | PID:26 | Motors:53

  [1] PID Tuning        R:62%OS  P:ok  Y:N/A
  [2] Noise Analysis    94/100 - 2 sources
  [3] Hover & Stability R:moderate  P:wind  Y:wind
  [4] Nav Performance   45/100 - 4 decel events, CEP 1271cm
  [5] Nav Sensors       Com:70  GPS:100  Bar:70
  [6] Config Review     1 critical, 2 warnings
  [7] Flight History    available
  [C] CLI Commands      2 changes
  [A] Show All
  [Q] Quit

  Section (1-7, C, A, Q):
```

Each section shows a one-line status at a glance. Select a number to drill into details. The CLI Commands section gives you paste-ready commands for INAV Configurator.

## Tools

### Blackbox Analyzer

Decodes `.bbl` / `.bfl` blackbox logs natively in Python (no `blackbox_decode` needed) and produces actionable tuning and navigation recommendations.

```bash
# Connect to FC - pull config + download + analyze + cleanup + erase
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

# Archive analyzed flight (compress to blackbox/archive/)
python3 -m inav_toolkit.blackbox_analyzer --device auto --archive

# Skip nav analysis even if nav fields are present
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --no-nav

# Flight progression history
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --history
```

**What it analyzes (auto-detected from available data):**

PID tuning:
- Step response - overshoot %, tracking delay, settling time per axis
- FeedForward-aware - attributes overshoot between FF and P, recommends FF reduction before P cuts
- Hover oscillation - RMS and peak-to-peak gyro, wind buffeting vs P oscillation on large frames
- Noise spectrum - peak frequencies, source classification (propwash, electrical, motor imbalance)
- Filter recommendations - phase-lag-aware, won't recommend destructive LPF cuts on 10"+ frames
- Motor balance - average load and saturation per motor

Navigation performance (when nav fields are present):
- Deceleration overshoot - measures position error when stopping, oscillation count, settling time
- Position hold quality - CEP, RMS error, max drift, toilet-bowl pattern detection
- Altitude hold quality - vertical error RMS, oscillation detection
- Controller saturation - detects when nav demands exceed platform capability
- Wind vs tuning - correlates position error with wind estimates to separate environmental drift from PID problems

Navigation sensors:
- Compass health - heading jitter, magnetic interference, drift rate
- GPS quality - fix type, satellite count, HDOP, position jumps
- Barometer - noise level, spike detection, throttle correlation
- Position estimator - GPS vs estimator divergence

**What it outputs:**
- Interactive terminal menu (with `--device auto`) or sequential report
- HTML report with interactive charts (noise spectrum, PID response, motor traces)
- `state.json` for cross-referencing with the parameter analyzer
- SQLite flight database for progression tracking across sessions

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

**Setup mode** generates conservative starting PIDs for 7/10/12/15" frames at 4S/6S/8S/12S.

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
   -> Conservative starting PIDs, filters, and nav settings
   -> Paste CLI commands into INAV Configurator

2. SAFETY CHECK
   python3 -m inav_toolkit.param_analyzer my_dump.txt --frame 10
   -> Catches disabled beepers, failsafe issues, battery limits
   -> Flags nav PIDs that are INAV defaults (tuned for 5") on larger frames
   -> Fix before flying

3. FLY
   -> Enable blackbox logging (GYRO_RAW, MOTORS, RC_COMMAND)
   -> Hover for 10s, then do deliberate roll/pitch/yaw sweeps
   -> Engage POSHOLD and RTH for nav analysis data

4. ANALYZE
   python3 -m inav_toolkit.blackbox_analyzer --device auto
   -> Downloads log, pulls full config dump, runs all analysis
   -> Interactive menu with PID, noise, nav, config review sections
   -> Shows exactly what to change with CLI commands
   -> Auto-cleans log files and erases flash when done

5. APPLY + REPEAT
   -> Paste CLI commands, fly again
   -> Tool auto-detects which flights use the new config
   -> Flight database tracks your progression across sessions
```

## Frame Size Profiles

The `--setup` mode provides conservative starting configurations:

| Frame | P Roll | P Pitch | D | Gyro LPF | Dyn Notch Min |
|-------|--------|---------|---|----------|---------------|
| 7" 4S | 35 | 38 | 23 | 90 Hz | 60 Hz |
| 10" 4S | 25 | 28 | 18 | 65 Hz | 50 Hz |
| 12" 4S | 20 | 22 | 14 | 50 Hz | 35 Hz |
| 15" 4S | 15 | 16 | 10 | 40 Hz | 25 Hz |

Higher voltage (6S/8S/12S) scales P and D down proportionally.

## Project Structure

```
INAV-Toolkit/
├── inav_toolkit/
│   ├── __init__.py              # Package version
│   ├── blackbox_analyzer.py     # Blackbox log analyzer + nav performance
│   ├── msp.py                   # MSP v2 serial communication
│   ├── flight_db.py             # SQLite flight history
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

Developed and tested against **INAV 9.0.x**. The blackbox binary decoder handles the Cleanflight/INAV encoding format natively in Python - no external `blackbox_decode` tool needed. Parameter names are INAV-specific; Betaflight is not currently supported.

## Contributing

This is an active project. Planned: autonomous tuning loop (Pi Zero 2W or ESP32 strapped to the quad) and web-based report viewer.

## License

MIT License. See [LICENSE](LICENSE).

## Acknowledgments

- The INAV development team and community
- QuadMeUp (Pawel Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
- UAV Tech for the spark that gave me the idea to create this
