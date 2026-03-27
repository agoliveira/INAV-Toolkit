# INAV Toolkit

A Python toolkit for analyzing blackbox logs, validating configurations, and tuning INAV flight controllers. Plug in your FC via USB - it pulls the config, downloads the blackbox, tells you exactly what to change, and gives you the CLI commands to paste.

Built for the INAV long-range community. Tested on 7" to 15" multirotors with GPS navigation, INAV 9.0.x.

## Install / Update

```bash
git clone https://github.com/agoliveira/INAV-Toolkit.git
cd INAV-Toolkit
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 -m inav_toolkit.blackbox_analyzer --device auto
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

That's it. The tool will auto-detect your FC, pull the current config, download the blackbox, split multi-flight logs, detect config changes between sessions, and run full analysis on the best flight.

### Output

```
  INAV Blackbox Analyzer v2.16.0
  Connected: NAZGUL 10 - INAV 9.0.1
  Pulling configuration (diff all)... 89 settings
  Downloading 2236KB ... 204KB/s
  Found 3 flights - analyzing best from latest config

  TUNE QUALITY: █████████████░░░░░░░ 65/100
    Noise:95 | PID:N/A | Motors:94

  CONFIG:
    Roll   P= 32  I= 82  D= 32  (FC now: P->35, I->75, D->28)
    Pitch  P= 35  I= 90  D= 35  (FC now: P->38, I->82, D->31)

  NOISE SOURCES:
    50Hz motor imbalance on Roll/Yaw [medium]
    128Hz motor noise on Pitch/Yaw [low]
    187-437Hz (6 peaks) electrical on Pitch/Roll/Yaw [medium]

  DO THIS - 2 changes:
    1. Lower dynamic_gyro_notch_min_hz: 50 -> 40
    2. Consider enabling RPM filter

  INAV CLI - paste into Configurator CLI tab:
    set dynamic_gyro_notch_min_hz = 40
    save

  CONFIG REVIEW:
    [CRITICAL] Critical beeper warnings disabled - BAT_CRIT_LOW, RX_LOST
```

## Tools

### Blackbox Analyzer

Decodes `.bbl` / `.bfl` blackbox logs natively in Python (no `blackbox_decode` needed) and produces actionable tuning recommendations.

```bash
# Connect to FC - pull config + download + analyze
python3 -m inav_toolkit.blackbox_analyzer --device auto

# Analyze an existing log file
python3 -m inav_toolkit.blackbox_analyzer flight.bbl

# With a saved diff for config comparison
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --diff my_diff.txt

# Specify frame size for tailored thresholds
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --frame 10

# Download only, analyze later
python3 -m inav_toolkit.blackbox_analyzer --device auto --download-only

# Flight progression history
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --history
```

**What it measures:**
- PID step response - overshoot %, tracking delay, settling time per axis
- Noise spectrum - identifies peak frequencies, classifies sources (propwash, electrical, motor imbalance), groups and ranks by severity
- Hover oscillation - RMS and peak-to-peak gyro during centered stick
- Motor balance - average load and saturation per motor
- Filter phase lag - total delay through the gyro/D-term filter chain
- Config mismatch - detects when the FC config has changed since the flight being analyzed

**What it outputs:**
- Terminal report with score, noise sources, specific CLI commands to paste
- HTML report with interactive charts (noise spectrum, PID response, motor traces)
- `state.json` for cross-referencing with the parameter analyzer
- SQLite flight database for progression tracking across sessions

### Parameter Analyzer

Validates an INAV `diff all` export for configuration issues. Catches problems that blackbox data alone can't detect.

```bash
# Check existing config
python3 -m inav_toolkit.param_analyzer my_diff.txt --frame 10

# Generate starting PIDs for a new build
python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S

# Compare starting PIDs with current config
python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S my_diff.txt

# Cross-reference with blackbox results
python3 -m inav_toolkit.param_analyzer my_diff.txt --blackbox state.json
```

**What it checks:** safety (beepers, failsafe, battery limits), motors (protocol, RPM filter, ESC telemetry), filters (frame-appropriate LPF, dynamic notch), PIDs (cross-profile consistency, antigravity), navigation (RTH altitude, position PIDs, safehome), GPS (constellations, compass cal), blackbox (logging rate, essential fields).

**Setup mode** generates conservative starting PIDs for 7/10/12/15" frames at 4S/6S/8S/12S.

### VTOL Configurator

Validates INAV VTOL configurations using mixer_profile switching.

```bash
python3 -m inav_toolkit.vtol_configurator vtol_diff.txt
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

Handles MSP v2 framing, pipelined dataflash download (4-deep), CLI command/batch, auto-reconnection after USB VCP reset.

## Tuning Workflow

```
1. NEW BUILD
   python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S
   -> Conservative starting PIDs and filter settings
   -> Paste CLI commands into INAV Configurator

2. SAFETY CHECK
   python3 -m inav_toolkit.param_analyzer my_diff.txt --frame 10
   -> Catches disabled beepers, failsafe issues, battery limits
   -> Fix before flying

3. FLY
   -> Enable blackbox logging (GYRO_RAW, MOTORS, RC_COMMAND)
   -> Hover for 10s, then do deliberate roll/pitch/yaw sweeps
   -> The analyzer needs stick inputs to measure PID response

4. ANALYZE
   python3 -m inav_toolkit.blackbox_analyzer --device auto
   -> Downloads log, pulls current config, runs full analysis
   -> Shows exactly what to change with CLI commands

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
│   ├── blackbox_analyzer.py     # Blackbox log analyzer
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

This is an active project. Planned: autonomous tuning loop (Pi Zero 2W or ESP32 strapped to the quad), expanded nav analysis, and web-based report viewer.

## License

MIT License. See [LICENSE](LICENSE).

## Acknowledgments

- The INAV development team and community
- QuadMeUp (Pawel Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
- UAV Tech for the spark that gave me the idea to create this
