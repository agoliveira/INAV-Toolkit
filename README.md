# INAV Toolkit

[![CI](https://github.com/agoliveira/INAV-Toolkit/actions/workflows/ci.yml/badge.svg)](https://github.com/agoliveira/INAV-Toolkit/actions/workflows/ci.yml)

A suite of Python tools for analyzing, validating, and tuning INAV flight controller configurations. Built for the INAV long-range community - 7" to 15" multirotors with GPS navigation.

## Tools

| Tool | Purpose |
|------|---------|
| **Blackbox Analyzer** | Decode blackbox logs, analyze PID performance, detect oscillation, recommend tuning changes |
| **Guided Wizard** | Interactive session manager - connects to FC, backs up config, runs analysis, applies changes |
| **Parameter Analyzer** | Validate `diff all` configs for safety, filter, PID, and navigation issues |
| **VTOL Configurator** | Validate VTOL mixer profiles, motor/servo mixing, and transition setup |
| **MSP Communication** | Direct FC communication - download blackbox, pull config, identify hardware |

## Quick Start

### Install

```bash
pip install inav-toolkit
```

That's it. All dependencies (numpy, scipy, matplotlib, pyserial) are installed automatically.

**Alternative install methods:**

```bash
# In a virtual environment
python3 -m venv inav
source inav/bin/activate
pip install inav-toolkit

# With pipx (auto-isolates CLI tools)
pipx install inav-toolkit

# From source (development)
git clone https://github.com/agoliveira/INAV-Toolkit
cd INAV-Toolkit
pip install -e .
```

Python 3.8+ required.

### Blackbox Analyzer

Analyzes binary blackbox logs (`.bbl` / `.bfl`) from INAV. Decodes natively in Python - no `blackbox_decode` needed.

```bash
# Full pipeline: connect to FC, download blackbox, analyze (diff pulled automatically)
inav-analyze --device auto

# Download, analyze, and erase flash for next session
inav-analyze --device auto --erase

# Download only (saves to ./blackbox/, no analysis)
inav-analyze --device auto --download-only

# Analyze from file (auto-detects frame size, cells, platform)
inav-analyze flight.bbl

# Multi-log files are automatically split and analyzed individually
inav-analyze dataflash_dump.bbl

# Show flight history and progression for a craft
inav-analyze flight.bbl --history

# Override frame size if auto-detection doesn't apply
inav-analyze flight.bbl --frame 10

# Add motor KV for RPM noise prediction (cells auto-detected from vbatref)
inav-analyze flight.bbl --kv 980

# Navigation health analysis mode
inav-analyze flight.bbl --nav

# Compare two flights side-by-side
inav-analyze flight_before.bbl --compare flight_after.bbl

# Interactive replay with Plotly.js (spectrogram, synced zoom, flight modes)
inav-analyze flight.bbl --replay

# Quick log quality check (exit code 1 if unusable)
inav-analyze flight.bbl --check-log

# Generate Markdown report for forum/Discord
inav-analyze flight.bbl --report md

# Output in Portuguese or Spanish
inav-analyze flight.bbl --lang pt_BR
inav-analyze flight.bbl --lang es

# Skip database storage for one-off analysis
inav-analyze flight.bbl --no-db

# Custom database path
inav-analyze flight.bbl --db-path ~/my_flights.db
```

**What it measures:**
- **Hover oscillation:** Detects oscillation during hover from gyro variance, classifies severity, identifies dominant frequency, diagnoses cause (P too high / PD interaction / D-term noise / filter gap)
- **PID step response:** Overshoot percentage, settling time, delay
- **Noise spectrum:** Identifies peak frequencies per axis
- **Motor balance:** Detects thrust asymmetry
- **Filter phase lag:** Total delay through the filter chain
- **Tracking error:** RMS deviation between setpoint and gyro
- **Navigation health** (`--nav`): Compass interference, GPS quality, barometer noise, position hold drift, altitude hold accuracy

**Comparative analysis** (`--compare`): Side-by-side analysis of two flights with overlay noise spectra, PID response deltas, motor balance comparison, and overall score improvement/degradation arrow. Warns on craft/frame mismatches.

**Interactive replay** (`--replay`): Standalone HTML with Plotly.js WebGL rendering. Panels: gyro vs setpoint (3 axes), motor outputs, noise spectrogram waterfall (sliding FFT heatmap), throttle. Synced zoom/pan across all panels. Flight mode overlay bar decoded from S-frame bitmask.

**Log quality scoring** (`--check-log`): Pre-analysis validation that grades logs as GOOD/MARGINAL/UNUSABLE. Checks: duration, sample rate, field completeness, stick activity, all-zeros sensors, corrupt frames, NaN gaps. Also runs automatically during normal analysis.

**Markdown reports** (`--report md`): Generates a forum/Discord-pasteable report with scores, findings, noise sources, PID metrics, and CLI commands in a fenced code block.

**Localization** (`--lang`): Output in English (default), Brazilian Portuguese (`pt_BR`), or Spanish (`es`). Auto-detects from `INAV_LANG` environment variable or system locale. Technical terms (PID, Hz, Roll/Pitch/Yaw, CLI commands) stay untranslated. Add new languages by creating a JSON file in `inav_toolkit/locales/`.

**Multi-log splitting:** Dataflash dumps containing multiple arm/disarm cycles are automatically detected and split. Each flight is analyzed individually with per-flight progression tracking.

**CLI diff merge:** When connected to the FC via `--device`, the analyzer automatically pulls the full `diff all` configuration. Settings not present in blackbox headers (motor_poles, nav PIDs, rates, level mode, antigravity) are enriched from the diff. Mismatches between what was flying and the current FC config are detected and displayed.

**Auto-detection:** Frame size from craft name, battery cells from vbatref, platform type (quad/hex/tri) from motor count. Warns on conflicts between detected and user-specified values.

**Oscillation-first enforcement:** When hover oscillation is detected, the analyzer generates aggressive P/D reductions as top-priority actions and defers regular PID recommendations on oscillating axes - you can't tune what you can't stabilize.

**Filter-first enforcement:** When filter changes are needed, PID recommendations are deferred until after filters are fixed and a re-fly, preventing the common death spiral of raising D-term gains into wide-open filters.

**Output:** Terminal report with actionable CLI commands, HTML report with plots, state JSON for cross-referencing, and SQLite database for history.

### Guided Wizard

Interactive session that orchestrates the full workflow: connect to FC, backup config, download blackbox, analyze, review changes, and apply.

```bash
# Start guided session (auto-detects FC)
inav-toolkit

# Specify serial port
inav-toolkit --device /dev/ttyACM0
```

The wizard creates a timestamped backup before any changes and walks you through each step with safety gates.

### Parameter Analyzer

Validates an INAV `diff all` export for configuration issues.

```bash
# Analyze existing config
inav-params my_diff.txt --frame 10

# Generate starting PIDs for a new build
inav-params --setup 10 --voltage 6S

# Compare starting PIDs with current config
inav-params --setup 10 --voltage 6S my_diff.txt

# Cross-reference with blackbox results
inav-params my_diff.txt --blackbox state.json
```

**What it checks:**
- **Safety:** Beeper configuration, failsafe procedure, battery limits
- **Motors:** Protocol validation, RPM filter + ESC telemetry consistency
- **Filters:** Frame-appropriate gyro LPF, dynamic notch configuration
- **PIDs:** Cross-profile consistency, iterm_relax, D-boost, antigravity
- **Navigation:** RTH altitude, hover throttle, position PIDs, safehome
- **GPS:** Multi-constellation, compass calibration quality (mag gain spread)
- **Blackbox:** Logging rate, essential fields for analysis
- **Battery:** Li-ion vs LiPo detection, voltage limits

**Setup mode** generates conservative starting PIDs for 5/7/10/12/15" frames at 4S/6S/8S/12S.

### VTOL Configurator

Validates INAV VTOL configurations using mixer_profile switching.

```bash
# Validate VTOL config
python3 -m inav_toolkit.vtol_configurator vtol_diff.txt

# JSON output
python3 -m inav_toolkit.vtol_configurator vtol_diff.txt --json
```

**What it checks:**
- MC and FW mixer profiles present with correct `platform_type`
- Motor role inference: tilt / lift-only / pusher by cross-referencing profiles
- Tilt servo rules (smix source 38) for transition
- Yaw authority: motor yaw mixing or tilt servo yaw
- FW control surfaces: roll + pitch servo mixing
- Mode assignments: MIXER PROFILE 2 (mode 62) and MIXER TRANSITION (mode 63)
- Automated RTH transition: `mixer_automated_switch` in both profiles
- Safety: `airmode_type` conflict with transition motors, compass for MC nav
- Control profile linking between mixer profiles

### MSP Communication

Direct serial communication with INAV flight controllers over USB.

```bash
# Identify connected FC
inav-msp --info-only

# Download blackbox data
inav-msp --device auto

# Specify serial port
inav-msp --device /dev/ttyACM0
```

**Capabilities:**
- MSP v2 protocol with CRC-8 validation
- FC identification (craft name, firmware, board, build date)
- Dataflash summary, read, and erase
- SD card blackbox detection with guidance
- CLI mode for `diff all` config retrieval
- Auto-detection of serial ports (Linux/macOS)
- Binary blackbox download with progress bar and retry logic

## Tuning Workflow

The tools are designed to work together in a tuning pipeline:

```
1. NEW BUILD
   └─ inav-params --setup 10 --voltage 6S
      → Conservative starting PIDs, filters, settings
      → Paste CLI commands into INAV Configurator

2. FIRST FLIGHTS
   └─ inav-params my_diff.txt --frame 10
      → Catches safety issues, missing settings, profile inconsistencies
      → Fix before flying

3. BLACKBOX LOGGING
   └─ Fly with blackbox enabled (GYRO_RAW, MOTORS, RC_COMMAND)
      → Hover segment for oscillation detection
      → Manual/acro segment for PID step response data
      → Nav segment for position hold / RTH data

4. PID TUNING (one-step with FC connected)
   └─ inav-analyze --device auto --erase
      → Downloads blackbox, pulls current config
      → Splits multi-log files automatically
      → Detects hover oscillation first
      → Recommends filter fixes before PID changes
      → Stores results in flight database
      → Shows progression from previous flights

5. GUIDED SESSION (interactive)
   └─ inav-toolkit
      → Connects, backs up, downloads, analyzes, applies
      → Safety gates at every step

6. ITERATE
   └─ Apply CLI commands → fly → analyze → repeat
      → Database tracks score progression across sessions
      → --history shows the full tuning journey
      → Mismatch detection catches config drift
```

## Frame Size Profiles

The `--setup` mode provides conservative starting configurations:

| Frame | P Roll | P Pitch | D | Gyro LPF | Dyn Notch Min |
|-------|--------|---------|---|----------|---------------|
| 5" 4S  | 44 | 46 | 30 | 120Hz | 100Hz |
| 7" 4S  | 35 | 38 | 23 | 90Hz | 60Hz |
| 10" 4S | 25 | 28 | 18 | 65Hz | 50Hz |
| 12" 4S | 20 | 22 | 14 | 50Hz | 35Hz |
| 15" 4S | 15 | 16 | 10 | 40Hz | 25Hz |

Higher voltage (6S/8S/12S) scales P and D down proportionally.

## INAV Version Support

Developed and tested against **INAV 9.0.x**. The blackbox binary decoder handles the shared Cleanflight/INAV encoding format. MSP v2 protocol for FC communication. Parameter names are INAV-specific - this toolkit does not support Betaflight.

## Project Structure

```
inav-toolkit/
├── pyproject.toml                   # Package configuration (pip install)
├── requirements.txt                 # Python dependencies
├── README.md
├── CHANGELOG.md
├── LICENSE
├── inav_toolkit/                    # Python package
│   ├── __init__.py                  # Version: 2.15.1
│   ├── blackbox_analyzer.py         # Blackbox log analyzer
│   ├── param_analyzer.py            # Config validator + setup generator
│   ├── msp.py                       # MSP v2 serial communication
│   ├── wizard.py                    # Guided session manager
│   ├── i18n.py                      # Localization system
│   ├── locales/                     # Translation catalogs (en, pt_BR, es)
│   ├── autotune.py                  # Experimental auto-tuning
│   └── vtol_configurator.py         # VTOL mixer profile validator
├── docs/
│   ├── BLACKBOX_ANALYZER.md         # Detailed blackbox analyzer docs
│   ├── NAV_ANALYZER.md              # Navigation health analysis docs
│   ├── PARAM_ANALYZER.md            # Detailed parameter analyzer docs
│   ├── VTOL_CONFIGURATOR.md         # Detailed VTOL configurator docs
│   └── TUNING_WORKFLOW.md           # Step-by-step tuning guide
├── tests/
│   ├── test_smoke.py                # 90 tests across 13 test classes
│   ├── conftest.py                  # Shared pytest fixtures
│   ├── generate_fixtures.py         # Synthetic blackbox CSV generator
│   ├── test_basic_diff.txt          # Sample diff for param_analyzer tests
│   └── test_vtol_diff.txt           # Sample diff for VTOL tests
└── .github/workflows/ci.yml        # CI: Python 3.8-3.13 test matrix
```

## Development

```bash
git clone https://github.com/agoliveira/INAV-Toolkit
cd INAV-Toolkit
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[test]"

# Generate synthetic test fixtures
python3 tests/generate_fixtures.py

# Run tests
python3 -m pytest tests/ -v
```

CI runs automatically on every push and PR across Python 3.8-3.13.

## Contributing

This is an active project. Contributions welcome.

## License

GPL-3.0 License. See [LICENSE](LICENSE).

## Acknowledgments

- [UAV Tech](https://www.youtube.com/@uavtech) — the YouTube channel that inspired this project
- The INAV development team and community
- QuadMeUp (Paweł Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
