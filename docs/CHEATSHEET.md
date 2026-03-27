# INAV Toolkit - Cheat Sheet

## Daily Use

```bash
# Plug in FC, do everything automatically
python3 -m inav_toolkit.blackbox_analyzer --device auto

# Download only (analyze later)
python3 -m inav_toolkit.blackbox_analyzer --device auto --download-only

# Erase flash after download
python3 -m inav_toolkit.blackbox_analyzer --device auto --erase

# Analyze an existing log
python3 -m inav_toolkit.blackbox_analyzer flight.bbl

# Analyze with a saved diff file
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --diff my_diff.txt

# Specify frame size (if not auto-detected from craft name)
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --frame 10

# Flight history / progression
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --history

# Skip HTML report
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --no-html

# JSON output for scripting
python3 -m inav_toolkit.blackbox_analyzer flight.bbl --json
```

## Config Validation

```bash
# Check a diff all export
python3 -m inav_toolkit.param_analyzer my_diff.txt --frame 10

# Generate starting PIDs for a new build
python3 -m inav_toolkit.param_analyzer --setup 10 --voltage 6S

# Cross-reference with blackbox analysis
python3 -m inav_toolkit.param_analyzer my_diff.txt --blackbox state.json
```

## Other Tools

```bash
# Guided wizard (interactive)
python3 -m inav_toolkit.wizard

# VTOL config validation
python3 -m inav_toolkit.vtol_configurator vtol_diff.txt

# MSP standalone (FC info + download)
python3 -m inav_toolkit.msp --device auto
```

## Workflow

```
1. param_analyzer --setup 10 --voltage 6S    # new build starting PIDs
2. param_analyzer diff.txt --frame 10        # safety check before flying
3. Fly: hover 10s, then roll/pitch/yaw sweeps
4. blackbox_analyzer --device auto           # analyze + get CLI commands
5. Paste CLI commands, fly again             # tool tracks progression
```

## Reading the Output

```
TUNE QUALITY: █████████████░░░░░░░ 65/100    <- overall score
  Noise:95 | PID:N/A | Motors:94              <- sub-scores

STALE DATA: FC config has changed...          <- fly again with current config

CONFIG:
  Roll  P=32 I=82 D=32 (FC now: P->35)       <- blackbox vs current FC

NOISE SOURCES:
  50Hz motor imbalance [medium]               <- grouped by type, top 3
  128Hz motor noise [low]
  187-437Hz (6 peaks) electrical [medium]

DO THIS - 2 changes:                          <- actionable recommendations
  1. Lower dynamic_gyro_notch_min_hz: 50->40
  2. Consider enabling RPM filter

INAV CLI:                                     <- copy-paste into Configurator
  set dynamic_gyro_notch_min_hz = 40
  save

MEASUREMENTS:                                 <- raw data
  Roll   Hover: stable  RMS: 0.5 deg/s
  Roll   OS: 12.3%  Delay: 18.2ms

CONFIG REVIEW:                                <- from param_analyzer on diff
  [CRITICAL] Beepers disabled
```

## Verdicts

| Verdict | Score | Meaning |
|---------|-------|---------|
| DIALED_IN | 85+ | Fly it, enjoy it |
| NEARLY_THERE | 75-84 | Minor tweaks left |
| GETTING_BETTER | 60-74 | On the right track |
| NEEDS_WORK | 40-59 | Several issues to fix |
| ROUGH | <40 | Significant problems |
| NEED_DATA | any | Fly with stick inputs |
| GROUND_ONLY | - | Armed but didn't fly |

## Tips

- **No PID data?** The analyzer needs stick inputs. Hover for 10s, then do
  deliberate rolls, pitches, and yaw sweeps. Smooth but decisive.
- **All flights "old config"?** You changed settings since the last flight.
  Erase flash or just fly again - the tool detects which flights match.
- **Filter changes recommended?** Apply filters first, fly one pack, then
  re-analyze. PID changes are deferred until filters are clean.
- **Beeper warning?** Fix it. BAT_CRIT_LOW and RX_LOST silent on a 10"
  is how you lose a quad.
- **Multiple flights on flash?** The tool splits them, groups by config
  session, and picks the best one from your latest config automatically.
- **Want Portuguese or Spanish?** Use `--lang pt_BR` or `--lang es`.

## Install

```bash
pip install inav-toolkit            # from PyPI
# or
git clone https://github.com/agoliveira/INAV-Toolkit.git
cd INAV-Toolkit && pip install -e .
```

Requires: Python 3.8+, numpy, scipy, pyserial (for --device)
