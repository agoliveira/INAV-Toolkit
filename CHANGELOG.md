# Changelog

All notable changes to the INAV Flight Analyzer Toolkit.

## [2025-02-20] — Project Restructure

### Added
- Project documentation: README, detailed docs per tool, tuning workflow guide
- VTOL Configurator v1.0.1 — new tool for validating VTOL mixer profiles
- Parameter Analyzer setup mode (`--setup`) for generating starting configs

### Blackbox Analyzer v2.9.0
- **Improved:** Filter phase lag calculation with proper transfer functions
  - PT1, PT2, PT3: correct cascaded first-order phase
  - BIQUAD: proper H(s) = ωc² / (s² + (ωc/Q)s + ωc²) with Q factor support
  - Previous version approximated BIQUAD as cascaded PT1 which was inaccurate at high Q
- **Fixed:** RPM filter recommendation — correctly states INAV requires ESC telemetry wire, not bidirectional DSHOT (which is Betaflight-only)

### Parameter Analyzer v1.1.0
- **Added:** `--setup` mode for generating starting PIDs by frame size (7/10/12/15") and voltage (4S/6S/8S/12S)
- **Added:** Li-ion battery detection — uses capacity heuristic (≥7000mAh + low min voltage) to adjust severity
- **Added:** ESC telemetry serial port check for RPM filter validation
- **Fixed:** Removed all bidirectional DSHOT references (not supported in INAV)
- **Fixed:** EZ Tune detection — checks `ez_enabled` flag, not presence of `ez_*` parameters
- **Fixed:** Compass calibration quality check — mag gain spread analysis

### VTOL Configurator v1.0.1
- **Fixed:** Mode IDs — MIXER PROFILE 2 is mode 62, MIXER TRANSITION is mode 63 (was using incorrect Betaflight-derived values)
- **Added:** Control profile linking validation (`mixer_control_profile_linking`)
- **Added:** INAV mode name lookup table for readable output

## [2025-02-17] — Blackbox Analyzer v2.8.0

### Added
- Native binary blackbox decoder (no external tools needed)
- Cross-correlation delay measurement between setpoint and gyro
- Noise spectral analysis with peak identification
- Motor balance analysis
- PID step response (overshoot, settling time, delay)
- Filter chain phase lag estimation
- HTML report with embedded plots
- JSON state output for automation
- Frame-size-aware recommendations

## [2025-02-17] — Parameter Analyzer v1.0.0

### Added
- `diff all` parser with profile support
- Safety checks (beepers, failsafe, battery)
- Motor protocol and RPM filter validation
- Filter checks with frame-size awareness
- PID cross-profile consistency
- Navigation parameter validation
- GPS and compass checks
- Blackbox configuration audit
- CLI fix generation
- JSON output mode
- Cross-reference with blackbox state
