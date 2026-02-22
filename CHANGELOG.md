# Changelog

All notable changes to INAV Toolkit.

## [2026-02-22] - v2.14.0

### Added
- **pip-installable package**: `pip install inav-toolkit` now installs the full suite with all dependencies and creates system-wide commands: `inav-analyze`, `inav-params`, `inav-toolkit`, `inav-msp`. Works with pip, venv, and pipx.
- **Guided wizard** (`inav-toolkit`): Interactive session manager that orchestrates the full tuning workflow. Connects to FC, creates timestamped config backup, downloads blackbox, runs analysis, reviews changes with safety gates, and applies CLI commands via MSP batch mode.
- **MSP batch CLI mode**: `cli_batch()` sends multiple CLI commands to the FC in a single session with per-command echo verification.
- **SD card blackbox detection**: `MSP_BLACKBOX_CONFIG` (cmd 80) queries the FC's blackbox storage device. When SD card is detected, provides guidance (remove card or use `msc` CLI command) instead of misleading "Dataflash not supported" errors. Serial logger detection included.
- **Backup/restore safety system**: Wizard creates a full `diff all` backup before any changes. Restore gate prevents applying changes if backup wasn't taken. Handles FC reboot and serial reconnection after `save` command.

### Changed
- **Package layout**: All modules moved into `inav_toolkit/` package directory. Old `inav_*.py` root scripts replaced by proper Python package with entry points.
- **Unified version**: All modules now report v2.14.0 from a single source (`inav_toolkit.__version__`).
- **pyserial now a default dependency**: Was optional; now included by default since the wizard and MSP tools need it. Adds only ~200KB.
- **requirements.txt**: Added missing `matplotlib>=3.5.0` dependency (was imported but not listed).

### Fixed
- **Parameter analyzer crash on `--frame` omission**: `check_filters()` crashed with `TypeError` when `frame_inches` was `None` (no `--frame` flag and no auto-detection). Now guards frame-specific threshold checks.
- **Windows ANSI color support**: All modules now enable Windows console virtual terminal processing. Respects `NO_COLOR` environment variable.

### Docs
- README rewritten with `pip install` instructions, new command names, updated project structure.
- All docs (`docs/*.md`) updated from `python3 inav_*.py` to new command names.
- Smoke tests updated for package module invocation (`python -m inav_toolkit.*`).

## [2026-02-22] - v2.13.0

### Added
- **Navigation health analysis** (`--nav` flag): Completely separate analysis mode that bypasses PID tuning and produces navigation-focused diagnostics. Six analysis modules:
  - **Compass health**: Heading jitter, magnetic interference correlation with throttle, heading drift rate during GPS-controlled flight.
  - **GPS quality**: Fix type distribution, satellite count statistics, HDOP analysis, position jump detection, 3D speed consistency.
  - **Barometer performance**: Altitude noise (RMS), propwash interference correlation, short-term stability assessment.
  - **Position estimator**: Estimated vs GPS position divergence, velocity estimation accuracy, estimator reset detection.
  - **Altitude hold**: Altitude tracking error during ALTHOLD/POSHOLD, climb/descent response analysis.
  - **Position hold drift**: Circular error probable (CEP), toilet-bowl pattern detection, drift rate measurement during POSHOLD.
- **Extended blackbox decoder**: Extracts 30+ navigation fields from blackbox logs. Decodes S-frames (flight mode changes) and G-frames (GPS data) that were previously skipped.
- **Flight phase segmentation**: Analysis isolates nav-controlled flight periods (POSHOLD, ALTHOLD, RTH, WP) to avoid false positives from manual flight maneuvers.
- **Navigation HTML report**: Standalone dark-themed HTML report with compass, baro, GPS, and position hold charts. Separate from the PID tuning report.
- **Tuning prerequisite warnings**: Navigation analysis detects PID oscillation issues and warns users to fix tuning problems before trusting navigation health metrics.
- **Frame size profiles expanded**: Added 5-inch profile with research-validated defaults. All profiles (5/7/10/12/15") now include size-specific analysis thresholds for noise frequencies, filter recommendations, and PID starting points.
- **Color-coded navigation scores**: Terminal output shows per-module scores (compass, GPS, baro, position hold) with color-coded pass/warn/fail indicators.

## [2026-02-22] - Blackbox Analyzer v2.12.1

### Fixed
- **MSP download speed regression**: Fixed 100x speed drop (2 KB/s vs 200 KB/s) when downloading blackbox through the analyzer. Root cause: `_recv()` hot-spun at 100% CPU when the serial buffer held a partial MSP frame with no new data arriving. In the lightweight standalone `inav_msp.py` process this was harmless, but in the analyzer process (89,000+ GC-tracked objects from numpy/scipy/matplotlib) the spin competed with Python's garbage collector and OS USB interrupt servicing, causing FC USB flow control back-pressure. Fix: 200us sleep when waiting for partial frame completion.
- **Em dash cleanup**: Replaced all Unicode em dashes with ASCII hyphens across all source files and documentation.

### Changed
- **Automatic diff pull on --device**: `diff all` is now pulled automatically when the FC is connected (previously required explicit `--diff` flag). Enables session detection, CONFIG REVIEW, and mismatch detection without extra flags.

## [2026-02-21] - Blackbox Analyzer v2.12.0

### Added
- **Flight history database** (`inav_flight_db.py`): SQLite database stores analysis results from every flight - scores, per-axis oscillation data, PID values, filter config, motor balance, and actions. Enables progression tracking across tuning sessions. Zero external dependencies (Python stdlib sqlite3).
- **Multi-log splitter**: Dataflash dumps containing multiple arm/disarm cycles are automatically detected and split into individual flights. Each is analyzed separately with per-flight progression tracking.
- **Flight progression tracking**: After each analysis, the database compares with the previous flight for the same craft and shows score deltas, oscillation changes, and PID value changes. Trend detection (improving/stable/degrading).
- **MSP CLI diff merge**: New `--diff` flag pulls the full `diff all` configuration from the FC before downloading blackbox data. Settings missing from blackbox headers (motor_poles, nav PIDs, rates, level mode, antigravity) are enriched from the diff. Mismatches between blackbox and current FC config are detected and displayed.
- **`--history` flag**: Shows a tabular flight history for the craft with scores, verdicts, and progression summary.
- **`--db-path` / `--no-db` flags**: Custom database path or skip storage entirely.

## [2026-02-21] - Blackbox Analyzer v2.11.0

### Fixed
- **Removed `profile 1` CLI command**: INAV 9 CLI does not support profile switching. The v2.10.0 fix incorrectly emitted `profile 1` before profile-scoped parameters, causing CLI errors.
- **Scoring bug: inflated scores on hover-only logs**: When no stick inputs were detected (0 steps), PID score silently defaulted to 50/100 instead of being flagged as unmeasurable. A pure hover log could score 83/100 with "No changes needed" even on a wobbling quad. Now: PID shows as N/A, overall score is capped at 65, and the verdict tells the pilot to fly with stick inputs.
- **Duplicate CLI commands**: When both oscillation and PID actions targeted the same axis, conflicting `set` commands were emitted. CLI generation now deduplicates by parameter name.
- **MSP dataflash detection**: INAV uses the flags byte as a simple ready boolean (0x01), not a bitfield like Betaflight. The `supported` bit (0x02) is never set by INAV, causing "Dataflash not available" errors on working hardware. Now inferred from `total_size > 0`.
- **MSP dataflash read format**: INAV's MSP_DATAFLASH_READ request takes `address(u32) + size(u16)` without a compression flag byte, and the response is `address(u32) + data` without dataSize/compressedSize headers. The Betaflight-style parsing was corrupting reads after the first chunk.
- **MSP serial port reuse**: `auto_detect_fc()` left the serial port open, then the analyzer opened a second connection to the same port, causing communication failures. Now returns the open device object for reuse.
- **MSP frame decoder robustness**: `$X` bytes appearing inside binary blackbox payload data could cause false frame matches. The decoder now skips false matches using CRC validation.

### Added
- **Hover oscillation detection**: Detects oscillation during hover (no stick input) by analyzing gyro variance in centered-stick segments. Classifies severity (mild/moderate/severe), identifies the dominant frequency, and diagnoses the cause: low-frequency (~2-10Hz) = P too high, mid-frequency (~10-25Hz) = P/D interaction, high-frequency (~25-50Hz) = D-term noise, very high (~50Hz+) = filter gap. Generates CRITICAL/IMPORTANT actions with aggressive P/D reductions. Oscillation actions take priority over regular PID recommendations using oscillation-first enforcement (similar to filter-first).
- **Direct FC communication via MSP**: New `--device` flag downloads blackbox data directly from the flight controller over USB serial. Auto-detects INAV FCs, saves logs with sensible names (craft_timestamp.bbl), and optionally erases dataflash after download. Feeds directly into the analysis pipeline - no more switching to INAV Configurator to download logs.
- **New module `inav_msp.py`**: Standalone MSP v2 protocol implementation for INAV. Handles FC identification, dataflash summary/read/erase. Can also be used independently for scripting.
- **Gyro oscillation detection**: Even without stick inputs, the analyzer now measures gyro variance to detect wobble/oscillation during hover. This feeds into the quality score as a PID proxy when step response data is unavailable.
- **Auto-detect frame size from craft name**: Parses the craft name header (e.g., "NAZGUL 10") to automatically determine frame size. No more silent 5" defaults when the log clearly says otherwise.
- **Frame size conflict warning**: When `--frame` is specified but contradicts the craft name (e.g., `--frame 5` on a "NAZGUL 10" log), the analyzer prints a clear ⚠ warning explaining the mismatch and which value is being used.
- **Auto-detect battery cells from vbatref**: If `--cells` is not specified, battery cell count is inferred from the blackbox `vbatref` header voltage.
- **Platform detection from field names**: Motor count and servo count are read from the `Field I name` header to determine platform type (Quadcopter, Hexacopter, Tricopter, etc.).
- **Comprehensive pre-analysis banner**: Before decoding begins, the analyzer now displays a structured identification block showing: aircraft name, firmware version and build date, platform type (quad/hex/tri + motor/servo count), frame size with source (user/auto-detected/conflict), prop configuration, battery cell count, motor KV, and the analysis profile with thresholds.

### Changed
- Headers are now parsed before building the frame profile, enabling auto-detection to inform profile selection.
- RPM prediction now uses auto-detected cell count when `--cells` is not explicitly provided.

## [2025-02-21] - Blackbox Analyzer v2.10.0

### Fixed
- **Filter-first enforcement**: When filter changes are needed, PID recommendations are now deferred until after filters are fixed and a re-fly. Previously, the analyzer would recommend both filter and PID changes simultaneously, leading users to raise D-term gains into wide-open filters, amplifying noise and creating a downward tuning spiral.
- **CLI parameter naming**: `gyro_lpf_hz` corrected to `gyro_main_lpf_hz` (renamed in INAV 3.0+).
- **Profile-scoped CLI commands**: Parameters like `dterm_lpf_hz`, `mc_p_roll`, etc. now correctly emit a `profile 1` line before profile-scoped settings. Previously, pasting CLI output at the master level would fail with "invalid setting".

### Changed
- CLI output now separates global parameters (gyro filters, dynamic notch) from profile-scoped parameters (PIDs, dterm filters) with proper context switching.
- State JSON now includes a `deferred_actions` array alongside `actions`, making it clear which recommendations require a re-fly first.
- HTML report shows deferred PID changes in a distinct visual section rather than mixing them with actionable filter fixes.

## [2025-02-20] - Project Restructure

### Added
- Project documentation: README, detailed docs per tool, tuning workflow guide
- VTOL Configurator v1.0.1 - new tool for validating VTOL mixer profiles
- Parameter Analyzer setup mode (`--setup`) for generating starting configs

### Blackbox Analyzer v2.9.0
- **Improved:** Filter phase lag calculation with proper transfer functions
  - PT1, PT2, PT3: correct cascaded first-order phase
  - BIQUAD: proper H(s) = ωc² / (s² + (ωc/Q)s + ωc²) with Q factor support
  - Previous version approximated BIQUAD as cascaded PT1 which was inaccurate at high Q
- **Fixed:** RPM filter recommendation - correctly states INAV requires ESC telemetry wire, not bidirectional DSHOT (which is Betaflight-only)

### Parameter Analyzer v1.1.0
- **Added:** `--setup` mode for generating starting PIDs by frame size (7/10/12/15") and voltage (4S/6S/8S/12S)
- **Added:** Li-ion battery detection - uses capacity heuristic (≥7000mAh + low min voltage) to adjust severity
- **Added:** ESC telemetry serial port check for RPM filter validation
- **Fixed:** Removed all bidirectional DSHOT references (not supported in INAV)
- **Fixed:** EZ Tune detection - checks `ez_enabled` flag, not presence of `ez_*` parameters
- **Fixed:** Compass calibration quality check - mag gain spread analysis

### VTOL Configurator v1.0.1
- **Fixed:** Mode IDs - MIXER PROFILE 2 is mode 62, MIXER TRANSITION is mode 63 (was using incorrect Betaflight-derived values)
- **Added:** Control profile linking validation (`mixer_control_profile_linking`)
- **Added:** INAV mode name lookup table for readable output

## [2025-02-17] - Blackbox Analyzer v2.8.0

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

## [2025-02-17] - Parameter Analyzer v1.0.0

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
