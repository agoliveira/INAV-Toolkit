# INAV Toolkit - TODO / Backlog

## Pending

### SD Card Blackbox Download via MSC
- **Priority:** Medium
- **Effort:** Multi-day (platform-specific code + hardware testing)
- **Context:** INAV supports three blackbox backends: dataflash (implemented), SD card, OpenLog (not worth supporting). SD card logs can only be retrieved via USB MSC (Mass Storage Class) mode — no MSP file-read commands exist. The `msc` CLI command reboots the FC into a USB drive; the OS mounts it, files are copied, then FC reboots back to normal.
- **Current state:** Detection implemented in `inav_msp.py`. When `get_dataflash_summary()` returns unsupported, `download_blackbox()` queries `MSP_BLACKBOX_CONFIG` (cmd 80) to identify the storage backend and gives a clear message for SD card and serial users. What remains is the actual MSC download automation.
- **Implementation plan:**
  1. Start with Linux only (testable on real hardware)
  2. Send `msc` via CLI, wait for serial port to disappear
  3. Detect new block device / mount point (`/dev/sd*`, `/media/`, `udisksctl`)
  4. Find `.bbl` / `LOG*.TXT` files, copy the latest
  5. Unmount/eject, wait for FC to reboot back to normal mode
  6. Reconnect via serial (reuse restore-flow reconnect logic)
  7. Later: Windows (new drive letter detection), macOS (`/Volumes/`)
- **Risks:** MSC mode has known bugs on some FCs (F722-WPX, H743 regressions). Failed MSC cycle leaves FC unresponsive with no serial port. Need robust timeout and manual recovery instructions.

### PyPI Publishing
- **Priority:** Low
- **Effort:** Small
- **Context:** Package is pip-installable from source (`pip install -e .`) and wheel builds cleanly. Publishing to PyPI would allow `pip install inav-toolkit` without cloning the repo. Steps: Create PyPI account, configure trusted publishing via GitHub Actions, add publish workflow on tag push.

### Version Bump Automation
- **Priority:** Low
- **Effort:** Small
- **Context:** Version is defined in `__init__.py` and `pyproject.toml`, plus `VERSION`/`REPORT_VERSION` in each module. Bumping requires editing 6+ files. A bump script or single-source version (e.g. `importlib.metadata`) would reduce errors -- the v2.14.0 to v2.14.1 bump missed several files and was caught by CI.

### GUI / Desktop App
- **Priority:** Low (exploratory)
- **Effort:** Large
- **Context:** CLI-only interface works for technical users but limits adoption. Options: PyInstaller single-file executables (simplest), Electron/Tauri wrapper around HTML reports, or native GUI. PyInstaller is the most pragmatic first step -- wraps the CLI with zero code changes and produces a single downloadable binary per platform.
