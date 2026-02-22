# INAV Toolkit - TODO / Backlog

## Pending

### SD Card Blackbox Download via MSC
- **Priority:** Medium
- **Effort:** Multi-day (platform-specific code + hardware testing)
- **Context:** INAV supports three blackbox backends: dataflash (implemented), SD card, OpenLog (not worth supporting). SD card logs can only be retrieved via USB MSC (Mass Storage Class) mode — no MSP file-read commands exist. The `msc` CLI command reboots the FC into a USB drive; the OS mounts it, files are copied, then FC reboots back to normal.
- **Current state:** Detection implemented in `inav_msp.py`. When `get_dataflash_summary()` returns unsupported, `download_blackbox()` queries `MSP_BLACKBOX_CONFIG` (cmd 80) to identify the storage backend and gives a clear message for SD card and serial users. Standalone CLI (`python3 inav_msp.py`) also detects and shows appropriate guidance. Offline analysis flow already handles user-provided files. What remains is the actual MSC download automation.
- **Implementation plan:**
  1. Start with Linux only (testable on real hardware)
  2. Send `msc` via CLI, wait for serial port to disappear
  3. Detect new block device / mount point (`/dev/sd*`, `/media/`, `udisksctl`)
  4. Find `.bbl` / `LOG*.TXT` files, copy the latest
  5. Unmount/eject, wait for FC to reboot back to normal mode
  6. Reconnect via serial (reuse restore-flow reconnect logic)
  7. Later: Windows (new drive letter detection), macOS (`/Volumes/`)
- **Risks:** MSC mode has known bugs on some FCs (F722-WPX, H743 regressions). Failed MSC cycle leaves FC unresponsive with no serial port. Need robust timeout and manual recovery instructions.
- **References:**
  - INAV docs: `docs/USB_Mass_Storage_(MSC)_mode.md`
  - GitHub issues: #9969 (SpeedyBee F405 Wing MSC broken), #9983 (MSC corrupting logs), #6288 (F722-WPX freeze)
  - INAV 9.0.1 release notes: "Fix H743 USB MSC regression, add timeout protection and retry logic"
- **OpenLog:** Not implementing — rarely used, FC has no read access to OpenLog's SD card (one-way serial stream).
