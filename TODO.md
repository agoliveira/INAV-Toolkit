# INAV Toolkit - TODO / Backlog

## Next Up

### Config Backup Manager
- **Priority:** Medium
- **Effort:** Small
- **Context:** `inav-toolkit backups` command that lists all backups by date/craft, lets you diff any two, or restore a specific one. The wizard already takes backups but they're loose timestamped files. Useful when tuning over weeks and wanting to revert to "the one that flew well."

### INAV Configurator Preset Export
- **Priority:** Medium
- **Effort:** Small
- **Context:** Generate a `.json` preset file that users can import directly into INAV Configurator instead of pasting CLI commands. Configurator supports preset imports, the format is documented.

### Version Bump Automation
- **Priority:** Medium
- **Effort:** Small
- **Context:** Version duplicated in 8 files (pyproject.toml + 7 .py files). A bump script or single-source version via `importlib.metadata` would prevent mismatches. Consider `bumpversion` or a simple `scripts/bump.sh`.

## Backlog

### Automated Tuning Loop (Raspberry Pi companion)
- **Priority:** Medium (deferred)
- **Effort:** Large
- **Context:** Run on a Raspberry Pi Zero 2W as an onboard companion. Analyze-compute-apply loop: download blackbox after landing, compute new PIDs, generate CLI commands, optionally apply before next flight. All pieces exist (analysis, MSP, CLI batch), needs orchestration and conservative gain scheduling. Discuss architecture separately.

### SD Card Blackbox Download via MSC
- **Priority:** Low
- **Effort:** Multi-day (platform-specific code + hardware testing)
- **Context:** Detection implemented in `inav_msp.py`. What remains is the actual MSC download automation: send `msc` CLI command, wait for USB mass storage mount, copy logs, unmount, reconnect. Linux first, then Windows/macOS.

### GUI / Desktop App
- **Priority:** Low (exploratory)
- **Effort:** Large
- **Context:** PyInstaller single-file executables as the most pragmatic first step — wraps CLI with zero code changes, produces downloadable binary per platform.

## Done

### ~~Comparative Flight Analysis~~ (v2.15.0)
Side-by-side `--compare` with overlay charts, score deltas, motor balance, craft mismatch warnings.

### ~~Blackbox Replay Visualization~~ (v2.15.0)
Interactive `--replay` with Plotly.js WebGL, spectrogram waterfall, flight mode overlay, synced axes.

### ~~First-Flight Sanity Check~~ (v2.14.1)
`inav-params --check` with 16 categories, interactive validation, GO/NO-GO verdict.

### ~~Log Quality Scorer~~ (v2.15.0)
`--check-log` with 8 quality checks, GOOD/MARGINAL/UNUSABLE grading, auto-gate in analysis.

### ~~Markdown Report Output~~ (v2.15.0)
`--report md` for forum/Discord-pasteable reports with CLI commands.

### ~~PyPI Publishing~~ (v2.15.0)
GitHub Actions trusted publishing on tag push. `pip install inav-toolkit`.
