#!/usr/bin/env python3
"""
INAV Toolkit - Guided Session Manager

Interactive wizard that orchestrates the INAV analysis tools into a
guided tuning workflow. Handles FC connection, blackbox download,
analysis, result presentation, and CLI command application.

Usage:
    inav-toolkit                          # Interactive guided mode
    inav-toolkit --device /dev/ttyACM0    # Specify port
"""

import glob
import json
import os
import re
import subprocess
import sys
import time

try:
    from inav_toolkit import __version__ as VERSION
except ImportError:
    VERSION = "2.15.0"

# Module paths for subprocess invocation (package-aware)
ANALYZER_MODULE = "inav_toolkit.blackbox_analyzer"
PARAM_MODULE = "inav_toolkit.param_analyzer"
# Legacy script names (for git-clone-without-install usage)
ANALYZER_SCRIPT = "inav_blackbox_analyzer.py"
PARAM_SCRIPT = "inav_param_analyzer.py"


# ─── Color Support ────────────────────────────────────────────────────────────

def _enable_ansi_colors():
    """Enable ANSI color support."""
    if os.environ.get("NO_COLOR") is not None:
        return False
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetStdHandle(-11)
            mode = ctypes.c_ulong()
            kernel32.GetConsoleMode(handle, ctypes.byref(mode))
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
            return True
        except Exception:
            return False
    return True

_ANSI = _enable_ansi_colors()

def _c(code):
    """Return ANSI code if colors enabled, empty string otherwise."""
    return code if _ANSI else ""

R  = _c("\033[0m")
B  = _c("\033[1m")
C  = _c("\033[96m")
G  = _c("\033[92m")
Y  = _c("\033[93m")
RED = _c("\033[91m")
DIM = _c("\033[2m")


# ─── Utilities ────────────────────────────────────────────────────────────────

def _script_dir():
    """Directory where toolkit scripts live."""
    return os.path.dirname(os.path.abspath(__file__))


def _module_cmd(module_path, fallback_script=None):
    """Build command prefix to run a toolkit module.

    Works both as installed package (python -m inav_toolkit.X)
    and from legacy script layout (python inav_blackbox_analyzer.py).
    """
    # Try package module first (pip install or running from repo root)
    cmd = [sys.executable, "-m", module_path]
    try:
        # Quick check: can Python find this module?
        result = subprocess.run(
            cmd + ["--help"], capture_output=True, timeout=5)
        if result.returncode in (0, 2):  # 0=ok, 2=argparse error (no args)
            return cmd
    except Exception:
        pass

    # Fallback: look for legacy script alongside this file's parent dir
    if fallback_script:
        parent = os.path.dirname(_script_dir())
        script = os.path.join(parent, fallback_script)
        if os.path.isfile(script):
            return [sys.executable, script]
        # Also check same directory (flat layout)
        script = os.path.join(_script_dir(), fallback_script)
        if os.path.isfile(script):
            return [sys.executable, script]

    # PyInstaller bundle
    if fallback_script and getattr(sys, '_MEIPASS', None):
        script = os.path.join(sys._MEIPASS, fallback_script)
        if os.path.isfile(script):
            return [sys.executable, script]

    return None


def _clear_line():
    """Clear current terminal line."""
    print("\r" + " " * 70 + "\r", end="", flush=True)


def _prompt(question, options=None, default=None):
    """Interactive prompt with optional choices.

    Args:
        question: Text to display
        options: List of (key, label) tuples, or None for free text
        default: Default value if user presses Enter

    Returns:
        User's choice (lowercase key, or free text)
    """
    if options:
        print(f"\n  {B}{question}{R}")
        for key, label in options:
            marker = f" {DIM}(default){R}" if key == default else ""
            print(f"    {C}{key}{R}  {label}{marker}")
        while True:
            hint = f" [{default}]" if default else ""
            choice = input(f"\n  > {hint} ").strip().lower()
            if not choice and default:
                return default
            valid = [k.lower() for k, _ in options]
            if choice in valid:
                return choice
            print(f"    {Y}Choose one of: {', '.join(valid)}{R}")
    else:
        hint = f" [{default}]" if default else ""
        print(f"\n  {B}{question}{R}")
        choice = input(f"  > {hint} ").strip()
        if not choice and default:
            return default
        return choice


def _confirm(question, default=True):
    """Yes/no prompt."""
    hint = "Y/n" if default else "y/N"
    answer = input(f"  {question} [{hint}] ").strip().lower()
    if not answer:
        return default
    return answer in ("y", "yes")


def _banner():
    """Print welcome banner."""
    print(f"""
  {B}{C}{'=' * 54}{R}
  {B}  INAV Toolkit v{VERSION} - Guided Session Manager  {R}
  {B}{C}{'=' * 54}{R}
    """)


# ─── FC Connection ────────────────────────────────────────────────────────────

def _connect_fc(port=None):
    """Connect to FC and return (device, info) or (None, None)."""
    try:
        from inav_toolkit.msp import INAVDevice, auto_detect_fc, find_serial_ports
    except ImportError:
        try:
            from inav_msp import INAVDevice, auto_detect_fc, find_serial_ports
        except ImportError:
            print(f"  {RED}ERROR: MSP module not found{R}")
            print(f"  Install: pip install inav-toolkit[serial]")
            return None, None

    try:
        import serial
    except ImportError:
        print(f"  {RED}ERROR: pyserial is required for FC communication.{R}")
        print(f"    Debian/Ubuntu: sudo apt install python3-serial")
        print(f"    Other:         pip install pyserial  (in a venv)")
        return None, None

    if port and port != "auto":
        print(f"  Connecting to {port}...", end="", flush=True)
        try:
            fc = INAVDevice(port)
            fc.open()
            info = fc.get_info()
            if info and info.get("fc_variant") == "INAV":
                print(f" {G}connected{R}")
                return fc, info
            print(f" {RED}not an INAV FC{R}")
            fc.close()
        except Exception as e:
            print(f" {RED}failed: {e}{R}")
        return None, None

    print(f"  Scanning for INAV flight controller...", end="", flush=True)
    fc, info = auto_detect_fc()
    if fc and info:
        print(f" {G}found{R}")
        return fc, info

    print(f" {Y}not found{R}")
    ports = find_serial_ports()
    if ports:
        print(f"    Ports detected but none responded as INAV: {', '.join(ports)}")
        print(f"    Make sure the FC is powered and not in DFU mode.")
    else:
        print(f"    No serial ports detected. Is the FC connected via USB?")
    return None, None


def _print_fc_info(info):
    """Display FC identification."""
    craft = info.get("craft_name") or "(unnamed)"
    fw = info.get("firmware", "")
    board = info.get("board_id", "")
    print(f"\n  {B}Aircraft:{R}  {craft}")
    print(f"  {B}Firmware:{R}  {fw}")
    if board:
        print(f"  {B}Board:{R}     {board}")


# ─── Download & Analyze ──────────────────────────────────────────────────────

def _download_blackbox(fc, info, output_dir="./blackbox"):
    """Download blackbox from FC. Returns filepath or None."""
    summary = fc.get_dataflash_summary()
    if not summary or not summary.get("supported"):
        print(f"  {RED}Dataflash not available on this FC.{R}")
        return None

    used_kb = summary["used_size"] / 1024
    total_kb = summary["total_size"] / 1024
    pct = summary["used_size"] * 100 // summary["total_size"] if summary["total_size"] > 0 else 0

    if summary["used_size"] == 0:
        print(f"\n  Dataflash: {used_kb:.0f}KB / {total_kb:.0f}KB ({pct}% used)")
        print(f"  {Y}No blackbox data to download.{R}")
        print(f"  Fly with blackbox enabled, then come back.")
        return None

    print(f"\n  Dataflash: {used_kb:.0f}KB / {total_kb:.0f}KB ({pct}% used)")

    craft = info.get("craft_name") or "fc"
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"{craft}_{timestamp}.bbl"
    filepath = fc.download_blackbox(output_dir=output_dir, filename=filename)

    if filepath and os.path.isfile(filepath):
        return filepath
    return None


def _pull_diff(fc, output_dir="./blackbox", craft_name="fc"):
    """Pull diff all from FC. Returns raw text or None."""
    print(f"  Pulling configuration...", end="", flush=True)
    diff_raw = fc.get_diff_all(timeout=10.0)
    if diff_raw:
        n = len([l for l in diff_raw.splitlines() if l.strip().startswith("set ")])
        print(f" {n} settings")
        # Save alongside blackbox
        diff_path = os.path.join(output_dir, f"{craft_name}_diff.txt")
        os.makedirs(output_dir, exist_ok=True)
        with open(diff_path, "w") as f:
            f.write(diff_raw)
        return diff_raw
    print(f" {Y}no response{R}")
    return None


def _run_analyzer(logfile, mode="tune", diff_file=None, frame=None, extra_args=None):
    """Run the blackbox analyzer and return results.

    Args:
        logfile: Path to BBL file
        mode: 'tune' or 'nav'
        diff_file: Path to diff file (optional)
        frame: Frame size in inches (optional)
        extra_args: Additional CLI arguments

    Returns:
        dict with keys: success, score, verdict, actions, state_json_path,
                        html_path, output (terminal text)
    """
    cmd_prefix = _module_cmd(ANALYZER_MODULE, ANALYZER_SCRIPT)
    if not cmd_prefix:
        return {"success": False, "output": f"Cannot find analyzer module"}

    cmd = cmd_prefix + [logfile]
    if mode == "nav":
        cmd.append("--nav")
    if diff_file:
        cmd.extend(["--diff", diff_file])
    if frame:
        cmd.extend(["--frame", str(frame)])
    if extra_args:
        cmd.extend(extra_args)

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return {"success": False, "output": "Analysis timed out (120s)"}
    except Exception as e:
        return {"success": False, "output": str(e)}

    # Find state.json
    state_json = logfile.rsplit(".", 1)[0] + "_state.json"
    state = None
    if os.path.isfile(state_json):
        try:
            with open(state_json, "r") as f:
                state = json.load(f)
        except Exception:
            pass

    # Find HTML report
    html_path = None
    base = logfile.rsplit(".", 1)[0]
    for suffix in ("_report.html", "_nav_report.html"):
        candidate = base + suffix
        if os.path.isfile(candidate):
            html_path = candidate

    score = None
    verdict = None
    actions = []
    if state:
        scores = state.get("scores", {})
        score = scores.get("overall")
        verdict = state.get("verdict", "")
        actions = state.get("actions", [])

    return {
        "success": result.returncode == 0,
        "score": score,
        "verdict": verdict,
        "actions": actions,
        "deferred": state.get("deferred_actions", []) if state else [],
        "state": state,
        "state_json_path": state_json if state else None,
        "html_path": html_path,
        "output": output,
    }


def _run_param_check(diff_file, frame=None):
    """Run parameter analyzer safety check. Returns (success, output)."""
    cmd_prefix = _module_cmd(PARAM_MODULE, PARAM_SCRIPT)
    if not cmd_prefix:
        return False, f"Cannot find parameter analyzer module"

    cmd = cmd_prefix + [diff_file]
    if frame:
        cmd.extend(["--frame", str(frame)])

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        return result.returncode == 0, result.stdout + result.stderr
    except Exception as e:
        return False, str(e)


# ─── Backup & Restore ─────────────────────────────────────────────────────────

def _create_backup(fc, info, output_dir="./blackbox"):
    """Pull diff from FC and save as timestamped backup.

    This is a SAFETY GATE - if the backup cannot be written, the session
    must not proceed with any changes to the FC.

    Args:
        fc: Connected INAVDevice
        info: FC info dict
        output_dir: Directory to save backup

    Returns:
        (backup_path, diff_raw) on success, (None, None) on failure
    """
    craft = info.get("craft_name") or "fc"
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    backup_name = f"{craft}_backup_{timestamp}.txt"

    print(f"\n  {B}Creating configuration backup...{R}")

    # Pull current config
    diff_raw = fc.get_diff_all(timeout=10.0)
    if not diff_raw or not diff_raw.strip():
        print(f"  {RED}FAILED: Could not read configuration from FC.{R}")
        print(f"  {RED}No changes will be made without a backup.{R}")
        return None, None

    # Count settings to sanity-check the diff
    set_lines = [l for l in diff_raw.splitlines() if l.strip().startswith("set ")]
    if len(set_lines) < 5:
        print(f"  {RED}FAILED: Config looks incomplete ({len(set_lines)} settings).{R}")
        print(f"  {RED}No changes will be made without a valid backup.{R}")
        return None, None

    # Write backup file
    try:
        os.makedirs(output_dir, exist_ok=True)
        backup_path = os.path.join(output_dir, backup_name)
        with open(backup_path, "w") as f:
            f.write(f"# INAV Toolkit backup - {craft}\n")
            f.write(f"# Created: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# Firmware: {info.get('firmware', 'unknown')}\n")
            f.write(f"# Board: {info.get('board_id', 'unknown')}\n")
            f.write(f"# To restore: paste this entire file in the INAV CLI tab\n")
            f.write(f"# after running 'defaults' (or use the toolkit's restore).\n")
            f.write(f"#\n")
            f.write(diff_raw)
        # Verify write
        verify_size = os.path.getsize(backup_path)
        if verify_size < len(diff_raw):
            raise IOError("Written file smaller than expected")
    except Exception as e:
        print(f"  {RED}FAILED: Could not write backup file: {e}{R}")
        print(f"  {RED}No changes will be made without a backup.{R}")
        return None, None

    print(f"  {G}Backup saved: {backup_path}{R}")
    print(f"  {DIM}({len(set_lines)} settings, {verify_size:,} bytes){R}")
    print(f"  {Y}Keep this file safe. If anything goes wrong, paste it in{R}")
    print(f"  {Y}the INAV Configurator CLI tab after running 'defaults'.{R}")

    return backup_path, diff_raw


def _restore_backup(fc, info, backup_path, port=None):
    """Restore FC to backup state: defaults + replay diff.

    This is a destructive operation:
    1. Sends 'defaults' (FC reboots)
    2. Reconnects to FC
    3. Replays the backup diff line by line
    4. Saves

    Args:
        fc: Connected INAVDevice (will be closed and reopened)
        info: FC info dict
        backup_path: Path to backup diff file
        port: Serial port path for reconnection

    Returns:
        (new_fc, success) - new device object and success flag
    """
    # Read backup
    try:
        with open(backup_path, "r") as f:
            backup_text = f.read()
    except Exception as e:
        print(f"  {RED}Cannot read backup file: {e}{R}")
        return fc, False

    # Extract CLI commands (skip comments, empty lines)
    cli_lines = []
    for line in backup_text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        cli_lines.append(line)

    if not cli_lines:
        print(f"  {RED}Backup file is empty or contains only comments.{R}")
        return fc, False

    set_count = len([l for l in cli_lines if l.startswith("set ")])
    print(f"\n  {B}Restoring from backup: {os.path.basename(backup_path)}{R}")
    print(f"  {DIM}({set_count} settings to restore){R}")
    print(f"\n  {Y}WARNING: This will reset ALL settings to defaults,{R}")
    print(f"  {Y}then restore from backup. The FC will reboot.{R}")

    if not _confirm(f"\n  Proceed with full restore?", default=False):
        print(f"  Restore cancelled.")
        return fc, False

    # Remember port for reconnection
    port_path = port or fc.port_path

    # Step 1: Send defaults (triggers reboot)
    print(f"\n  Sending 'defaults'...", end="", flush=True)
    try:
        ser = fc._ser
        # Enter CLI
        ser.write(b"#")
        time.sleep(0.3)
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        # Send defaults
        ser.write(b"defaults\n")
        time.sleep(0.5)
        print(f" FC is rebooting...")
    except Exception as e:
        print(f" {RED}failed: {e}{R}")
        print(f"  {Y}You may need to manually restore by pasting the backup{R}")
        print(f"  {Y}in the CLI tab after running 'defaults'.{R}")
        return fc, False

    # Close old connection
    try:
        fc.close()
    except Exception:
        pass

    # Step 2: Wait for reboot and reconnect
    print(f"  Waiting for FC to reboot...", end="", flush=True)
    time.sleep(5)

    try:
        from inav_toolkit.msp import INAVDevice, auto_detect_fc
    except ImportError:
        try:
            from inav_msp import INAVDevice, auto_detect_fc
        except ImportError:
            print(f" {RED}cannot import MSP module{R}")
            return None, False

    # Try reconnecting several times
    new_fc = None
    for attempt in range(6):
        try:
            new_fc = INAVDevice(port_path)
            new_fc.open()
            new_info = new_fc.get_info()
            if new_info and new_info.get("fc_variant") == "INAV":
                print(f" {G}reconnected{R}")
                break
            new_fc.close()
            new_fc = None
        except Exception:
            new_fc = None
        time.sleep(2)
        print(".", end="", flush=True)

    if not new_fc:
        print(f" {RED}could not reconnect{R}")
        print(f"\n  {RED}FC was reset to defaults but config was NOT restored.{R}")
        print(f"  {Y}To restore manually:{R}")
        print(f"    1. Open INAV Configurator")
        print(f"    2. Go to CLI tab")
        print(f"    3. Paste contents of: {backup_path}")
        print(f"    4. Type 'save'")
        return None, False

    # Step 3: Replay backup config
    print(f"  Restoring {set_count} settings...", end="", flush=True)
    try:
        results = new_fc.cli_batch(cli_lines, timeout=5.0, save=True)
        # Check for errors
        errors = []
        for cmd, response in results:
            if cmd == "save":
                continue
            if "invalid" in response.lower():
                errors.append(cmd)

        if errors:
            print(f" {Y}done with {len(errors)} warnings{R}")
            print(f"  {DIM}Unrecognized settings (may be version-specific):{R}")
            for cmd in errors[:5]:
                print(f"    {DIM}{cmd}{R}")
            if len(errors) > 5:
                print(f"    {DIM}...and {len(errors) - 5} more{R}")
        else:
            print(f" {G}done{R}")

        print(f"\n  {G}Configuration restored successfully.{R}")
        return new_fc, True

    except Exception as e:
        print(f" {RED}failed: {e}{R}")
        print(f"\n  {RED}Defaults were applied but restore did not complete.{R}")
        print(f"  {Y}Restore manually from: {backup_path}{R}")
        return new_fc, False

def _extract_cli_commands(actions):
    """Extract CLI set commands from action list.

    Returns list of 'set param = value' strings.
    """
    commands = []
    for a in actions:
        action_text = a.get("action", "")
        # Match patterns like "set mc_p_roll = 28" within the action text
        matches = re.findall(r"set\s+\S+\s*=\s*\S+", action_text)
        commands.extend(matches)
        # Also check param/new fields
        param = a.get("param", "")
        new_val = a.get("new", "")
        if param and new_val:
            cmd = f"set {param} = {new_val}"
            if cmd not in commands:
                commands.append(cmd)
    return commands


def _apply_commands(fc, commands):
    """Apply CLI commands to FC via batch mode."""
    if not commands:
        return True

    print(f"\n  Applying {len(commands)} changes...")
    for cmd in commands:
        print(f"    {C}{cmd}{R}")

    try:
        results = fc.cli_batch(commands, save=True)
        # Check for errors
        errors = []
        for cmd, response in results:
            if cmd == "save":
                continue
            if "invalid" in response.lower() or "error" in response.lower():
                errors.append((cmd, response))

        if errors:
            print(f"\n  {Y}Some commands had issues:{R}")
            for cmd, resp in errors:
                print(f"    {cmd}: {resp}")
            return False

        print(f"  {G}Settings applied and saved.{R}")
        return True

    except Exception as e:
        print(f"  {RED}Failed to apply: {e}{R}")
        return False


def _erase_dataflash(fc):
    """Erase dataflash after confirmation."""
    print(f"\n  Erasing dataflash...", end="", flush=True)
    try:
        fc.erase_dataflash()
        print(f" {G}done{R}")
        return True
    except Exception as e:
        print(f" {RED}failed: {e}{R}")
        return False


# ─── Result Presentation ─────────────────────────────────────────────────────

def _print_score(score, verdict, prev_score=None):
    """Display the tuning score prominently."""
    if score is None:
        print(f"\n  {DIM}Score: N/A{R}")
        return

    if score >= 80:
        color = G
    elif score >= 60:
        color = Y
    else:
        color = RED

    bar_width = 30
    filled = int(score / 100 * bar_width)
    bar = f"{color}{'=' * filled}{DIM}{'-' * (bar_width - filled)}{R}"

    delta = ""
    if prev_score is not None:
        diff = score - prev_score
        if diff > 0:
            delta = f"  {G}+{diff}{R}"
        elif diff < 0:
            delta = f"  {RED}{diff}{R}"
        else:
            delta = f"  {DIM}+0{R}"

    vd = (verdict or "").replace("_", " ").title()
    print(f"\n  Score: {color}{B}{score}/100{R}{delta}")
    print(f"  [{bar}]")
    if vd:
        print(f"  {DIM}{vd}{R}")


def _print_actions(actions, deferred=None):
    """Display recommended actions."""
    if not actions and not deferred:
        print(f"\n  {G}No changes needed - go fly!{R}")
        return

    if actions:
        print(f"\n  {B}Recommended changes:{R}")
        for i, a in enumerate(actions, 1):
            action = a.get("action", "")
            print(f"    {C}{i}.{R} {action}")

    if deferred:
        print(f"\n  {DIM}Deferred (fix filters first, then re-fly):{R}")
        for a in deferred:
            action = a.get("action", a.get("original_action", ""))
            print(f"    {DIM}  {action}{R}")


def _print_nav_summary(output):
    """Extract and display nav score from analyzer output."""
    # Parse nav score from terminal output
    for line in output.splitlines():
        if "Nav Score:" in line or "nav_score" in line.lower():
            print(f"  {line.strip()}")


# ─── Session Flows ────────────────────────────────────────────────────────────

def _flow_new_build(fc, info):
    """New build flow: safety check then baseline."""
    print(f"\n  {B}{C}--- New Build: Safety Check ---{R}")

    # Backup first - before any changes
    backup_path, diff_raw = _create_backup(fc, info)
    if backup_path is None:
        print(f"  {RED}Cannot proceed without a backup.{R}")
        return

    # Save diff to temp file for param analyzer
    diff_path = os.path.join("./blackbox", f"{info.get('craft_name', 'fc')}_diff.txt")
    os.makedirs("./blackbox", exist_ok=True)
    with open(diff_path, "w") as f:
        f.write(diff_raw)

    # Ask frame size
    frame = _prompt("What is your frame size (inches)?",
                    options=[("7", "7 inch"), ("10", "10 inch"),
                             ("12", "12 inch"), ("15", "15 inch")],
                    default="10")
    frame = int(frame)

    # Run parameter analyzer
    print(f"\n  Running safety check...")
    success, output = _run_param_check(diff_path, frame=frame)
    print(output)

    if not success:
        print(f"\n  {Y}Issues found. Review above and fix before flying.{R}")
    else:
        print(f"\n  {G}Config looks good for first flights.{R}")

    if _confirm("\n  Ready to analyze a flight?", default=True):
        _flow_tune_session(fc, info, frame=frame, diff_raw=diff_raw)


def _flow_tune_session(fc, info, frame=None, diff_raw=None):
    """Tuning session: download, analyze, apply, repeat.

    Backup is created before the first change is applied. If the backup
    fails, no changes will be made to the FC. The user can restore to
    the original state at any point during the session.
    """
    craft = info.get("craft_name", "fc")
    prev_score = None
    session_num = 0
    output_dir = "./blackbox"
    backup_path = None
    changes_applied = False

    # Pull diff if not already available
    if diff_raw is None:
        diff_raw = _pull_diff(fc, output_dir=output_dir, craft_name=craft)

    # Save diff file path for analyzer
    diff_file = None
    if diff_raw:
        diff_file = os.path.join(output_dir, f"{craft}_diff.txt")

    try:
        while True:
            session_num += 1
            print(f"\n  {B}{C}--- Tune: Session {session_num} ---{R}")

            # Download
            filepath = _download_blackbox(fc, info, output_dir=output_dir)
            if not filepath:
                if _confirm("Try again?"):
                    continue
                break

            # Analyze
            print(f"\n  Analyzing...\n")
            results = _run_analyzer(filepath, mode="tune", diff_file=diff_file,
                                    frame=frame)

            if not results["success"]:
                print(f"  {RED}Analysis failed:{R}")
                print(results.get("output", "")[:500])
                if _confirm("Try again?"):
                    continue
                break

            # Show results
            _print_score(results["score"], results["verdict"], prev_score)
            _print_actions(results["actions"], results.get("deferred"))

            if results.get("html_path"):
                print(f"\n  {DIM}Full report: {results['html_path']}{R}")

            prev_score = results["score"]

            # Extract CLI commands
            commands = _extract_cli_commands(results["actions"])
            if commands:
                print(f"\n  {B}CLI commands to apply:{R}")
                for cmd in commands:
                    print(f"    {C}{cmd}{R}")

                choice = _prompt("What do you want to do?",
                                 options=[
                                     ("a", "Apply these changes to the FC"),
                                     ("s", "Skip - fly again without changes"),
                                     ("r", "Restore original config and exit"),
                                     ("q", "Quit session"),
                                 ],
                                 default="a")

                if choice == "a":
                    # Create backup before first apply
                    if backup_path is None:
                        backup_path, _backup_diff = _create_backup(
                            fc, info, output_dir=output_dir)
                        if backup_path is None:
                            print(f"\n  {RED}Cannot proceed without a backup.{R}")
                            print(f"  {RED}No changes will be made to the FC.{R}")
                            break

                    _apply_commands(fc, commands)
                    changes_applied = True

                elif choice == "r":
                    if backup_path and changes_applied:
                        fc_new, ok = _restore_backup(fc, info, backup_path)
                        if fc_new:
                            fc.__dict__.update(fc_new.__dict__)
                        if ok:
                            print(f"  {G}Restored to original config.{R}")
                    elif not changes_applied:
                        print(f"  No changes were applied - nothing to restore.")
                    break

                elif choice == "q":
                    if changes_applied and backup_path:
                        print(f"\n  {Y}Changes were applied to the FC.{R}")
                        if _confirm("Restore original config before quitting?",
                                    default=False):
                            fc_new, ok = _restore_backup(fc, info, backup_path)
                            if fc_new:
                                fc.__dict__.update(fc_new.__dict__)
                    break

                elif choice == "s":
                    pass  # Fall through to erase/fly

            else:
                # No commands to apply
                if not _confirm("\n  Continue tuning?", default=True):
                    break

            # Erase and loop
            if _confirm("\n  Erase dataflash for next flight?", default=True):
                _erase_dataflash(fc)

            print(f"\n  {B}Go fly!{R}")
            print(f"  When you land, plug back in and press Enter.\n")
            input(f"  {DIM}Press Enter when ready...{R} ")

            # Refresh diff after applying changes
            diff_raw = _pull_diff(fc, output_dir=output_dir, craft_name=craft)
            if diff_raw:
                diff_file = os.path.join(output_dir, f"{craft}_diff.txt")

    except KeyboardInterrupt:
        print(f"\n\n  {Y}Session interrupted.{R}")

    # Session end - always remind about backup if changes were made
    if backup_path and changes_applied:
        print(f"\n  {B}Session backup:{R} {backup_path}")
        print(f"  To restore at any time, run the toolkit and")
        print(f"  choose 'Restore configuration from backup'.")


def _flow_nav_check(fc, info):
    """Nav health check flow."""
    craft = info.get("craft_name", "fc")
    output_dir = "./blackbox"

    print(f"\n  {B}{C}--- Nav Health Check ---{R}")

    diff_raw = _pull_diff(fc, output_dir=output_dir, craft_name=craft)
    diff_file = None
    if diff_raw:
        diff_file = os.path.join(output_dir, f"{craft}_diff.txt")

    filepath = _download_blackbox(fc, info, output_dir=output_dir)
    if not filepath:
        return

    print(f"\n  Analyzing nav health...\n")
    results = _run_analyzer(filepath, mode="nav", diff_file=diff_file)

    # Nav mode output goes to terminal directly via subprocess
    print(results.get("output", ""))

    if results.get("html_path"):
        print(f"\n  {DIM}Full report: {results['html_path']}{R}")

    if _confirm("\n  Erase dataflash?", default=False):
        _erase_dataflash(fc)


def _flow_download_only(fc, info):
    """Just download the blackbox log."""
    print(f"\n  {B}{C}--- Download Blackbox ---{R}")

    filepath = _download_blackbox(fc, info)
    if filepath:
        print(f"\n  {G}Saved: {filepath}{R}")

        if _confirm("Erase dataflash?", default=False):
            _erase_dataflash(fc)


def _flow_restore(fc, info):
    """Restore from a previous backup file."""
    print(f"\n  {B}{C}--- Restore Configuration ---{R}")

    # Find backup files
    backup_dir = "./blackbox"
    craft = info.get("craft_name") or "fc"
    backups = []
    if os.path.isdir(backup_dir):
        for f in sorted(os.listdir(backup_dir), reverse=True):
            if f.endswith(".txt") and "_backup_" in f:
                backups.append(os.path.join(backup_dir, f))

    if backups:
        print(f"\n  {B}Available backups:{R}")
        for i, bp in enumerate(backups[:5], 1):
            mtime = time.strftime("%Y-%m-%d %H:%M",
                                  time.localtime(os.path.getmtime(bp)))
            size = os.path.getsize(bp)
            print(f"    {C}{i}{R}  {os.path.basename(bp)}  "
                  f"{DIM}({mtime}, {size:,} bytes){R}")

        choice = _prompt("Choose a backup (number) or enter a file path:",
                         default="1")
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(backups):
                backup_path = backups[idx]
            else:
                print(f"  {RED}Invalid selection.{R}")
                return
        except ValueError:
            backup_path = choice
    else:
        backup_path = _prompt("Path to backup file:")

    if not backup_path or not os.path.isfile(backup_path):
        print(f"  {RED}File not found: {backup_path}{R}")
        return

    fc_new, ok = _restore_backup(fc, info, backup_path)
    if fc_new and fc_new is not fc:
        # Update the caller's reference
        fc.__dict__.update(fc_new.__dict__)


def _flow_offline():
    """Offline analysis: analyze an existing BBL file."""
    print(f"\n  {B}{C}--- Offline Analysis ---{R}")

    logfile = _prompt("Path to blackbox log file (.bbl):")
    if not logfile or not os.path.isfile(logfile):
        print(f"  {RED}File not found: {logfile}{R}")
        return

    mode = _prompt("Analysis mode:",
                   options=[("t", "PID tuning"), ("n", "Nav health")],
                   default="t")

    diff_file = None
    # Auto-discover diff
    log_dir = os.path.dirname(os.path.abspath(logfile))
    candidates = [f for f in os.listdir(log_dir)
                  if f.endswith("_diff.txt") or f in ("diff.txt", "diff_all.txt")]
    if candidates:
        diff_file = os.path.join(log_dir, candidates[0])
        print(f"  {DIM}Found config: {candidates[0]}{R}")

    print(f"\n  Analyzing...\n")
    results = _run_analyzer(logfile,
                            mode="nav" if mode == "n" else "tune",
                            diff_file=diff_file)

    if mode == "n":
        print(results.get("output", ""))
    else:
        if results["success"]:
            _print_score(results["score"], results["verdict"])
            _print_actions(results["actions"], results.get("deferred"))
        else:
            print(results.get("output", "")[:500])

    if results.get("html_path"):
        print(f"\n  {DIM}Full report: {results['html_path']}{R}")


# ─── Main Entry ───────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description=f"INAV Toolkit v{VERSION} - Guided Session Manager")
    parser.add_argument("--version", action="version", version=f"inav-toolkit {VERSION}")
    parser.add_argument("--device", metavar="PORT", default=None,
                        help="Serial port or 'auto' (e.g., /dev/ttyACM0, COM3)")
    parser.add_argument("--no-color", action="store_true",
                        help="Disable colored output")
    args = parser.parse_args()

    if args.no_color:
        global R, B, C, G, Y, RED, DIM
        R = B = C = G = Y = RED = DIM = ""

    _banner()

    # Try to connect to FC
    fc = None
    info = None

    if args.device:
        fc, info = _connect_fc(args.device)
    else:
        # Ask if FC is connected
        has_fc = _confirm("Is your flight controller connected via USB?", default=True)
        if has_fc:
            fc, info = _connect_fc("auto")

    if fc and info:
        _print_fc_info(info)

        try:
            mode = _prompt("What are we doing today?",
                           options=[
                               ("1", "Tuning session - analyze and improve"),
                               ("2", "Nav health check"),
                               ("3", "New build - safety check + baseline"),
                               ("4", "Just download the blackbox log"),
                               ("5", "Restore configuration from backup"),
                           ],
                           default="1")

            if mode == "1":
                _flow_tune_session(fc, info)
            elif mode == "2":
                _flow_nav_check(fc, info)
            elif mode == "3":
                _flow_new_build(fc, info)
            elif mode == "4":
                _flow_download_only(fc, info)
            elif mode == "5":
                _flow_restore(fc, info)

        except KeyboardInterrupt:
            print(f"\n\n  {DIM}Interrupted.{R}")
        finally:
            fc.close()
            print(f"\n  {DIM}Disconnected.{R}")
    else:
        # No FC - offer offline analysis
        if _confirm("\n  No FC connected. Analyze an existing log file?", default=True):
            try:
                _flow_offline()
            except KeyboardInterrupt:
                print(f"\n\n  {DIM}Interrupted.{R}")

    print(f"\n  {DIM}Fly safe.{R}\n")


if __name__ == "__main__":
    main()
