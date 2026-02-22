"""Shared pytest fixtures for INAV Toolkit test suite."""
import os
import subprocess
import sys

import pytest

# Paths
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS_DIR = os.path.join(PROJECT_DIR, "tests")
FIXTURES_DIR = os.path.join(TESTS_DIR, "fixtures")


# ─── Helpers ──────────────────────────────────────────────────────────────────

@pytest.fixture(scope="session")
def project_dir():
    return PROJECT_DIR


@pytest.fixture(scope="session")
def fixtures_dir():
    return FIXTURES_DIR


def run_module(module, *args, cwd=None):
    """Run a toolkit module via python -m, return (stdout, stderr, rc)."""
    result = subprocess.run(
        [sys.executable, "-m", module, *args],
        capture_output=True, text=True, cwd=cwd or PROJECT_DIR)
    return result.stdout, result.stderr, result.returncode


def run_entry(cmd, *args, cwd=None):
    """Run an entry point command, return (stdout, stderr, rc)."""
    result = subprocess.run(
        [cmd, *args],
        capture_output=True, text=True, cwd=cwd or PROJECT_DIR)
    return result.stdout, result.stderr, result.returncode


# ─── Lazy-loaded module fixtures ─────────────────────────────────────────────

@pytest.fixture(scope="session")
def blackbox_module():
    """Import the blackbox analyzer module once per session."""
    from inav_toolkit import blackbox_analyzer
    return blackbox_analyzer


@pytest.fixture(scope="session")
def param_module():
    """Import the param analyzer module once per session."""
    from inav_toolkit import param_analyzer
    return param_analyzer


# ─── Synthetic flight data fixtures ──────────────────────────────────────────

@pytest.fixture(scope="session")
def clean_hover_data(blackbox_module):
    """Parse the clean hover CSV fixture into a data dict."""
    csv_path = os.path.join(FIXTURES_DIR, "clean_hover.csv")
    if not os.path.exists(csv_path):
        pytest.skip("Fixture clean_hover.csv not found — run generate_fixtures.py")
    return blackbox_module.parse_csv_log(csv_path)


@pytest.fixture(scope="session")
def noisy_motors_data(blackbox_module):
    """Parse the noisy motors CSV fixture into a data dict."""
    csv_path = os.path.join(FIXTURES_DIR, "noisy_motors.csv")
    if not os.path.exists(csv_path):
        pytest.skip("Fixture noisy_motors.csv not found — run generate_fixtures.py")
    return blackbox_module.parse_csv_log(csv_path)


@pytest.fixture(scope="session")
def over_tuned_data(blackbox_module):
    """Parse the over-tuned CSV fixture into a data dict."""
    csv_path = os.path.join(FIXTURES_DIR, "over_tuned.csv")
    if not os.path.exists(csv_path):
        pytest.skip("Fixture over_tuned.csv not found — run generate_fixtures.py")
    return blackbox_module.parse_csv_log(csv_path)


@pytest.fixture(scope="session")
def diff_path():
    """Path to the basic diff fixture."""
    p = os.path.join(TESTS_DIR, "test_basic_diff.txt")
    if not os.path.exists(p):
        pytest.skip("Fixture test_basic_diff.txt not found")
    return p


@pytest.fixture(scope="session")
def vtol_diff_path():
    """Path to the VTOL diff fixture."""
    p = os.path.join(TESTS_DIR, "test_vtol_diff.txt")
    if not os.path.exists(p):
        pytest.skip("Fixture test_vtol_diff.txt not found")
    return p
