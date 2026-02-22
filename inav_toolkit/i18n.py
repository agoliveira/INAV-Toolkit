"""Internationalization (i18n) for INAV Toolkit.

Lightweight JSON-based translation system. No external dependencies.

Usage:
    from inav_toolkit.i18n import t, set_locale

    set_locale("pt_BR")       # or "es", "en", etc.
    print(t("verdict.dialed_in"))
    print(t("quality.too_short", duration="1.2"))

Translation keys use dotted notation (e.g., "verdict.dialed_in").
Missing keys fall back to English. Missing English keys return the key itself.
Technical terms (PID, Hz, Roll/Pitch/Yaw, CLI commands) stay untranslated.
"""

import json
import os
import locale as _locale_mod

_catalogs = {}       # lang -> {key: translated_string}
_active_locale = "en"


def _find_locales_dir():
    """Find the locales directory, trying multiple strategies."""
    # Strategy 1: relative to this file (works for editable installs and source runs)
    candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)), "locales")
    if os.path.isdir(candidate) and any(f.endswith(".json") for f in os.listdir(candidate)):
        return candidate

    # Strategy 2: importlib.resources (works for installed wheels, Python 3.9+)
    try:
        from importlib.resources import files
        pkg_dir = str(files("inav_toolkit"))
        candidate = os.path.join(pkg_dir, "locales")
        if os.path.isdir(candidate):
            return candidate
    except (ImportError, TypeError, ModuleNotFoundError):
        pass

    # Strategy 3: walk up from __file__ looking for locales/en.json
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(3):
        candidate = os.path.join(here, "locales")
        if os.path.isdir(candidate):
            return candidate
        here = os.path.dirname(here)

    # Fallback: return the expected path even if missing (will just produce empty catalogs)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "locales")


_locales_dir = _find_locales_dir()


def _load_catalog(lang):
    """Load a locale JSON catalog. Returns dict or empty dict on failure."""
    if lang in _catalogs:
        return _catalogs[lang]

    path = os.path.join(_locales_dir, f"{lang}.json")
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                _catalogs[lang] = json.load(f)
        except (json.JSONDecodeError, OSError):
            _catalogs[lang] = {}
    else:
        _catalogs[lang] = {}

    return _catalogs[lang]


def set_locale(lang):
    """Set the active locale. Loads the catalog if not already loaded.

    Args:
        lang: Language code, e.g., "en", "pt_BR", "es".
              Also accepts formats like "pt-BR", "pt_BR.UTF-8".
    """
    global _active_locale

    # Normalize: "pt-BR" → "pt_BR", "pt_BR.UTF-8" → "pt_BR"
    lang = lang.replace("-", "_").split(".")[0]

    _load_catalog(lang)

    # Also preload base language (e.g., "pt" from "pt_BR") for fallback
    base = lang.split("_")[0]
    if base != lang:
        _load_catalog(base)

    # Always ensure English is loaded as final fallback
    _load_catalog("en")

    _active_locale = lang


def get_locale():
    """Return the current active locale code."""
    return _active_locale


def detect_locale():
    """Auto-detect locale from environment.

    Priority: INAV_LANG env var → LANG/LC_ALL → "en"
    """
    # Check INAV-specific env var first
    env_lang = os.environ.get("INAV_LANG", "")
    if env_lang:
        return env_lang.replace("-", "_").split(".")[0]

    # System locale
    try:
        sys_locale = _locale_mod.getdefaultlocale()[0] or ""
        if sys_locale:
            return sys_locale.replace("-", "_").split(".")[0]
    except (ValueError, AttributeError):
        pass

    # Fallback to LANG env var directly
    lang_env = os.environ.get("LANG", os.environ.get("LC_ALL", ""))
    if lang_env:
        return lang_env.replace("-", "_").split(".")[0]

    return "en"


def available_locales():
    """List available locale codes (based on JSON files in locales/)."""
    locales = []
    if os.path.isdir(_locales_dir):
        for f in sorted(os.listdir(_locales_dir)):
            if f.endswith(".json"):
                locales.append(f[:-5])
    return locales


def t(key, **kwargs):
    """Translate a key, with optional format substitution.

    Fallback chain: active locale → base language → English → key itself.

    Args:
        key: Dotted translation key, e.g., "verdict.dialed_in"
        **kwargs: Format substitution values, e.g., duration="1.2"

    Returns:
        Translated string with substitutions applied.

    Examples:
        t("verdict.dialed_in")
        t("quality.too_short", duration="1.2")
    """
    # Try active locale
    catalog = _catalogs.get(_active_locale, {})
    text = catalog.get(key)

    # Fallback to base language (e.g., "pt" from "pt_BR")
    if text is None:
        base = _active_locale.split("_")[0]
        if base != _active_locale:
            catalog = _catalogs.get(base, {})
            text = catalog.get(key)

    # Fallback to English
    if text is None:
        catalog = _catalogs.get("en", {})
        text = catalog.get(key)

    # Final fallback: return key itself
    if text is None:
        text = key

    # Apply format substitution
    if kwargs:
        try:
            text = text.format(**kwargs)
        except (KeyError, IndexError, ValueError):
            pass  # Return template as-is if substitution fails

    return text


# Auto-initialize English catalog on import
_load_catalog("en")
