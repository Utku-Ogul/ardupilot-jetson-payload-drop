"""
Loader for config.json.

Users edit `config.json` in the repository root; `suas/config.py` reads every value
through `get()`, falling back to a documented default when the key is absent or null.

Section names in the JSON exist purely for readability -- the loader flattens them, so a
key can be moved between sections without breaking anything. Keys and sections starting
with an underscore are treated as comments and ignored.

Unknown keys (typos) are reported loudly rather than silently ignored, because a
misspelled safety setting that silently keeps its default is exactly the kind of failure
this project cannot afford.
"""
import json
import os
import sys

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")

# Populated at import: {KEY: value} flattened from every section.
_user: dict = {}
# Every key config.py actually asked for -- used to detect typos and by tools/show_config.py.
_consumed: set = set()
# Non-fatal problems found while loading, surfaced by tools/show_config.py.
load_errors: list = []


def _warn(msg: str):
    load_errors.append(msg)
    print(f"[config] {msg}", file=sys.stderr)


def _load() -> dict:
    if not os.path.exists(CONFIG_PATH):
        _warn(f"{os.path.basename(CONFIG_PATH)} not found -- using built-in defaults. "
              f"Copy config.json from the repository to customise your setup.")
        return {}
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        _warn(f"config.json is not valid JSON ({e}) -- using built-in defaults. "
              f"A trailing comma or a missing quote is the usual cause.")
        return {}
    except OSError as e:
        _warn(f"could not read config.json ({e}) -- using built-in defaults.")
        return {}

    if not isinstance(data, dict):
        _warn("config.json must contain a JSON object -- using built-in defaults.")
        return {}

    flat = {}
    for section, values in data.items():
        if section.startswith("_"):
            continue                       # comment block
        if not isinstance(values, dict):
            _warn(f"section '{section}' is not an object -- skipped.")
            continue
        for key, value in values.items():
            if key.startswith("_"):
                continue
            name = key.upper()
            if name in flat:
                _warn(f"'{name}' is defined more than once -- the last one wins.")
            flat[name] = value
    return flat


_user = _load()


def get(name: str, default):
    """Return the user's value for `name`, or `default` if unset or null.

    `null` in JSON means "use the default", which lets a key stay visible in config.json
    as documentation without overriding anything.
    """
    _consumed.add(name)
    value = _user.get(name, None)
    return default if value is None else value


def unknown_keys() -> list:
    """Keys present in config.json that no setting consumes -- almost always typos."""
    return sorted(set(_user) - _consumed)


def overridden_keys() -> list:
    """Keys the user actually set (non-null) that the code consumes."""
    return sorted(k for k in _consumed if _user.get(k) is not None)


def report_unknown_keys():
    """Warn about unrecognised keys. Called once at the end of config.py."""
    unknown = unknown_keys()
    if unknown:
        _warn("unrecognised key(s) in config.json: " + ", ".join(unknown)
              + "  -- check the spelling; these are being IGNORED. "
                "Run 'python3 tools/show_config.py' to list valid settings.")
