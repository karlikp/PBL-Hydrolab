"""Ground-station configuration: persisted site settings + provisioning source.

The config decouples *logical* tanks (C1/C2/C3) from the *physical*
channels they're wired to, so a rig that's plugged together differently
can be remapped here instead of rewiring or reflashing.

  - Each physical channel (1/2/3) is a fixed pump+sensor PAIR on the
    board (PUMP1+TOPCN1, PUMP2+TOPCN2, PUMP3+TOPCN3). They're wired
    together so they move as a unit.
  - A servo id is the SC-09 bus address programmed into the winch servo
    (a free integer, not a channel).
  - A logical tank picks one channel + one servo id, and can be disabled
    entirely (e.g. only tanks 1 and 3 mounted right now).

The active config lives in `config.json` (gitignored, site-specific,
created from DEFAULTS on first load). It's the single source of truth:
the GCS uses it for its own serial connection + level polling, and
provisions it to the ESP at startup / on save (the ESP holds it in RAM
only — no firmware persistence, consistent with the rest of the system).

Serial port/baud live here too, so the GCS no longer needs env-var
flags on the command line — set them once in the settings page.
"""

import copy
import json
import os
from typing import Optional

CONFIG_PATH = os.environ.get("CONFIG_PATH", "config.json")

TANK_IDS = ("C1", "C2", "C3")
VALID_CHANNELS = (1, 2, 3)

DEFAULTS = {
    "serial": {
        "port": "/dev/ttyUSB0",   # COMx on Windows; set in the settings page
        "baud": 115200,           # 57600 for the RFD868 radio
    },
    "level_poll_interval_s": 1.5, # 0 disables the live wet/dry poller
    "global": {
        "pumping_timeout_s": 90,  # hard cap on the PUMPING step
    },
    "tanks": {
        "C1": {"enabled": True, "channel": 1, "servo_id": 1},
        "C2": {"enabled": True, "channel": 2, "servo_id": 2},
        "C3": {"enabled": True, "channel": 3, "servo_id": 3},
    },
}


class ConfigError(ValueError):
    """Raised when a config fails validation. `errors` is a list of strings."""

    def __init__(self, errors: list[str]):
        self.errors = errors
        super().__init__("; ".join(errors))


def default_config() -> dict:
    return copy.deepcopy(DEFAULTS)


def _merge_defaults(cfg: dict) -> dict:
    """Fill any missing keys from DEFAULTS so older/partial files still load."""
    out = copy.deepcopy(DEFAULTS)
    if not isinstance(cfg, dict):
        return out
    if isinstance(cfg.get("serial"), dict):
        out["serial"].update({k: v for k, v in cfg["serial"].items()
                              if k in ("port", "baud")})
    if "level_poll_interval_s" in cfg:
        out["level_poll_interval_s"] = cfg["level_poll_interval_s"]
    if isinstance(cfg.get("global"), dict) and "pumping_timeout_s" in cfg["global"]:
        out["global"]["pumping_timeout_s"] = cfg["global"]["pumping_timeout_s"]
    if isinstance(cfg.get("tanks"), dict):
        for tid in TANK_IDS:
            t = cfg["tanks"].get(tid)
            if isinstance(t, dict):
                out["tanks"][tid].update({k: v for k, v in t.items()
                                          if k in ("enabled", "channel", "servo_id")})
    return out


def validate_config(cfg: dict) -> list[str]:
    """Return a list of human-readable errors ([] if valid)."""
    errors: list[str] = []

    baud = cfg.get("serial", {}).get("baud")
    if not isinstance(baud, int) or baud <= 0:
        errors.append("serial.baud must be a positive integer")
    if not str(cfg.get("serial", {}).get("port", "")).strip():
        errors.append("serial.port must not be empty")

    poll = cfg.get("level_poll_interval_s")
    if not isinstance(poll, (int, float)) or poll < 0:
        errors.append("level_poll_interval_s must be >= 0 (0 disables polling)")

    to = cfg.get("global", {}).get("pumping_timeout_s")
    if not isinstance(to, (int, float)) or to <= 0:
        errors.append("global.pumping_timeout_s must be a positive number")

    tanks = cfg.get("tanks", {})
    seen_channels: dict[int, str] = {}
    seen_servos: dict[int, str] = {}
    for tid in TANK_IDS:
        t = tanks.get(tid)
        if not isinstance(t, dict):
            errors.append(f"{tid} config missing")
            continue
        if not isinstance(t.get("enabled"), bool):
            errors.append(f"{tid}.enabled must be true/false")
        ch = t.get("channel")
        sv = t.get("servo_id")
        if ch not in VALID_CHANNELS:
            errors.append(f"{tid}.channel must be one of {VALID_CHANNELS}")
        if not isinstance(sv, int) or not (1 <= sv <= 253):
            errors.append(f"{tid}.servo_id must be an integer 1..253")
        # Collisions only matter among ENABLED tanks — two disabled tanks
        # can nominally share a channel without consequence.
        if t.get("enabled"):
            if ch in seen_channels:
                errors.append(
                    f"{tid} and {seen_channels[ch]} both use channel {ch} "
                    f"(pump+sensor pair can't be shared)")
            elif ch in VALID_CHANNELS:
                seen_channels[ch] = tid
            if isinstance(sv, int):
                if sv in seen_servos:
                    errors.append(
                        f"{tid} and {seen_servos[sv]} both use servo id {sv} "
                        f"(would collide on the bus)")
                else:
                    seen_servos[sv] = tid
    return errors


def errors_for(cfg: dict) -> list[str]:
    """Validation errors for an incoming (possibly partial) config —
    used by the validate-only endpoint so the UI can report what's wrong
    without saving."""
    return validate_config(_merge_defaults(cfg))


def load_config() -> dict:
    """Load the active config, creating it from DEFAULTS on first run.

    Always returns a fully-populated, defaults-merged dict. A corrupt or
    invalid file falls back to DEFAULTS rather than crashing the server.
    """
    if not os.path.exists(CONFIG_PATH):
        cfg = default_config()
        _write(cfg)
        return cfg
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        print(f"[Config] {CONFIG_PATH} unreadable ({e}); using defaults")
        return default_config()
    cfg = _merge_defaults(raw)
    if validate_config(cfg):
        print(f"[Config] {CONFIG_PATH} failed validation; using defaults")
        return default_config()
    return cfg


def save_config(cfg: dict) -> dict:
    """Validate, merge with defaults, persist, and return the stored config.
    Raises ConfigError on validation failure (nothing is written)."""
    merged = _merge_defaults(cfg)
    errors = validate_config(merged)
    if errors:
        raise ConfigError(errors)
    _write(merged)
    return merged


def reset_config() -> dict:
    cfg = default_config()
    _write(cfg)
    return cfg


def _write(cfg: dict):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")


def enabled_tanks(cfg: dict) -> list[str]:
    return [tid for tid in TANK_IDS if cfg["tanks"][tid].get("enabled")]


def channel_to_tank(cfg: dict) -> dict[int, str]:
    """Reverse map: physical channel → logical tank (enabled only)."""
    return {cfg["tanks"][tid]["channel"]: tid
            for tid in TANK_IDS if cfg["tanks"][tid].get("enabled")}
