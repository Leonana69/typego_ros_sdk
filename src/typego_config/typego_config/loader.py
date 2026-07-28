"""Load robot.yaml, validate with pydantic, and flatten to ROS parameters.

Discovery and merging live in :mod:`typego_config.bootstrap`, which is
dependency-free; this module is the pydantic layer on top. Import bootstrap
directly when you need the raw dict without paying for validation.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from .bootstrap import (
    ConfigNotFound,
    ConfigUnreadable,
    deep_merge,
    load_merged,
    load_raw as _load_raw_str,
    locate_robot_yaml,
)
from .schema import RobotConfig


class ConfigError(RuntimeError):
    """Raised when robot.yaml fails validation or cannot be read."""


# Kept as module-level aliases: callers and tests reference these names.
_deep_merge = deep_merge


def default_config_path() -> Path:
    found = locate_robot_yaml()
    if found:
        return Path(found)
    # Preserve the historical behaviour of naming where it *would* live.
    here = Path(__file__).resolve()
    return here.parents[3] / 'src' / 'typego_config' / 'config' / 'robot.yaml'


def load_raw(path: Optional[Path] = None) -> Dict[str, Any]:
    target = str(path) if path else str(default_config_path())
    try:
        return _load_raw_str(target)
    except (ConfigNotFound, ConfigUnreadable) as exc:
        raise ConfigError(str(exc)) from exc


def load(path: Optional[Path] = None,
         profile: Optional[str] = None) -> RobotConfig:
    """Load, overlay an optional profile, and validate."""
    try:
        data, resolved = load_merged(str(path) if path else None, profile)
    except (ConfigNotFound, ConfigUnreadable) as exc:
        raise ConfigError(str(exc)) from exc
    try:
        return RobotConfig.model_validate(data)
    except Exception as exc:
        # Name the file. With $TYPEGO_CONFIG set, or a profile overlaid, the
        # failing document is not necessarily the one the reader expects.
        detail = resolved
        if profile:
            detail += f' (with profile {profile!r} overlaid)'
        raise ConfigError(
            f'robot.yaml validation failed: {detail}\n{exc}'
        ) from exc


def flatten(cfg: RobotConfig,
            prefix: str = '',
            exclude: Iterable[str] = ()) -> Dict[str, Any]:
    """Flatten nested fields into dotted-key form for ROS parameters."""
    exclude_set = set(exclude)
    out: Dict[str, Any] = {}

    def walk(value: Any, key: str) -> None:
        if key in exclude_set:
            return
        if hasattr(value, 'model_dump'):
            for sub_name, _ in value.__class__.model_fields.items():
                child_key = f'{key}.{sub_name}' if key else sub_name
                walk(getattr(value, sub_name), child_key)
            return
        out[key] = value

    walk(cfg, prefix)
    return out


def to_json_dict(cfg: RobotConfig) -> Dict[str, Any]:
    return cfg.model_dump(mode='json')
