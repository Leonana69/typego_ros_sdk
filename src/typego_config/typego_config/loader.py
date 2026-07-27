"""Load robot.yaml, validate, and flatten to ROS-parameter dicts."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import yaml

from .schema import RobotConfig

_FLAT_EXCLUDE: Tuple[str, ...] = ('dynamic',)


class ConfigError(RuntimeError):
    """Raised when robot.yaml fails pydantic validation or cannot be read."""


def default_config_path() -> Path:
    env = os.environ.get('TYPEGO_CONFIG')
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('typego_config'))
        candidate = share / 'config' / 'robot.yaml'
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    here = Path(__file__).resolve()
    repo = here.parents[3]
    return repo / 'src' / 'typego_config' / 'config' / 'robot.yaml'


def load_raw(path: Optional[Path] = None) -> Dict[str, Any]:
    path = Path(path) if path else default_config_path()
    if not path.is_file():
        raise ConfigError(f'robot.yaml not found: {path}')
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f'robot.yaml at {path} is not valid YAML: {exc}') from exc
    if not isinstance(data, dict):
        raise ConfigError(f'robot.yaml at {path} must be a mapping at the top level')
    return data


def _profiles_dir(config_path: Path) -> Path:
    return config_path.parent / 'profiles'


def _resolve_profile(name: str, config_path: Path) -> Path:
    direct = Path(name)
    if direct.is_file():
        return direct
    candidate = _profiles_dir(config_path) / f'{name}.yaml'
    if candidate.is_file():
        return candidate
    raise ConfigError(
        f'profile {name!r} not found (looked for {direct} and {candidate})'
    )


def _deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge overlay into base. Overlay values win. Lists replace."""
    out: Dict[str, Any] = {}
    for key in set(base) | set(overlay):
        if key in overlay and isinstance(overlay[key], dict) \
                and isinstance(base.get(key), dict):
            out[key] = _deep_merge(base[key], overlay[key])
        elif key in overlay:
            out[key] = overlay[key]
        else:
            out[key] = base[key]
    return out


def load(path: Optional[Path] = None,
         profile: Optional[str] = None) -> RobotConfig:
    """Load + validate. Optionally overlay a named profile."""
    resolved = Path(path) if path else default_config_path()
    data = load_raw(resolved)
    if profile:
        overlay_path = _resolve_profile(profile, resolved)
        overlay_raw = load_raw(overlay_path)
        data = _deep_merge(data, overlay_raw)
    try:
        return RobotConfig.model_validate(data)
    except Exception as exc:
        # Name the file. With $TYPEGO_CONFIG set, or a profile overlaid, the
        # failing document is not necessarily the one the reader expects.
        detail = f'{resolved}'
        if profile:
            detail += f' (with profile {profile!r} overlaid)'
        raise ConfigError(
            f'robot.yaml validation failed: {detail}\n{exc}'
        ) from exc


def flatten(cfg: RobotConfig,
            prefix: str = '',
            exclude: Iterable[str] = _FLAT_EXCLUDE) -> Dict[str, Any]:
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


def dynamic_keys(cfg: RobotConfig) -> Tuple[str, ...]:
    return tuple(cfg.dynamic)
