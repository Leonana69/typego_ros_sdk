"""Dependency-free robot.yaml discovery and merging.

This module deliberately imports **nothing but PyYAML**. It is the one place
that knows how to find robot.yaml and how a profile overlay is applied, so the
launch files, the pydantic loader, and any future consumer all resolve the same
file the same way. Three hand-copied forks of this logic previously drifted
apart -- one raised on error, two swallowed it silently.

Keeping it pydantic-free matters: ``typego_config/__init__.py`` is empty, so
``from typego_config.bootstrap import ...`` succeeds even on an interpreter
that cannot import pydantic. Consumers that want validation import
``typego_config.loader`` on top; consumers that only need a raw dict do not
pay for it.

Precedence, applied identically everywhere:

    $TYPEGO_CONFIG  ->  installed typego_config share  ->  source tree
"""
from __future__ import annotations

import os
from typing import Any, Dict, Optional, Tuple

import yaml


class ConfigNotFound(RuntimeError):
    """robot.yaml could not be located."""


class ConfigUnreadable(RuntimeError):
    """robot.yaml was found but could not be read or parsed as a mapping."""


def locate_robot_yaml() -> Optional[str]:
    """Resolve robot.yaml, or None when no candidate exists.

    Order: $TYPEGO_CONFIG, the installed ``typego_config`` share directory,
    then the in-tree source copy. The share directory is preferred over the
    source tree because that is what a launched node actually reads -- colcon
    copies rather than symlinks it, so the two can differ until a rebuild.

    A $TYPEGO_CONFIG that does not resolve is an **error**, not a reason to
    fall back: the caller named a specific file, and quietly launching the
    shipped config instead is how a stack ends up running geometry nobody
    asked for.
    """
    env = os.environ.get('TYPEGO_CONFIG')
    if env:
        if os.path.isfile(env):
            return env
        raise ConfigNotFound(
            f'$TYPEGO_CONFIG points at {env!r}, which is not a readable file. '
            f'Fix the path or unset TYPEGO_CONFIG to use the shipped config; '
            f'falling back silently would launch a different configuration '
            f'than the one requested.'
        )

    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory('typego_config'), 'config', 'robot.yaml')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    # …/src/typego_config/typego_config/bootstrap.py -> repo root
    here = os.path.abspath(__file__)
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(here))))
    candidate = os.path.join(
        repo, 'src', 'typego_config', 'config', 'robot.yaml')
    return candidate if os.path.isfile(candidate) else None


def load_raw(path: str) -> Dict[str, Any]:
    """Parse one YAML file, requiring a mapping at the top level."""
    if not os.path.isfile(path):
        raise ConfigNotFound(f'robot.yaml not found: {path}')
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as exc:
        raise ConfigUnreadable(f'{path} is not valid YAML: {exc}') from exc
    if not isinstance(data, dict):
        raise ConfigUnreadable(f'{path} must be a mapping at the top level')
    return data


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge overlay into base. Overlay wins; lists replace."""
    out: Dict[str, Any] = {}
    for key in set(base) | set(overlay):
        if key in overlay and isinstance(overlay[key], dict) \
                and isinstance(base.get(key), dict):
            out[key] = deep_merge(base[key], overlay[key])
        elif key in overlay:
            out[key] = overlay[key]
        else:
            out[key] = base[key]
    return out


def resolve_profile(name: str, config_path: str) -> str:
    """Locate a profile overlay: a direct path, or profiles/<name>.yaml."""
    if os.path.isfile(name):
        return name
    candidate = os.path.join(
        os.path.dirname(config_path), 'profiles', f'{name}.yaml')
    if os.path.isfile(candidate):
        return candidate
    raise ConfigNotFound(
        f'profile {name!r} not found (looked for {name} and {candidate})')


def load_merged(path: Optional[str] = None,
                profile: Optional[str] = None) -> Tuple[Dict[str, Any], str]:
    """Return ``(merged_dict, resolved_path)`` with no schema validation.

    ``profile`` defaults to $TYPEGO_PROFILE so that every consumer honours it
    without each having to remember to.
    """
    resolved = path or locate_robot_yaml()
    if not resolved:
        raise ConfigNotFound(
            'robot.yaml not found: $TYPEGO_CONFIG is unset or missing, there '
            'is no config/robot.yaml in the typego_config share directory, '
            'and no in-tree copy could be located'
        )
    data = load_raw(resolved)
    name = profile if profile is not None else os.environ.get('TYPEGO_PROFILE')
    if name:
        data = deep_merge(data, load_raw(resolve_profile(name, resolved)))
    return data, resolved
