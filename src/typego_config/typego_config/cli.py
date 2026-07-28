"""CLI entry point for robot.yaml operations."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import List, Optional

from .loader import ConfigError, default_config_path, flatten, load, to_json_dict
from .schema import RobotConfig


def _add_config_arg(sub: argparse.ArgumentParser) -> None:
    sub.add_argument(
        '--config', type=Path, default=None,
        help='Path to robot.yaml. Defaults to $TYPEGO_CONFIG, then the '
             'shipped typego_config/config/robot.yaml.',
    )
    sub.add_argument(
        '--profile', default=None,
        help='Optional profile name (e.g. go2_indoor) to overlay on robot.yaml.',
    )


def _load_or_die(path: Optional[Path], profile: Optional[str] = None) -> RobotConfig:
    try:
        return load(path, profile=profile)
    except ConfigError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)


def _cmd_validate(args: argparse.Namespace) -> int:
    cfg = _load_or_die(args.config, getattr(args, 'profile', None))
    resolved = args.config or default_config_path()
    print(f'OK — {resolved} validates against RobotConfig', file=sys.stderr)
    del cfg
    return 0


def _cmd_dryrun(args: argparse.Namespace) -> int:
    cfg = _load_or_die(args.config, getattr(args, 'profile', None))
    flat = flatten(cfg)
    width = max(len(k) for k in flat) if flat else 0
    for key in sorted(flat):
        print(f'{key.ljust(width)}  {flat[key]!r}')
    return 0


def _cmd_schema(args: argparse.Namespace) -> int:
    del args
    print(json.dumps(RobotConfig.model_json_schema(), indent=2, sort_keys=True))
    return 0


def _cmd_env(args: argparse.Namespace) -> int:
    from .env import render_env_text
    cfg = _load_or_die(args.config, getattr(args, 'profile', None))
    sys.stdout.write(render_env_text(cfg))
    return 0


def _cmd_render(args: argparse.Namespace) -> int:
    from .render import write_render_bundle
    cfg = _load_or_die(args.config, getattr(args, 'profile', None))
    out = args.out.resolve() if args.out else Path(os.environ.get(
        'TYPEGO_RENDER_DIR', '/tmp/typego-rendered'))
    files = write_render_bundle(cfg, out)
    for f in files:
        print(f)
    return 0


def _cmd_config_as_json(args: argparse.Namespace) -> int:
    cfg = _load_or_die(args.config, getattr(args, 'profile', None))
    print(json.dumps(to_json_dict(cfg), indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='typego-config')
    sub = p.add_subparsers(dest='cmd', required=True)

    pv = sub.add_parser('validate', help='validate robot.yaml')
    _add_config_arg(pv)
    pv.set_defaults(func=_cmd_validate)

    pd = sub.add_parser('dryrun', help='print resolved parameter tree')
    _add_config_arg(pd)
    pd.set_defaults(func=_cmd_dryrun)

    ps = sub.add_parser('schema', help='dump JSON schema')
    ps.set_defaults(func=_cmd_schema)

    pe = sub.add_parser('env', help='emit KEY=VALUE lines for shell / --env-file')
    _add_config_arg(pe)
    pe.set_defaults(func=_cmd_env)

    pr = sub.add_parser('render', help='write ROS 2 params file(s)')
    _add_config_arg(pr)
    pr.add_argument('--out', type=Path, default=None,
                    help='Output directory (default $TYPEGO_RENDER_DIR '
                         'or /tmp/typego-rendered).')
    pr.set_defaults(func=_cmd_render)

    pj = sub.add_parser('json', help='dump robot.yaml as JSON')
    _add_config_arg(pj)
    pj.set_defaults(func=_cmd_config_as_json)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
