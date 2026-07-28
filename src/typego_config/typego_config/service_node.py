"""Config service node — owns robot.yaml at runtime."""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
)
from rclpy.node import Node
from std_srvs.srv import Trigger

from .loader import ConfigError, default_config_path, flatten, load, to_json_dict
from .schema import RobotConfig

LOG = logging.getLogger('typego_config')


_PY_TO_ROS_TYPE = {
    bool: ParameterType.PARAMETER_BOOL,
    int: ParameterType.PARAMETER_INTEGER,
    float: ParameterType.PARAMETER_DOUBLE,
    str: ParameterType.PARAMETER_STRING,
}


def _descriptor_for(value: object, dotted_key: str, read_only: bool,
                    ranges: Dict[str, Tuple[float, float]]) -> ParameterDescriptor:
    desc = ParameterDescriptor()
    desc.name = dotted_key
    desc.read_only = read_only
    ros_type = _PY_TO_ROS_TYPE.get(type(value), ParameterType.PARAMETER_NOT_SET)
    desc.type = ros_type

    rng = ranges.get(dotted_key)
    if rng is not None:
        lo, hi = rng
        if isinstance(value, int) and not isinstance(value, bool):
            ir = IntegerRange(from_value=int(lo), to_value=int(hi), step=1)
            desc.integer_range = [ir]
        elif isinstance(value, float):
            fr = FloatingPointRange(
                from_value=float(lo), to_value=float(hi), step=0.0,
            )
            desc.floating_point_range = [fr]
    return desc


def _build_ranges_from_schema() -> Dict[str, Tuple[float, float]]:
    out: Dict[str, Tuple[float, float]] = {}

    def walk(model_cls, prefix: str) -> None:
        for name, info in model_cls.model_fields.items():
            dotted = f'{prefix}.{name}' if prefix else name
            lo: Optional[float] = None
            hi: Optional[float] = None
            for meta in info.metadata:
                cls_name = type(meta).__name__
                if cls_name == 'Ge':
                    lo = getattr(meta, 'ge', None)
                elif cls_name == 'Le':
                    hi = getattr(meta, 'le', None)
            if lo is not None and hi is not None:
                out[dotted] = (lo, hi)
            anno = info.annotation
            if hasattr(anno, 'model_fields'):
                walk(anno, dotted)

    walk(RobotConfig, '')
    return out


class ConfigServiceNode(Node):
    def __init__(self, config_path: Path):
        super().__init__('typego_config')
        self._config_path = config_path
        self._cfg = load(config_path)
        self._ranges = _build_ranges_from_schema()
        self._declared: List[str] = []
        self._declare_all()

        self.create_service(
            Trigger, 'typego_config/get_config', self._srv_get_config,
        )

        self.get_logger().info(
            f'loaded {len(self._declared)} parameters from {config_path}',
        )

    def _declare_all(self) -> None:
        # Static model: the resolved config is immutable for the life of the
        # launch, so every parameter is declared read-only. The node exists to
        # be *inspected* (`ros2 param list /typego_config`, get_config) -- not
        # to be mutated. Change robot.yaml and relaunch.
        flat = flatten(self._cfg)
        for key, value in flat.items():
            desc = _descriptor_for(value, key, True, self._ranges)
            try:
                # ignore_override=True: a `-p key:=value` at startup must not
                # win over the validated config. Read-only blocks runtime
                # set() but not launch-time overrides, so without this the
                # parameter table and get_config report different values.
                self.declare_parameter(
                    key, value, descriptor=desc, ignore_override=True)
            except Exception as exc:
                self.get_logger().error(
                    f'failed to declare {key!r}={value!r}: {exc}'
                )
                continue
            self._declared.append(key)

    def _srv_get_config(self, request, response):
        del request
        response.success = True
        response.message = json.dumps(to_json_dict(self._cfg))
        return response


def _parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog='config_service_node')
    parser.add_argument(
        '--config', default=os.environ.get('TYPEGO_CONFIG'),
        help='Path to robot.yaml; defaults to $TYPEGO_CONFIG or the '
             'shipped config/robot.yaml under typego_config share.',
    )
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    args = _parse_args(argv)
    path = Path(args.config) if args.config else default_config_path()

    rclpy.init(args=argv)
    try:
        node = ConfigServiceNode(path)
    except ConfigError as exc:
        print(str(exc), file=sys.stderr)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
