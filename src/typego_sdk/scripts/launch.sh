#!/bin/bash
# Everything is read from src/typego_config/config/robot.yaml.
# To override a single field for one run: pass `key:=value` on the
# command line (CLI args beat robot.yaml).
# To apply a profile overlay: TYPEGO_PROFILE=<name> (see
# src/typego_config/config/profiles/).
# To point at a different config file: TYPEGO_CONFIG=/abs/path/robot.yaml.
ros2 launch typego_sdk typego_bringup.launch.py "$@"
