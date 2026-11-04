#!/usr/bin/env python3
"""Connect to the configured Lite 6 and apply the required reduced-mode limits."""

from __future__ import annotations

import argparse
import ipaddress
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

MODE_POSITION = 0
SET_STATE_MOTION = 0
MAX_JOINT_SPEED_DEG_S = 54.0
MAX_TCP_SPEED_MM_S = 150.0
DEFAULT_CONFIG = 'config/robot.yaml'


def load_config(path: Path = DEFAULT_CONFIG) -> Mapping[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError('Install dependencies with: python -m pip install -r requirements.txt') from exc

    with path.expanduser().open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, Mapping):
        raise ValueError(f'{path} must contain a YAML mapping')
    return config


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{label} must be finite')
    return number


def _validated_settings(config: Mapping[str, Any]) -> tuple[str, float, float, list[float]]:
    robot_ip = config.get('robot_ip')
    if not isinstance(robot_ip, str) or not robot_ip.strip():
        raise ValueError('Set robot_ip in config/robot.yaml')
    try:
        ipaddress.ip_address(robot_ip)
    except ValueError as exc:
        raise ValueError('robot_ip must be a valid IP address') from exc

    safety = config.get('safety')
    if not isinstance(safety, Mapping) or safety.get('reduced_mode') is not True:
        raise ValueError('safety.reduced_mode must be true in config/robot.yaml')

    joint_speed = _number(safety.get('max_joint_speed_deg_s'),
                          'safety.max_joint_speed_deg_s')
    tcp_speed = _number(safety.get('max_tcp_speed_mm_s'),
                        'safety.max_tcp_speed_mm_s')
    if not 0 < joint_speed <= MAX_JOINT_SPEED_DEG_S:
        raise ValueError(f'max_joint_speed_deg_s must be in (0, {MAX_JOINT_SPEED_DEG_S:g}]')
    if not 0 < tcp_speed <= MAX_TCP_SPEED_MM_S:
        raise ValueError(f'max_tcp_speed_mm_s must be in (0, {MAX_TCP_SPEED_MM_S:g}]')

    raw_boundary = safety.get('tcp_boundary_mm')
    if not isinstance(raw_boundary, Sequence) or isinstance(raw_boundary, (str, bytes)) \
            or len(raw_boundary) != 6:
        raise ValueError(
            'Set safety.tcp_boundary_mm to the six bench-approved values in '
            '[x_max, x_min, y_max, y_min, z_max, z_min] order')
    boundary = [_number(v, f'safety.tcp_boundary_mm[{i}]')
                for i, v in enumerate(raw_boundary)]
    if any(boundary[i] <= boundary[i + 1] for i in (0, 2, 4)):
        raise ValueError('Each TCP boundary maximum must exceed its corresponding minimum')
    return robot_ip.strip(), joint_speed, tcp_speed, boundary


def _check(code: Any, operation: str) -> None:
    if isinstance(code, bool) or not isinstance(code, int):
        raise RuntimeError(f'{operation} returned invalid status code {code!r}')
    if code != 0:
        raise RuntimeError(f'{operation} failed with xArm status code {code}')


def safe_session(
    config_path: Path = DEFAULT_CONFIG,
    *,
    mode: int = MODE_POSITION,
    api_factory: Optional[Any] = None,
) -> Any:
    """Return a connected XArmAPI after applying reduced mode and session state."""
    config = load_config(config_path)
    robot_ip, joint_speed, tcp_speed, boundary = _validated_settings(config)
    if mode not in (MODE_POSITION, 2):
        raise ValueError('mode must be position control (0) or supervised joint teaching (2)')

    if api_factory is None:
        try:
            from xarm.wrapper import XArmAPI
        except ImportError as exc:
            raise RuntimeError(
                'xarm-python-sdk is missing; install requirements.txt') from exc
        api_factory = XArmAPI

    arm = api_factory(robot_ip, is_radian=False)
    try:
        if getattr(arm, 'connected', True) is False:
            raise RuntimeError(f'Could not connect to the Lite 6 at {robot_ip}')
        _check(arm.clean_warn(), 'clean_warn')
        _check(arm.clean_error(), 'clean_error')
        _check(arm.set_reduced_max_joint_speed(joint_speed, is_radian=False),
               'set_reduced_max_joint_speed')
        _check(arm.set_reduced_max_tcp_speed(tcp_speed), 'set_reduced_max_tcp_speed')
        _check(arm.set_reduced_tcp_boundary(boundary), 'set_reduced_tcp_boundary')
        _check(arm.set_reduced_mode(True), 'set_reduced_mode')
        _check(arm.motion_enable(enable=True), 'motion_enable')
        _check(arm.set_mode(mode), 'set_mode')
        _check(arm.set_state(SET_STATE_MOTION), 'set_state')
        return arm
    except Exception:
        try:
            _check(arm.motion_enable(enable=False), 'motion_enable(False)')
        except Exception as cleanup_error:
            print(f'Could not disable the arm after setup failed: {cleanup_error}',
                  file=sys.stderr)
        try:
            arm.disconnect()
        except Exception as cleanup_error:
            print(f'Could not disconnect after setup failed: {cleanup_error}',
                  file=sys.stderr)
        raise


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                        help='robot configuration (default: config/robot.yaml)')
    args = parser.parse_args(argv)
    arm = None
    try:
        arm = safe_session(args.config)
        print('Lite 6 connected; reduced mode and configured TCP boundary are active.')
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'safe session failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            try:
                _check(arm.motion_enable(enable=False), 'motion_enable(False)')
            except Exception as cleanup_error:
                print(f'Could not disable the arm when closing the session: {cleanup_error}',
                      file=sys.stderr)
            arm.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
