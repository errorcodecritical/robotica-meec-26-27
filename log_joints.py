#!/usr/bin/env python3
"""Log and plot Lite 6 joint angles during the supervised hand-guiding exercise."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Sequence

from safe_session import DEFAULT_CONFIG, safe_session

JOINT_COUNT = 6

def _get_angles(arm: Any) -> list[float]:
    code, angles = arm.get_servo_angle()
    if isinstance(code, bool) or not isinstance(code, int):
        raise RuntimeError(f'get_servo_angle returned invalid status code {code!r}')
    if code != 0:
        raise RuntimeError(f'get_servo_angle failed with xArm status code {code}')
    if not isinstance(angles, Sequence) or isinstance(angles, (str, bytes)) \
            or len(angles) < JOINT_COUNT:
        raise RuntimeError('get_servo_angle did not return six joint angles')
    result = [float(value) for value in angles[:JOINT_COUNT]]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError('get_servo_angle returned a non-finite joint angle')
    return result


def record(arm: Any, output_dir: Path, duration: float) -> tuple[Path, Path, float]:
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be a finite positive number')
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    csv_path = output_dir / f'joint_log_{stamp}.csv'
    plot_path = output_dir / f'joint_log_{stamp}.png'
    started = time.monotonic()
    count = 0
    with csv_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['elapsed_s', *(f'joint{i}_deg' for i in range(1, 7))])
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= duration:
                break
            angles = _get_angles(arm)
            elapsed = time.monotonic() - started
            if elapsed >= duration:
                break
            writer.writerow([f'{elapsed:.6f}', *(f'{angle:.6f}' for angle in angles)])
            file.flush()
            count += 1

    if count == 0:
        raise RuntimeError('No joint samples were collected')
    elapsed = time.monotonic() - started

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            'matplotlib is missing; install dependencies with requirements.txt') from exc

    with csv_path.open(encoding='utf-8') as file:
        rows = list(csv.reader(file))[1:]
    times = [float(row[0]) for row in rows]
    figure, axis = plt.subplots()
    for joint in range(JOINT_COUNT):
        axis.plot(times, [float(row[joint + 1]) for row in rows],
                  label=f'joint{joint + 1}')
    axis.set(xlabel='Time (s)', ylabel='Joint angle (degrees)',
             title='Lite 6 joint angles')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(plot_path)
    plt.close(figure)
    return csv_path, plot_path, count / elapsed


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, default=Path('logs'))
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args(argv)
    arm = None
    try:
        arm = safe_session(args.config, mode=2)
        print('Hand-guide only under supervision; confirm payload and safety area first.')
        csv_path, plot_path, rate = record(arm, args.output_dir, args.duration)
        print(f'Saved {csv_path} and {plot_path}; sample rate {rate:.2f} Hz')
        return 0
    except KeyboardInterrupt:
        print('Joint logging interrupted.', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'joint logging failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            try:
                code = arm.motion_enable(enable=False)
                if code != 0:
                    raise RuntimeError(f'motion_enable(False) failed with code {code}')
            except Exception as cleanup_error:
                print(f'Could not disable the arm when closing the session: {cleanup_error}',
                      file=sys.stderr)
            finally:
                arm.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
