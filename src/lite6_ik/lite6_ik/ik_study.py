"""ik_study - LW2 studies of the solutions. PROVIDED, DO NOT EDIT.

Generates desired poses from joint vectors, solves them with YOUR
inverse_kinematics() and writes the results to a CSV file. It needs neither
Gazebo nor the arm. The poses are forward_kinematics(q) of joint vectors drawn
with a fixed seed: the same poses in every run (they change only if your
forward_kinematics does).

    ros2 run lite6_ik ik_study count    500 poses from random joint vectors inside the joint ranges
                                         -> ~/shared/lw2_study_count.csv
    ros2 run lite6_ik ik_study wrist    100 poses from joint vectors with the axes of joints 4
                                         and 6 aligned (q5 = 0)
                                         -> ~/shared/lw2_study_wrist.csv

count - one row per pose: index, q_true1..6 (the joint vector the pose came from),
n_solutions, n_in_range (solutions that can be executed: every angle, plus a multiple
of 2 pi, inside its joint range), found (the joint vector it came from is among the
solutions, up to multiples of 2 pi), ep_mm and eo_deg (the largest distance between
forward_kinematics(solution) and the desired pose, over all the solutions).

wrist - one row per solution of each pose: index, q_true1..6, n_solutions, solution,
q1..6, ep_mm and eo_deg of that solution, and q4_plus_q6 of the joint vector the pose came
from and of the solution (rad, in (-pi, pi]).
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import Counter

import numpy as np

from lite6_kinematics.checking import pose_errors
from lite6_kinematics.forward_kinematics import forward_kinematics
from lite6_ik.inverse_kinematics import JOINT_LIMITS, inverse_kinematics

N_COUNT = 500
N_WRIST = 100
SEED = 2627
TOL_FOUND = 1e-6        # rad - "the joint vector is among the solutions"


def wrap(a):
    """Angle in (-pi, pi]."""
    a = np.remainder(np.asarray(a, dtype=float) + math.pi, 2.0 * math.pi) - math.pi
    return np.where(a <= -math.pi, a + 2.0 * math.pi, a)


def in_range(q):
    """True if every angle of q, plus a multiple of 2 pi, can be inside its joint range."""
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)):
        return False
    for i in range(6):
        lo, hi = JOINT_LIMITS[i]
        k = math.ceil((lo - q[i]) / (2.0 * math.pi))
        if q[i] + 2.0 * math.pi * k > hi:
            return False
    return True


def make_count_poses():
    rng = np.random.default_rng(SEED)
    q_true = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(N_COUNT, 6))
    return q_true, [np.asarray(forward_kinematics(q), dtype=float) for q in q_true]


def make_wrist_poses():
    rng = np.random.default_rng(SEED + 1)
    q_true = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(N_WRIST, 6))
    q_true[:, 4] = 0.0                      # the axes of joints 4 and 6 are aligned
    return q_true, [np.asarray(forward_kinematics(q), dtype=float) for q in q_true]


def errors(solutions, T_d):
    """(ep_mm, eo_deg) of each solution against T_d."""
    out = []
    for s in solutions:
        e_p, e_o = pose_errors(np.asarray(forward_kinematics(s), dtype=float), T_d)
        out.append((e_p * 1e3, math.degrees(e_o)))
    return out


def write(rows, name, output_dir):
    out = os.path.expanduser(output_dir)
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, name)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return path


def histogram(values, title):
    counts = Counter(values)
    print(f'\n  {title}')
    for n in range(min(counts), max(counts) + 1):
        c = counts.get(n, 0)
        print(f'  {n:3d}  {c:4d}  {100.0 * c / len(values):5.1f} %  {"#" * round(50.0 * c / len(values))}')


def run_count(output_dir):
    q_true, poses = make_count_poses()
    rows = []
    t0 = time.monotonic()
    print(f'solving {len(poses)} poses ...', end='', flush=True)
    for i, (qt, T_d) in enumerate(zip(q_true, poses)):
        sols = [np.asarray(s, dtype=float).reshape(6) for s in inverse_kinematics(T_d)]
        err = errors(sols, T_d)
        found = any(np.all(np.abs(wrap(s - qt)) <= TOL_FOUND) for s in sols)
        row = {'index': i}
        row.update({f'q_true{j + 1}': qt[j] for j in range(6)})
        row.update({'n_solutions': len(sols), 'n_in_range': sum(in_range(s) for s in sols),
                    'found': int(found),
                    'ep_mm': max((e[0] for e in err), default=float('nan')),
                    'eo_deg': max((e[1] for e in err), default=float('nan'))})
        rows.append(row)
    print(f' {time.monotonic() - t0:.1f} s')
    histogram([r['n_solutions'] for r in rows], 'solutions per pose')
    histogram([r['n_in_range'] for r in rows], 'solutions inside the joint ranges per pose')
    print(f"\n  joint vector among the solutions: {sum(r['found'] for r in rows)} of {len(rows)}")
    finite = [r for r in rows if r['n_solutions']]
    if finite:
        print(f"  largest error over all solutions: {max(r['ep_mm'] for r in finite):.3g} mm, "
              f"{max(r['eo_deg'] for r in finite):.3g} deg")
    print(f"\nwritten: {write(rows, 'lw2_study_count.csv', output_dir)}")


def run_wrist(output_dir):
    q_true, poses = make_wrist_poses()
    rows = []
    for i, (qt, T_d) in enumerate(zip(q_true, poses)):
        sols = [np.asarray(s, dtype=float).reshape(6) for s in inverse_kinematics(T_d)]
        err = errors(sols, T_d)
        base = {'index': i}
        base.update({f'q_true{j + 1}': qt[j] for j in range(6)})
        base.update({'n_solutions': len(sols), 'q4_plus_q6_true': float(wrap(qt[3] + qt[5]))})
        if not sols:
            rows.append(dict(base, solution='', **{f'q{j + 1}': '' for j in range(6)},
                             ep_mm='', eo_deg='', q4_plus_q6=''))
        for k, (s, (e_p, e_o)) in enumerate(zip(sols, err), start=1):
            row = dict(base, solution=k)
            row.update({f'q{j + 1}': s[j] for j in range(6)})
            row.update({'ep_mm': e_p, 'eo_deg': e_o, 'q4_plus_q6': float(wrap(s[3] + s[5]))})
            rows.append(row)
    n_sol = Counter(r['n_solutions'] for r in rows if r['solution'] != '' and r['solution'] == 1)
    print(f'{len(poses)} poses with q5 = 0, {len(rows)} solutions in all; solutions per pose: '
          f'{dict(sorted(n_sol.items()))}')
    aligned = [r for r in rows if r['solution'] != '' and abs(float(r['q5'])) < 1e-6]
    if aligned:
        print(f"  {len(aligned)} solutions have the axes of joints 4 and 6 aligned (|q5| < 1e-6); "
              f"largest |q4| among them: {max(abs(float(r['q4'])) for r in aligned):.3g} rad")
    print(f"\nwritten: {write(rows, 'lw2_study_wrist.csv', output_dir)}")


def main(args=None):
    parser = argparse.ArgumentParser(prog='ik_study', description=__doc__.split('\n')[0])
    parser.add_argument('mode', choices=['count', 'wrist'])
    parser.add_argument('--output-dir', default='~/shared')
    opts = parser.parse_args(sys.argv[1:] if args is None else args)
    try:
        {'count': run_count, 'wrist': run_wrist}[opts.mode](opts.output_dir)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it first (forward_kinematics.py, inverse_kinematics.py).',
              file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
