"""LW2 public tests - PROVIDED, DO NOT EDIT.

    python3 -m pytest ~/dev_ws/src/lite6_ik/test -v

These check structure, units and internal consistency - not whether your
forward kinematics is right: they use it, and cannot tell. That is what
ik_check (Gazebo, then the arm) is for. They need your Labwork 1
forward_kinematics() to work. The node test needs ROS, so it runs inside the
container and is skipped elsewhere.
"""

import ast
import itertools
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, PACKAGE)
sys.path.insert(1, os.path.join(os.path.dirname(PACKAGE), 'lite6_kinematics'))    # your Labwork 1

from lite6_kinematics import forward_kinematics as fk  # noqa: E402
from lite6_ik import inverse_kinematics as ik  # noqa: E402

SOURCE = os.path.join(PACKAGE, 'lite6_ik', 'inverse_kinematics.py')
ALLOWED_IMPORTS = {'numpy', 'math', 'typing', '__future__', 'rclpy', 'sensor_msgs', 'geometry_msgs'}
ALLOWED_MODULES = {'lite6_kinematics.forward_kinematics'}

LOW = np.array([-3.11, -2.61, -0.06, -3.11, -2.16, -3.11])
HIGH = np.array([3.11, 2.61, 3.11, 3.11, 2.16, 3.11])
RNG = np.random.default_rng(2627)
SAMPLES = RNG.uniform(LOW, HIGH, size=(40, 6))
TOL_Q = 1e-6                # rad - "this joint vector is that one"
TOL_FK = 1e-8               # m, rad - "this joint vector reproduces that pose"


def FK(q):
    try:
        return np.asarray(fk.forward_kinematics(q), dtype=float)
    except NotImplementedError:
        pytest.fail('forward_kinematics() is not implemented: finish Labwork 1 first', pytrace=False)


def IK(T):
    try:
        return [np.asarray(s, dtype=float) for s in ik.inverse_kinematics(T)]
    except NotImplementedError:
        pytest.fail('inverse_kinematics() is not implemented yet', pytrace=False)


def SELECT(solutions, q_current):
    try:
        return ik.select_solution(solutions, q_current)
    except NotImplementedError:
        pytest.fail('select_solution() is not implemented yet', pytrace=False)


def wrap(a):
    """Angle(s) in (-pi, pi]."""
    a = np.remainder(np.asarray(a, dtype=float) + math.pi, 2.0 * math.pi) - math.pi
    return np.where(a <= -math.pi, a + 2.0 * math.pi, a)


def same(a, b, tol):
    """True if no angle of a and b differs by more than tol (modulo 2 pi)."""
    return bool(np.all(np.abs(wrap(np.asarray(a) - np.asarray(b))) <= tol))


def _errors(T_c, T_d):
    """(position distance in m, orientation angle in rad) between two poses."""
    R = T_c[:3, :3].T @ T_d[:3, :3]
    s = np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
    return float(np.linalg.norm(T_c[:3, 3] - T_d[:3, 3])), float(np.arctan2(s, (np.trace(R) - 1.0) / 2.0))


def _pose(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


# --- rules --------------------------------------------------------------------
def test_only_allowed_modules_are_imported():
    tree = ast.parse(open(SOURCE).read())
    bad = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name not in ALLOWED_MODULES and a.name.split('.')[0] not in ALLOWED_IMPORTS:
                    bad.add(a.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ''
            if module in ALLOWED_MODULES:
                continue
            if module == 'lite6_kinematics' and all(a.name == 'forward_kinematics' for a in node.names):
                continue
            if module.split('.')[0] not in ALLOWED_IMPORTS:
                bad.add(module)
        elif isinstance(node, ast.Call) and getattr(node.func, 'id', '') in ('__import__', 'exec', 'eval', 'open'):
            bad.add(node.func.id)
    assert not bad, f'not allowed in inverse_kinematics.py: {sorted(bad)}'


# --- provided code (these pass from day one) ------------------------------------------
# A regression test of the code you were given, not of your work: if one of
# these fails, something provided has been edited by mistake - restore it.
def test_provided_constants():
    assert (ik.TOL_SAME, ik.TOL_WRIST) == (1e-6, 1e-6)
    assert ik.JOINT_LIMITS.shape == (6, 2) and np.all(ik.JOINT_LIMITS[:, 0] < ik.JOINT_LIMITS[:, 1])
    assert np.all(ik.JOINT_LIMITS[:, 0] <= LOW) and np.all(ik.JOINT_LIMITS[:, 1] >= HIGH)


def test_transform_from_pose_normalises_and_orders_the_quaternion():
    class Msg:
        pass
    msg = Msg()
    msg.pose = Msg()
    msg.pose.position = Msg()
    msg.pose.orientation = Msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 0.1, -0.2, 0.3
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = 0.0, 0.0, 2.0 * math.sin(math.pi / 4), 2.0 * math.cos(math.pi / 4)   # 90 deg about z, not unit
    T = ik.transform_from_pose(msg)
    assert np.allclose(T[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
    assert np.allclose(T[:3, 3], [0.1, -0.2, 0.3]) and np.allclose(T[3], [0, 0, 0, 1])


# --- inverse_kinematics ---------------------------------------------------------------------
def test_solutions_have_the_right_form_and_reproduce_the_pose():
    for q in SAMPLES:
        T = FK(q)
        solutions = IK(T)
        assert len(solutions) > 0, f'no solution for a reachable pose (from {np.round(q, 3).tolist()})'
        for s in solutions:
            assert s.shape == (6,) and np.all(np.isfinite(s))
            assert np.all(s > -math.pi) and np.all(s <= math.pi), 'every angle must be in (-pi, pi]'
            e_p, e_o = _errors(FK(s), T)
            assert e_p < TOL_FK and e_o < TOL_FK, \
                f'a solution does not reproduce the pose: e_p = {e_p:.3g} m, e_o = {e_o:.3g} rad'


def test_the_joint_vector_of_origin_is_among_the_solutions():
    for q in SAMPLES:
        solutions = IK(FK(q))
        assert any(same(s, q, TOL_Q) for s in solutions), \
            f'the joint vector {np.round(q, 4).tolist()} is not among the solutions'


def test_no_solution_is_returned_twice():
    poses = [FK(q) for q in SAMPLES]
    for q in SAMPLES[:10]:                              # and some with the axes of joints 4 and 6 aligned
        q = q.copy()
        q[4] = 0.0
        poses.append(FK(q))
    for T in poses:
        for a, b in itertools.combinations(IK(T), 2):
            assert not same(a, b, ik.TOL_SAME), 'two solutions differ by less than TOL_SAME'


def _elbow_boundary_q3():
    """The values of q3 at which the arm is as stretched, and as folded, as it can be.

    Found numerically with your forward kinematics: the distance between the origins of {2} and
    {4}, as a function of q3, is largest when the arm is stretched and smallest when it is folded.
    """
    def reach(q3):
        F = fk.forward_kinematics(np.array([0.0, 0.0, q3, 0.0, 0.0, 0.0]), all_frames=True)
        return float(np.linalg.norm(np.asarray(F[4])[:3, 3] - np.asarray(F[2])[:3, 3]))

    grid = np.linspace(-math.pi, math.pi, 1441)
    values = np.array([reach(q3) for q3 in grid])
    found = []
    for sign, i in ((1.0, int(np.argmax(values))), (-1.0, int(np.argmin(values)))):
        a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
        for _ in range(80):                             # golden-section search
            c, d = b - 0.618033988 * (b - a), a + 0.618033988 * (b - a)
            if sign * reach(c) > sign * reach(d):
                b = d
            else:
                a = c
        found.append(0.5 * (a + b))
    return found


def test_the_arm_stretched_or_folded_has_no_missing_or_repeated_solution():
    for q3 in _elbow_boundary_q3():
        for q1, q2, q4, q5, q6 in ((0.3, -0.4, 0.5, 0.9, -0.2), (-2.0, 0.8, -1.0, -1.2, 2.0),
                                   (0.3, -0.4, math.pi, 0.9, -0.2), (0.3, -0.4, 0.5, 0.9, math.pi)):
            q = np.array([q1, q2, q3, q4, q5, q6])
            T = FK(q)
            solutions = IK(T)
            assert solutions, 'no solution for the arm stretched (or folded): that pose is reachable'
            assert any(same(s, q, TOL_Q) for s in solutions), 'the joint vector of origin is missing'
            for a, b in itertools.combinations(solutions, 2):
                assert not same(a, b, ik.TOL_SAME), 'a solution is returned twice'
            for s in solutions:
                e_p, e_o = _errors(FK(s), T)
                assert e_p < TOL_FK and e_o < TOL_FK


def test_a_pose_out_of_reach_has_no_solution():
    for p in ([2.0, 0.0, 0.3], [-1.5, 1.5, 0.5], [0.0, 0.0, 3.0], [0.0, 0.0, -2.0]):
        for R in (np.eye(3), FK(SAMPLES[3])[:3, :3]):
            solutions = IK(_pose(R, p))
            assert isinstance(solutions, list) and solutions == [], f'a solution for a pose out of reach {p}'


def test_aligned_wrist_has_q4_zero_and_reproduces_the_pose():
    for q in SAMPLES[:20]:
        q = q.copy()
        q[4] = 0.0                                      # axes of joints 4 and 6 aligned
        T = FK(q)
        aligned = [s for s in IK(T) if abs(wrap(s[4])) < ik.TOL_WRIST]
        assert aligned, 'the solution with the axes of joints 4 and 6 aligned is missing'
        for s in aligned:
            assert abs(s[3]) < 1e-12, 'with the axes of joints 4 and 6 aligned, q4 must be 0'
            e_p, e_o = _errors(FK(s), T)
            assert e_p < TOL_FK and e_o < TOL_FK
            assert abs(wrap(s[3] + s[5] - (q[3] + q[5]))) < 1e-9, 'q4 + q6 is what the pose fixes'


def test_wrist_just_inside_and_just_outside_the_tolerance():
    for q in SAMPLES[:10]:
        inside, outside = q.copy(), q.copy()
        inside[4] = 0.5 * ik.TOL_WRIST
        outside[4] = 10.0 * ik.TOL_WRIST
        T = FK(inside)
        near = [s for s in IK(T) if abs(wrap(s[4])) < 2 * ik.TOL_WRIST]
        assert near and all(abs(s[3]) < 1e-12 for s in near), 'q4 must be 0 inside the tolerance'
        for s in near:
            assert _errors(FK(s), T)[1] < 3 * ik.TOL_WRIST
        T = FK(outside)
        assert any(same(s, outside, TOL_Q) for s in IK(T)), 'outside the tolerance q4 is determined'
        for s in IK(T):
            e_p, e_o = _errors(FK(s), T)
            assert e_p < TOL_FK and e_o < TOL_FK


# --- select_solution --------------------------------------------------------------------------
Q_A = np.array([0.4, 0.5, 0.6, -0.7, 0.8, 0.9])
Q_B = np.array([-0.4, -0.5, 1.6, 0.7, -0.8, -0.9])


def test_select_solution_returns_a_vector_and_does_not_modify_its_inputs():
    solutions = [Q_A.copy(), Q_B.copy()]
    q_current = Q_A + 0.01
    q = SELECT(solutions, q_current)
    assert isinstance(q, np.ndarray) and q.shape == (6,) and q.dtype.kind == 'f'
    assert np.allclose(q, Q_A)
    assert np.array_equal(solutions[0], Q_A) and np.array_equal(solutions[1], Q_B)
    assert np.allclose(q_current, Q_A + 0.01)


def test_select_solution_picks_the_closest():
    assert np.allclose(SELECT([Q_A, Q_B], Q_B + 0.05), Q_B)
    assert np.allclose(SELECT([Q_B, Q_A], Q_B + 0.05), Q_B)
    assert np.allclose(SELECT([Q_B, Q_A], Q_A - 0.05), Q_A)
    assert np.allclose(SELECT([Q_A], Q_B), Q_A)


def test_select_solution_respects_the_joint_ranges():
    lo, hi = ik.JOINT_LIMITS[:, 0], ik.JOINT_LIMITS[:, 1]
    for i in range(6):
        for beyond in (hi[i] + 0.05, lo[i] - 0.05):
            bad = Q_A.copy()
            bad[i] = beyond                             # the closest to q_current, but outside its range
            q = SELECT([bad, Q_B], Q_A)
            assert q is not None and np.allclose(q, Q_B), f'joint {i + 1}: a solution outside its range'
            assert SELECT([bad], Q_A) is None


def test_select_solution_replaces_angles_by_multiples_of_2pi():
    two_pi = 2.0 * math.pi
    shifted = Q_A + two_pi * np.array([1, -1, 0, 2, -2, 1])
    q = SELECT([shifted], Q_A + 0.1)
    assert q is not None and np.allclose(q, Q_A)
    # the distance is measured after the replacement
    near, far = Q_A + two_pi * np.array([1, 0, 0, 0, 0, 0]), Q_A + 0.2
    assert np.allclose(SELECT([far, near], Q_A + 0.02), Q_A)
    # a solution that no multiple of 2 pi brings inside the range is discarded
    gap = Q_A.copy()
    gap[0] = math.pi                                    # between the upper limit and the lower one + 2 pi
    assert SELECT([gap], Q_A) is None


def test_select_solution_returns_none_when_nothing_is_executable():
    assert SELECT([], Q_A) is None
    out = ik.JOINT_LIMITS[:, 1] + 0.1
    assert SELECT([out, out + 0.2], Q_A) is None
    assert SELECT([out], Q_A) is None


# --- the node (needs ROS) -----------------------------------------------------------------
def test_node_publishes_the_selected_solution_for_the_target(monkeypatch, capfd):
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    prefix = f'/lw2_test_{os.getpid()}'
    q_true = SAMPLES[11]
    q_start = np.clip(q_true + RNG.uniform(-0.2, 0.2, 6), LOW, HIGH)
    order = [3, 0, 5, 1, 4, 2]                  # shuffled, plus a joint that is not ours
    state = JointState()
    state.name = [f'joint{i + 1}' for i in order] + ['gripper']
    state.position = [float(q_start[i]) for i in order] + [0.5]

    def target(stamp, T):
        msg = PoseStamped()
        msg.header.frame_id = 'link_base'
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
        x, y, z, w = fk.quaternion_from_matrix(T[:3, :3])
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        return msg

    currents = []                               # what the node selects from
    real_select = ik.select_solution

    def spy(solutions, q_current, *args, **kwargs):
        currents.append(np.asarray(q_current, dtype=float).copy())
        return real_select(solutions, q_current, *args, **kwargs)

    monkeypatch.setattr(ik, 'select_solution', spy)

    T_ok = FK(q_true)
    T_far = _pose(T_ok[:3, :3], [2.0, 0.0, 0.3])
    q_out = np.array([0.3, -0.4, -0.3, 0.5, 0.9, -0.2])     # reachable, but every solution is outside a joint range
    T_out = FK(q_out)
    good, far, out = target((123, 456789), T_ok), target((124, 1), T_far), target((125, 2), T_out)

    rclpy.init(args=[])
    got = []
    try:
        node = ik.IkNode(target_topic=prefix + '/target', joint_states_topic=prefix + '/joint_states',
                         solution_topic=prefix + '/solution')
        helper = Node('lw2_test_helper')
        helper.create_subscription(JointState, prefix + '/solution', got.append, 10)
        states = helper.create_publisher(JointState, prefix + '/joint_states', 10)
        targets = helper.create_publisher(PoseStamped, prefix + '/target', 10)

        def spin_for(seconds, stop=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not stop():
                states.publish(state)
                rclpy.spin_once(node, timeout_sec=0.02)
                rclpy.spin_once(helper, timeout_sec=0.02)

        end = time.monotonic() + 10.0
        while not got and time.monotonic() < end:       # until discovery and the first joint state are done
            targets.publish(good)
            spin_for(0.1, lambda: bool(got))
        n_good = len(got)
        capfd.readouterr()                              # from here on: only what the other two poses log
        for _ in range(10):                             # a pose out of reach, and one with every solution out of range
            targets.publish(far)
            targets.publish(out)
            spin_for(0.2)
        logged = capfd.readouterr()
        node.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()

    assert n_good, 'the node published nothing on solution_topic'
    stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in got}
    assert (124, 1) not in stamps, 'the node published a solution for a pose out of reach'
    assert (125, 2) not in stamps, 'the node published a solution outside the joint ranges'
    assert stamps == {(123, 456789)}, 'header.stamp must be the stamp of the target'
    msg = got[0]
    index = {n: i for i, n in enumerate(msg.name)}
    assert all(f'joint{i}' in index for i in range(1, 7)), 'name must hold joint1 ... joint6'
    q = np.array([msg.position[index[f'joint{i}']] for i in range(1, 7)])
    e_p, e_o = _errors(FK(q), T_ok)
    assert e_p < TOL_FK and e_o < TOL_FK, 'the solution must reach the desired pose'
    expected = real_select(IK(T_ok), q_start)
    assert expected is not None and np.allclose(q, expected, atol=1e-9), \
        'the node must publish the solution selected from the latest joint angles'
    assert currents and all(np.allclose(c, q_start, atol=1e-12) for c in currents), \
        'the node must select from the latest joint angles, picked by name from the JointState'
    text = logged.out + logged.err
    assert 'unreachable' in text, 'the node must log unreachable for a pose with no solution'
    assert 'out_of_range' in text, 'the node must log out_of_range when every solution is discarded'
