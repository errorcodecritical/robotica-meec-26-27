"""Labwork 2 - algebraic inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        inverse_kinematics(T_d)                    every distinct solution for a pose
        select_solution(solutions, q_current)      the solution that is executed

    Part B - the node
        IkNode                                     reads a desired pose, publishes
                                                   the selected joint angles

Run the node with ``ros2 run lite6_ik ik_node``.

``transform_from_pose``, ``JOINT_LIMITS``, the constants ``TOL_SAME`` and
``TOL_WRIST`` and ``main`` below are PROVIDED: already implemented, do not edit
them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. ``test/test_inverse_kinematics.py`` checks the imports.
"""

import math

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:         # the maths and its unit tests also run without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'


# =============================================================================
# Part B - the maths
# =============================================================================
def inverse_kinematics(T_d) -> list:
    """Every distinct joint vector that puts the end-effector at T_d.

    The solution is closed-form, from the expressions of Part A: no iterative
    or optimisation method.

    Args:
        T_d: (4, 4) desired pose of {6} in {0}; translation in metres.

    Returns:
        A list of (6,) float arrays, joint angles in radians, order
        joint1..joint6, one array per distinct solution (in any order). The
        empty list if T_d is out of the robot's reach.

    Conventions of Part A:
        * every angle is in (-pi, pi]; the joint ranges JOINT_LIMITS are NOT
          applied here: a solution outside them is still returned;
        * two solutions in which no angle differs by more than TOL_SAME
          (angles compared modulo 2 pi) are the same solution: only one of
          them is returned;
        * if the angle between the axes of joints 4 and 6 is smaller than
          TOL_WRIST, those axes are aligned and every solution returned has
          q4 = 0.
    """
    T_d = np.asarray(T_d, dtype=float)
    if T_d.shape != (4, 4) or not np.all(np.isfinite(T_d)):
        return []

    # Removing the last-link offset gives the origin of frame {4}. Its
    # position depends only on joints 1-3, so position and orientation can be
    # solved independently.
    p04 = T_d[:3, 3] - 0.0615 * T_d[:3, 2]
    x, y = float(p04[0]), float(p04[1])
    z = float(p04[2] - 0.2433)
    radial = math.hypot(x, y)

    # In the shoulder plane, the remaining two links have lengths 0.2 and
    # hypot(0.2276, 0.087). The law of cosines gives the two elbow branches.
    link1 = 0.2
    link2 = math.hypot(0.2276, 0.087)
    reach_sq = radial * radial + z * z
    cos_elbow = (link1 * link1 + link2 * link2 - reach_sq) / (2.0 * link1 * link2)
    reach_slack = 1e-10
    if cos_elbow < -1.0 - reach_slack or cos_elbow > 1.0 + reach_slack:
        return []
    cos_elbow = min(1.0, max(-1.0, cos_elbow))

    elbow_offset = math.atan2(0.087, 0.2276)
    shoulder_branches = [(math.atan2(y, x), radial)] if radial > 1e-12 else [(0.0, 0.0)]
    if radial > 1e-12:
        shoulder_branches.append((shoulder_branches[0][0] + math.pi, -radial))

    solutions = []
    for q1, signed_radial in shoulder_branches:
        # q1 is not constrained by position on the base axis; q1=0 is the
        # deterministic representative used for that singular position.
        elbow_magnitude = math.acos(cos_elbow)
        for elbow_sign in (1.0, -1.0):
            q3 = elbow_sign * elbow_magnitude - elbow_offset
            phase = q3 + elbow_offset
            a = link1 - link2 * math.cos(phase)
            b = link2 * math.sin(phase)
            q2 = math.atan2(signed_radial, z) - math.atan2(b, a)

            c1, s1 = math.cos(q1), math.sin(q1)
            beta = q2 - q3
            cb, sb = math.cos(beta), math.sin(beta)
            R03 = np.array([
                [c1 * cb, c1 * sb, s1],
                [s1 * cb, s1 * sb, -c1],
                [-sb, cb, 0.0],
            ])
            R36 = R03.T @ T_d[:3, :3]
            cos_q5 = min(1.0, max(-1.0, -float(R36[1, 2])))
            sin_q5_magnitude = math.hypot(float(R36[1, 0]), float(R36[1, 1]))
            q5_magnitude = math.atan2(sin_q5_magnitude, cos_q5)

            if q5_magnitude < TOL_WRIST:
                # At the aligned-wrist singularity, the pose fixes q4+q6.
                wrist_branches = [(0.0, 0.0, math.atan2(R36[2, 0], R36[0, 0]))]
            elif math.pi - q5_magnitude < TOL_WRIST:
                # At the opposed-wrist singularity, choose q4=0; the pose
                # fixes q6-q4. This branch is normally outside joint limits.
                wrist_branches = [(0.0, math.pi, math.atan2(R36[2, 0], R36[2, 1]))]
            else:
                wrist_branches = []
                for q5 in (q5_magnitude, -q5_magnitude):
                    sin_q5 = math.sin(q5)
                    q4 = math.atan2(-R36[2, 2] / sin_q5, -R36[0, 2] / sin_q5)
                    q6 = math.atan2(R36[1, 1] / sin_q5, -R36[1, 0] / sin_q5)
                    wrist_branches.append((q4, q5, q6))

            for q4, q5, q6 in wrist_branches:
                q = np.array([q1, q2, q3, q4, q5, q6], dtype=float)
                q = (q + math.pi) % (2.0 * math.pi) - math.pi
                q[q <= -math.pi] += 2.0 * math.pi

                # Reject numerical branches that do not reproduce the target.
                # The wrist tolerance intentionally permits the tiny angular
                # approximation specified for singular configurations.
                T_check = forward_kinematics(q)
                position_error = np.linalg.norm(T_check[:3, 3] - T_d[:3, 3])
                relative_rotation = T_check[:3, :3].T @ T_d[:3, :3]
                cosine_error = min(1.0, max(-1.0, (np.trace(relative_rotation) - 1.0) / 2.0))
                orientation_error = math.acos(cosine_error)
                position_tolerance = 1e-8 + 0.0615 * TOL_WRIST
                if position_error > position_tolerance or orientation_error > max(1e-7, 2.0 * TOL_WRIST):
                    continue
                if not any(
                    np.all(np.abs((q - other + math.pi) % (2.0 * math.pi) - math.pi) <= TOL_SAME)
                    for other in solutions
                ):
                    solutions.append(q)

    return solutions


def select_solution(solutions, q_current):
    """The solution that is executed, chosen among `solutions`.

    Args:
        solutions: list of (6,) joint vectors (rad), as inverse_kinematics()
                   returns them. Their angles may be any real numbers: this
                   function must not assume they are in (-pi, pi].
        q_current: six joint angles (rad), the current configuration of the arm.

    Returns:
        A (6,) float array, the selected joint vector, or None if no solution
        is executable (no solutions at all, or all discarded). The inputs are
        not modified.

    The rule is the one of Part A: in each solution, every angle q_i is
    replaced by the value q_i + 2 pi k (k an integer) that lies inside the
    range of joint i (JOINT_LIMITS) and is closest to q_current[i]; a solution
    for which no such value exists is discarded. Among the solutions that
    remain, the one at the smallest euclidean distance from q_current is
    selected, and the returned vector is that solution with the replaced angles.
    """
    q_current = np.asarray(q_current, dtype=float)
    if q_current.shape != (6,) or not np.all(np.isfinite(q_current)):
        return None

    executable = []
    two_pi = 2.0 * math.pi
    for solution in solutions:
        q = np.asarray(solution, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            continue
        adjusted = np.empty(6, dtype=float)
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            k_min = math.ceil((lo - q[i]) / two_pi)
            k_max = math.floor((hi - q[i]) / two_pi)
            if k_min > k_max:
                break

            nearest_k = int(round((q_current[i] - q[i]) / two_pi))
            k = min(k_max, max(k_min, nearest_k))
            adjusted[i] = q[i] + two_pi * k
        else:
            executable.append(adjusted)

    if not executable:
        return None
    return min(executable, key=lambda q: float(np.linalg.norm(q - q_current))).copy()


# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                 joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node computes the solutions and
    selects one from the latest joint angles received. If there is a selected
    solution it publishes a JointState:
        name             JOINT_NAMES
        position         the selected solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                         ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs why:
        unreachable      the pose has no solution at all
        out_of_range     it has solutions, but all of them were discarded

    The topic names are constructor arguments so that the tests can run the
    node on private topics: use them, do not write the names again.
    """

    def __init__(self, target_topic='/lite6_ik/target', joint_states_topic='/joint_states',
                 solution_topic='/lite6_ik/solution'):
        super().__init__('ik_node')
        self.q_latest = None
        self.solution_pub = self.create_publisher(JointState, solution_topic, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self.on_joint_state, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, target_topic, self.on_target, 10)

    def on_joint_state(self, msg):
        """Called for every JointState: keep the latest joint angles."""
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in JOINT_NAMES):
            self.q_latest = np.array([by_name[name] for name in JOINT_NAMES], dtype=float)

    def on_target(self, msg):
        """Called for every desired pose: solve it and publish the solution."""
        if self.q_latest is None:
            self.get_logger().warning('No joint states received yet; ignoring target.')
            return

        solutions = inverse_kinematics(transform_from_pose(msg))
        if not solutions:
            self.get_logger().warning('unreachable: target pose has no inverse-kinematics solution.')
            return

        selected = select_solution(solutions, self.q_latest.copy())
        if selected is None:
            self.get_logger().warning('out_of_range: no inverse-kinematics solution is within joint limits.')
            return

        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(JOINT_NAMES)
        out.position = [float(angle) for angle in selected]
        self.solution_pub.publish(out)


# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
TOL_SAME = 1e-6     # rad - two solutions closer than this in every angle are the same
TOL_WRIST = 1e-6    # rad - angle between the axes of joints 4 and 6 below which they are aligned

# Joint ranges (rad) of the xarm_description model of the Lite 6, one row per
# joint, [lower, upper].
JOINT_LIMITS = np.array([
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.61799, 2.61799],
    [-0.061087, math.pi * 0.99],
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.1642, 2.1642],
    [-math.pi * 0.99, math.pi * 0.99],
])


def transform_from_pose(msg) -> np.ndarray:
    """(4, 4) transform of a geometry_msgs/PoseStamped.

    The quaternion is normalised first, so a slightly off unit quaternion still
    gives a rotation matrix.
    """
    p, o = msg.pose.position, msg.pose.orientation
    n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
    x, y, z, w = o.x / n, o.y / n, o.z / n, o.w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def main(args=None):
    """Entry point of ``ros2 run lite6_ik ik_node``."""
    if rclpy is None:
        raise SystemExit('ROS 2 is not available here: run this inside the container.')
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = IkNode()
        node.get_logger().info('ik_node running: desired poses in, joint angles out')
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it in lite6_ik/inverse_kinematics.py first.')
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())