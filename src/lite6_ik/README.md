# `lite6_ik` — Labwork 2

Algebraic (closed-form) inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node —
Robotics (02000537), 2026/2027. The handout is the specification; this file is the map of
the package.

This package is **new and separate** from `lite6_kinematics`: it sits beside it and uses it.
Your Labwork 1 `forward_kinematics.py` stays where it is and is not touched; this package
imports it (`from lite6_kinematics.forward_kinematics import ...`). Everything you edit in
Labwork 2 is in this folder.

## The files

| File | | Purpose |
|---|---|---|
| `lite6_ik/inverse_kinematics.py` | **LW2: to implement** | `inverse_kinematics`, `select_solution` and the node `IkNode`. The only file you edit in LW2. It also holds `transform_from_pose`, `JOINT_LIMITS`, `TOL_SAME`, `TOL_WRIST` and `main`, which are **provided** — do not edit them |
| `lite6_ik/ik_check.py` | provided | checker node: moves the arm to a starting configuration, sends your node a desired pose, executes the solution it answers with, and compares the pose reached with the desired one. With `all:=true` it executes every solution of a pose instead |
| `lite6_ik/ik_study.py` | provided | studies on your `inverse_kinematics()`: `count` (500 poses) and `wrist` (100 poses). No Gazebo, no arm |
| `lite6_ik/__init__.py` | provided | empty; makes the folder a Python package |
| `config/lw2_targets.yaml` | provided | the starting configurations S0–S4 and the desired poses A–D and U that `ik_check` uses |
| `test/test_inverse_kinematics.py` | provided | the unit tests; the node test needs ROS, so it runs inside the container |
| `package.xml` | provided | the ROS 2 package description and its dependencies |
| `setup.py` | provided | installs the package and registers the `ik_node`, `ik_check` and `ik_study` commands |
| `setup.cfg` | provided | where `ros2 run` finds the installed commands |
| `resource/lite6_ik` | provided | empty marker that registers the package with ROS 2 |
| `README.md` | provided | this file |

The provided files are compared against the originals at grading. Edit them and your
results do not count.

`inverse_kinematics.py` may import `numpy` and `math` for the maths, `forward_kinematics`
from your own Labwork 1 (already imported in the template), and `rclpy`, `sensor_msgs` and
`geometry_msgs` for the node (already imported in the template). Nothing else.

## Getting the Labwork 2 files

Labwork 2 only **adds** a new folder, `src/lite6_ik`. In the repository root:

```powershell
git pull
```

Nothing you edited before is touched, and `lite6_kinematics` is not changed. If git
complains about a file, tell the instructor before you overwrite anything.

## Conventions

Those of Labwork 1: frame {0} = `link_base`, frame {6} = `link_eef`, joint angles in
**radians**, in the controller's convention, lengths in **metres**. The desired pose is the
pose of `link_eef` in `link_base`. Your node `ik_node` subscribes to `/lite6_ik/target`
(`geometry_msgs/PoseStamped`) and `/joint_states`, and publishes on `/lite6_ik/solution`
(`sensor_msgs/JointState`) the joint angles selected for the target — if there is one, else it
logs `unreachable` (no solution) or `out_of_range` (all solutions discarded). The
`header.stamp` of the solution is the stamp of the target: that is how `ik_check` knows which
target it answers.

## 1. Tests

Inside the container, after `lab build` (below):

```bash
python3 -m pytest ~/dev_ws/src/lite6_ik/test -v
```

All green before you go further. They need your Labwork 1 `forward_kinematics()` to work, and
use it: they check structure, units and internal consistency — not whether your forward
kinematics is right; that is what Gazebo and the arm are for. The import, constants and
`transform_from_pose` tests pass from the start: they guard the provided code, not your work.

## 2. Gazebo

Once, to register the new package and its commands:

```bash
lab build
```

Then open a new shell.

**Terminal 1:**

```bash
lab sim-lite
```

(To look at the robot, `lab rviz` in another terminal.)

**Terminal 2** — your node:

```bash
ros2 run lite6_ik ik_node
```

**Terminal 3** — the checker:

```bash
ros2 run lite6_ik ik_check --ros-args -p robot:=sim
```

For each of the five desired poses in `config/lw2_targets.yaml` the checker moves the arm to
the starting configuration S0 and records the angles it reaches, sends your node the desired
pose and waits 5 s for the solution, then moves the arm to the solution and compares the pose
it reaches (from tf, i.e. from the robot model) with the desired pose. One of the five is out
of the robot's reach: your node must not answer, and the checker then records the pose as
**refused** and does not move the arm. Result: `~/shared/lw2_ik_sim.csv`.

As in Labwork 1, the robot model differs slightly from the manufacturer's sheet, so a correct
implementation shows about 1 mm of position error here. That is within P2.4.

## 3. The real arm

Only after Gazebo is done: every pose reached, the out-of-reach pose refused. The rules of
Labwork 0 part 1 §3 apply without exception: reduced mode on, one group member at the
emergency stop, no hands within the workspace while the robot is enabled.

**Terminal 1:**

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — your node, the same one:

```bash
ros2 run lite6_ik ik_node
```

**Terminal 3:**

```bash
ros2 run lite6_ik ik_check --ros-args -p robot:=real
```

The same poses, slowly, and **it waits for Enter before every motion**. Ctrl+C quits between
motions; **it does not stop a motion in progress — the emergency stop does.** Besides tf, it
compares with the pose the arm's own controller reports (`/ufactory/robot_states`). Result:
`~/shared/lw2_ik_real.csv`.

## 4. The analyses

`ik_study` needs neither Gazebo nor the arm. It builds desired poses from joint vectors (the
same every time) with your `forward_kinematics()` and solves them with your
`inverse_kinematics()`:

```bash
ros2 run lite6_ik ik_study count
```

```bash
ros2 run lite6_ik ik_study wrist
```

They write `~/shared/lw2_study_count.csv` (E.1: 500 poses from random joint vectors inside the
joint ranges) and `lw2_study_wrist.csv` (E.4: 100 poses from joint vectors with the axes of
joints 4 and 6 aligned).

E.2 and E.3 use the checker, with Gazebo running. E.2 executes every solution of pose A that is
inside the joint ranges (your node is not needed: `ik_check` calls your `inverse_kinematics()`;
the arm stays 5 s at each solution, for a screenshot; `-p hold_s:=` changes it):

```bash
ros2 run lite6_ik ik_check --ros-args -p robot:=sim -p target:=A -p all:=true
```

E.3 needs your node running, once per starting configuration:

```bash
ros2 run lite6_ik ik_check --ros-args -p robot:=sim -p target:=A -p start:=S1
```

then `start:=S2`, `S3` and `S4`. Each run writes its own file, `~/shared/lw2_ik_sim_A_S1.csv`
and so on.

## The files in `shared/`

| File | Written by | Contents |
|---|---|---|
| `lw2_ik_<robot>.csv` | `ik_check` | one row per desired pose: the angles the arm started from, the desired pose, the solution, the angles and the pose reached (tf, and the controller on the real arm), the errors in mm and degrees, and the result: `reached`, `refused` (no solution within 5 s) or `rejected` (a solution outside the joint ranges, never executed) |
| `lw2_ik_<robot>_<target>_<start>.csv` | `ik_check` | the same, for a run with `-p target:=` or `-p start:=` |
| `lw2_ik_<robot>_<target>_all.csv` | `ik_check -p all:=true` | one row per solution of the pose: its number, how many solutions the pose has and how many are inside the joint ranges, the solution, and — for those executed — the angles and the pose reached |
| `lw2_ik_<robot>_once.csv` | `ik_check -p once:=true` | the same, for the checkpoint: the desired pose is the pose the end-effector has now; appended to on every run |
| `lw2_study_count.csv` | `ik_study count` | one row per pose: the joint vector it came from, the number of solutions, the number inside the joint ranges, whether the joint vector is among the solutions, and the largest error over all solutions |
| `lw2_study_wrist.csv` | `ik_study wrist` | one row per solution of each pose: the joint vector the pose came from, the solution, its error, and `q4 + q6` of both |

Do not edit these files: the logged solutions are checked against what your submitted code
computes from the logged starting angles.
