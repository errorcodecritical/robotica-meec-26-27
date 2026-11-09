# Lab 0 environment — Ubuntu / Docker

**Robotics (02000537) — 2026/2027** · UFACTORY Lite 6 · ROS 2 Jazzy Jalisco · MoveIt 2 · Gazebo Harmonic

Path **C** of the Lab 0 handout, for Linux. Everything in Part 1 (Steps 1–8) is
baked into the image — including the `xarm_ros2` clone and its `colcon build` — so
that time is spent once, by `docker build`, and never again.

> The `windows` branch has the Windows version of the same environment, with the
> identical `lab` commands. This branch is the Linux one. The
> [Dockerfile](Dockerfile) is commented step by step against the handout.
>
> This branch also carries `isaac-sim/`, an unrelated NVIDIA Isaac Sim setup that
> shares nothing with Lab 0 — see [CLAUDE.md](CLAUDE.md). Ignore it for the labs.

---

## First time

Docker Engine and the compose plugin. No GPU required.

```bash
sudo apt install -y docker.io docker-compose-v2
```

Add yourself to the `docker` group so you do not need `sudo` for every command —
log out and back in for it to take effect:

```bash
sudo usermod -aG docker $USER
```

Then, in this folder:

```bash
cp .env.example .env
```

Put your assigned `ROS_DOMAIN_ID` in `.env`, then build — once, mostly downloads:

```bash
./lab.sh build
```

```bash
./lab.sh up
```

```bash
./lab.sh doctor
```

`doctor` checks ROS, the xarm packages, the display, the GL renderer and both
workspaces. Run it before asking anyone anything.

Every terminal after that is `./lab.sh shell`, which drops you into the same
running container. Inside, `lab` is the menu. `./lab.sh down` stops everything;
your work is kept.

---

# The development pipeline

**This is how you work.** Two terminals: one runs the robot, the other runs your
node. Write your node once and it runs against both.

## 1. In simulation

```bash
./lab.sh gazebo on
```

**Terminal 1** — Gazebo, MoveIt and `ros2_control`, all from one launch:

```bash
./lab.sh shell
```

```bash
lab sim
```

Wait for the Lite 6 to appear in the Gazebo window standing under gravity — up to
a minute, and several minutes the very first time while Gazebo downloads models.

**Terminal 2** — your node:

```bash
./lab.sh shell
```

```bash
ros2 run lite6_control joint_echo
```

## 2. On the real robot

**Prerequisite: it ran in Gazebo first.** That is the course rule, and Test 7 is
the gate.

**Terminal 1** — the same stack, with real hardware underneath:

```bash
./lab.sh shell
```

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — the *identical* command as in simulation, not edited:

```bash
./lab.sh shell
```

```bash
ros2 run lite6_control joint_echo
```

Set `ROBOT_IP` in `.env` and you can drop the address: `lab real`.

## Why the same node works in both

`lab sim` and `lab real` load the same `xarm_controller/config/lite6_controllers.yaml`,
so both expose the same interface:

```
/lite6_traj_controller/follow_joint_trajectory     control_msgs/action/FollowJointTrajectory
/joint_states                                      sensor_msgs/msg/JointState
```

Only the hardware behind it changes — `gz_ros2_control` in simulation,
`uf_robot_hardware` on the arm. A node written against that action does not know
or care which, and that is what makes "Gazebo first, then hardware" a real
workflow rather than a slogan.

Watch the tracking error your node prints. Against fake controllers it is ~1e-5 rad;
in Gazebo it is far larger, because Gazebo simulates mass, gravity and friction
and the arm sags and overshoots. That difference *is* Test 7.

## The other path: talking to the driver directly

```bash
lab driver 192.168.1.xxx
```

This starts the bare xArm driver instead — `/ufactory/set_servo_angle`,
`/ufactory/motion_enable` and friends, with **no `controller_manager` at all**.
Lower level and more direct, and what `move_joints_demo` and `jog_demo` use.

**It has no Gazebo equivalent**, so a node written this way cannot be tested in
simulation. Use it when you specifically want the driver API; use `lab sim` /
`lab real` for anything you want to develop in simulation first.

**One owner of the arm at a time.** `lab real` hands the arm to `ros2_control`,
which holds it in **mode 1** (servo streaming). `set_servo_angle` needs **mode 0**.
Run one or the other — with both, your service calls are silently ignored. Check
with:

```bash
ros2 topic echo /ufactory/robot_states --once
```

---

## Your code

`src/lite6_control` is a worked package — see [its README](src/lite6_control/README.md).

| Node | Works with | |
|---|---|---|
| `joint_echo` | anything | minimal subscriber: subscribe, spin, print |
| `move_joints_demo` | `lab driver` | self-contained; the driver API, read and command |
| `jog_demo` | `lab driver` | one joint at a time, via the reusable `Lite6Client` |
| `fk_node`, `fk_check` | `lab sim`, `lab real` | Labwork 1, from `src/lite6_kinematics`: you write `fk_node` (joint states in, end-effector pose out); `fk_check` checks it against the model and the robot — see [its README](src/lite6_kinematics/README.md) |
| `ik_node`, `ik_check`, `ik_study` | `lab sim`, `lab real` (`ik_study`: nothing) | Labwork 2, from `src/lite6_ik` (a new package beside `lite6_kinematics`): you write `ik_node` (desired pose in, selected joint angles out, by the algebraic solution); `ik_check` checks it against the model and the robot; `ik_study` runs the studies — see [its README](src/lite6_ik/README.md) |
| *(the Newton version of Labwork 2 in `src/lite6_kinematics`)* | | superseded by `src/lite6_ik`; left in place so that nothing you already edited is touched. It is not what Labwork 2 asks for now |

Start your own package in `src/`:

```bash
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy control_msgs trajectory_msgs
```

```bash
lab build
```

`lab build` uses `--symlink-install`, so **editing a `.py` needs no rebuild**.
Rebuild only when you add an entry point, change `package.xml`/`setup.py`, or
create a package — and re-source (or just open a new shell) only for a brand-new
package.

---

## Part 2 — the seven tests

| Handout | Inside the container | Gazebo |
|---|---|---|
| 1 — ROS 2 core is alive | `lab test 1`, and `lab test 1b` in a second terminal | no |
| 2 — UFACTORY packages visible | `lab test 2` | no |
| 3 — robot model loads | `lab test 3` *(leave running)* | no |
| 4 — kinematic chain | `lab test 4` → PDF in `./shared/` | no |
| 5 — read a transform | `lab test 5` | no |
| 6 — Gazebo on its own | `lab test 6` | **yes** |
| 7 — Lite 6 in Gazebo under MoveIt | `lab test 7` | **yes** |

Each prints the real `ros2` command before running it — copy that into your
submission if something fails.

**To submit:** path C; pass/fail for 1–7; the two 4×4 matrices from Test 5; one
sentence from Test 7 on something Gazebo showed that RViz did not; and for any
failure, the **first** error message as text.

---

## The Gazebo flag

Gazebo is a physics engine; under software rendering it uses every core you have.
When you are driving the real arm you do not want it near your CPU.

```bash
./lab.sh gazebo off
```

`lab sim`, `lab gz` and tests 6–7 then refuse to start and say why — nothing can
quietly spin up a simulation behind your back. `lab moveit` (RViz + fake
controllers), `lab real` and `lab driver` are unaffected. Effective immediately in
every open terminal; `lab kill` stops a simulation already running.

It is a guard rail, not a throttle: Gazebo only costs CPU while it runs. Stored in
`.env` as `LAB_GAZEBO=0|1`, overridden for the running container by `lab gazebo
on|off`, and `lab gazebo reset` hands control back to `.env`.

---

## Where the windows appear

`DISPLAY_MODE` in `.env`, default `auto`, which on Linux resolves to `x11`:
**RViz and Gazebo open as ordinary windows on your desktop**, rendered by your
own GPU. Nothing to install and nothing to configure — `./lab.sh` mounts the X
socket from `/tmp/.X11-unix`, passes `DISPLAY` through, and copies your session's
auth cookie to a file the container can read.

The cookie matters and is the usual reason a container "cannot open display"
even though the socket is right there. On stock Ubuntu your session cookie is
not in `~/.Xauthority` at all — GDM keeps its own — so `./lab.sh` looks in
`$XAUTHORITY`, `~/.Xauthority` and `/run/user/$(id -u)/gdm/Xauthority`, and
rewrites the entry to match whatever hostname the container presents. Only if it
finds nothing does it fall back to `xhost +local:`, and it says so when it does.

Rendering is hardware by default (`LIBGL_ALWAYS_SOFTWARE=0`). `./lab.sh` passes
`/dev/dri` through, and adds an NVIDIA reservation when it finds the container
toolkit — so Gazebo runs at a sensible speed rather than the software-rasteriser
crawl the handout warns about. `lab doctor` prints the GL renderer actually in
use — your card's name rather than `llvmpipe`, which is the software one.

If any of that is unavailable — a headless box, plain ssh, a driver stack that
will not cooperate — set `DISPLAY_MODE=vnc` in `.env` and `./lab.sh restart`.
That runs a virtual X server inside the container and serves a desktop at
<http://localhost:6080/> (`./lab.sh vnc`), which always works. `auto` falls back
to it on its own when it finds no X socket. `LIBGL_ALWAYS_SOFTWARE=1` forces
Mesa if hardware GL misbehaves: correct, slow, and the handout's Test 6
fallback.

If `lab test 3` opens nothing, `lab doctor` says whether the container can reach
an X server at all.

---

## Where your work lives

| On your machine | In the container | |
|---|---|---|
| `./src/` | `~/dev_ws/src` | your packages — survives everything, including `nuke` |
| `./shared/` | `~/shared` | files in and out (the `view_frames` PDF, notes) |
| — | `~/dev_ws/{build,install,log}` | Docker volume; `nuke` deletes it, `lab build` recreates it |
| — | `/opt/xarm_ws` | xarm_ros2, pre-built into the image |

`./src/` is a bind mount — the same bytes under two names, no copy and no sync.
Edit it with any editor on the host, build and run in the container shell. You run
as your own UID inside, so files come back owned by you, not by root. Build
artefacts stay in a Docker volume rather than under `./src/`, which keeps
`colcon`'s output out of your git status.

**Anything else in the container is disposable.** A `pip install` or `apt install`
disappears when the container is recreated — put it in the `Dockerfile`.

`./lab.sh code` opens VS Code; choose **Reopen in Container** for an editor and
debugger running inside, with ROS sourced. Editing on the host and running
`./lab.sh shell` in a second terminal works just as well.

Workspace layering is `/opt/ros/jazzy` → `/opt/xarm_ws` → `~/dev_ws`, all sourced
by `~/.bashrc` as in Step 7 of the handout. Your overlay shadows the underlay,
which is what `lab overlay-xarm` is for if you ever need to modify `xarm_ros2`.

---

## When it goes wrong

| Symptom | Do this |
|---|---|
| `Docker is not installed (or not on PATH)` | `sudo apt install docker.io docker-compose-v2`. |
| `permission denied ... docker.sock` | You are not in the `docker` group. `sudo usermod -aG docker $USER`, then log out and back in. |
| `the engine is not responding` | The daemon is not running: `sudo systemctl start docker`. |
| Build fails | The **first** error matters, not the last. Scroll up. |
| Build killed / machine freezes | Lower `COLCON_JOBS` in `.env`, then `./lab.sh rebuild`. |
| `no action server on /lite6_traj_controller/...` | You are on `lab driver`, which has no `controller_manager`. Use `lab sim` or `lab real`. |
| Service calls ignored, arm does not move | Two owners. `ros2 topic echo /ufactory/robot_states --once` — mode 1 means `ros2_control` has it. |
| `ros2: command not found` inside | A shell that skipped `~/.bashrc`. Run `bash -l`. |
| RViz/Gazebo open nothing | `lab doctor`, then `DISPLAY_MODE=vnc` + `./lab.sh restart`, and open <http://localhost:6080/>. |
| Gazebo hangs for minutes on first launch | Normal — Fuel models and shaders, cached in a volume, once only. |
| A deleted node still shows in `ros2 pkg executables` | `colcon` never prunes `install/`. `lab clean`, then `lab build`. |
| Everything is confused | `./lab.sh nuke` then `./lab.sh up`. `./src` and `./shared` survive. |
| `xarm_ros2` upstream moved | `./lab.sh rebuild-xarm` — fresh recursive clone and build. |

Container logs are UTC; your clock is probably local. Timestamps will look offset.

---

## Files

```
Dockerfile              Steps 1-8 of the handout, commented per step
docker-compose.yml      volumes, display, ports, the LAB_GAZEBO switch
docker-compose.gpu.yml  /dev/dri passthrough; added by lab.sh when present
docker-compose.nvidia.yml   NVIDIA reservation; added when the toolkit is there
.env.example            settings template (ROS_DOMAIN_ID, Gazebo, display)
lab.sh / lab.ps1        host driver: build / up / shell / gazebo / doctor
                        (also: the X cookie, and which compose files to use)
docker/lab              in-container helper: the seven tests and the launches
docker/entrypoint.sh    sources ROS, sets up the display, prints the banner
docker/ros_setup.sh     Step 7 of the handout, sourced by every shell
docker/start-display    host X / VNC (WSLg on the windows branch)
.devcontainer/          VS Code "Reopen in Container"
src/lite6_control/      worked example ROS 2 Python package
src/lite6_kinematics/   Labwork 1: forward kinematics (template + checker)
src/lite6_ik/           Labwork 2: algebraic inverse kinematics (template + checkers)
src/                    YOUR packages     (= ~/dev_ws/src)
shared/                 files in and out  (= ~/shared)
```
