# Robótica MEEC 26/27 — UFACTORY Lite 6

Docker-based ROS 2 Jazzy + MoveIt 2 + Gazebo environment for the UFACTORY Lite 6 arm (course *Robotics 02000537*, 2026/2027). Branch `LW0.2` covers Labwork 0, Part 2.

## Layout

```
Dockerfile            # ROS 2 Jazzy image (MoveIt, Gazebo, xArm Python SDK)
docker-compose.yaml   # Runs the container with GUI support
ros2_ws/src/          # Workspace (xarm_ros2 is a git submodule)
docs/                 # Course PDFs (Labwork 0 parts 1 and 2, final challenge)
```

## Setup

Clone with submodules (the build fails without them):

```bash
git clone --recursive -b LW0.2 https://github.com/errorcodecritical/robotica-meec-26-27.git
cd robotica-meec-26-27
```

Allow GUI windows from Docker (Linux), then build and start:

```bash
xhost +local:docker
docker compose up --build
```

This installs dependencies and runs `colcon build` (about 15–30 minutes the first time). Then open a shell:

```bash
docker compose exec ros bash
```

## Run

```bash
# MoveIt with simulated controllers
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py

# MoveIt with the robot in Gazebo
ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py
```

## Troubleshooting

- **Build fails with missing headers:** run `git submodule update --init --recursive`, then rebuild.
- **Out of memory during build:** run `colcon build --parallel-workers 2` inside the container.
- **Package not found:** run `source /ros2_ws/install/setup.bash`.
- **Gazebo black window:** run `export LIBGL_ALWAYS_SOFTWARE=1`.

## Safety

Before using the real robot, read the safety rules in `docs/Robotics_2627_LabWork0_part1.pdf`. Everything must run in simulation first.
