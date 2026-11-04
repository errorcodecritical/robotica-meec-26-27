# Lab 0 standalone Python scripts

These scripts implement the xArm Python SDK exercises from Lab Work 0, Part 1.
They run directly with Python and are separate from the ROS 2 packages in
`src/`.

## Files

- `safe_session.py` connects to the configured Lite 6, clears warnings and
  errors, configures reduced mode, enables motion, and selects position mode.
  It also exports `safe_session()` for use by other Python scripts.
- `log_joints.py` calls `safe_session()` in joint-teaching mode, samples the
  six joint angles for 30 seconds, and saves a CSV log and PNG plot.

The logger writes angles in degrees, with elapsed time in seconds. Its output
directory defaults to `logs/` in the current working directory. The CSV
contains the sample timestamps and `joint1_deg` through `joint6_deg`; the
terminal reports the achieved average sample rate.

## Setup

Use Python 3.10 or later. From the repository root, create and activate a
virtual environment and install the dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Windows PowerShell, activate with:

```powershell
.\.venv\Scripts\Activate.ps1
```

Before connecting, configure `config/robot.yaml`:

```yaml
robot_ip: 192.168.1.181
safety:
  reduced_mode: true
  max_joint_speed_deg_s: 54
  max_tcp_speed_mm_s: 150
  tcp_boundary_mm: [x_max, x_min, y_max, y_min, z_max, z_min]
```

Replace the boundary placeholders with the six **bench-approved** TCP boundary
values in millimetres. Each maximum must exceed its corresponding minimum. The
provided config leaves this boundary unset intentionally; both scripts refuse
to enable the arm until it is configured. Do not guess the workspace boundary.

## Run

Run commands from the repository root:

```bash
python scripts/safe_session.py
python scripts/log_joints.py
```

Both scripts accept `--config PATH` to use another YAML config. The logger also
accepts `--duration SECONDS` and `--output-dir PATH`, for example:

```bash
python scripts/log_joints.py --duration 30 --output-dir logs
```

To reuse the session setup in another standalone script:

```python
from pathlib import Path
from safe_session import safe_session

arm = safe_session(Path("config/robot.yaml"))
try:
    code, angles = arm.get_servo_angle()
    if code != 0:
        raise RuntimeError(f"get_servo_angle failed: {code}")
    print(angles)
finally:
    arm.motion_enable(enable=False)
    arm.disconnect()
```

`safe_session.py` is a connection/setup check, not a motion test. On exit it
disables motion and disconnects. `log_joints.py` uses joint-teaching mode for
supervised hand-guiding, then disables motion and disconnects when it finishes
or is interrupted.