# M2 — Drone Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Một quadrotor có physics bay tự động tới scan pose của một BIN bất kỳ (từ `bin_map.yaml`), hover ổn định để camera quét, rồi quay về home — điều khiển bằng cascaded PID, định vị qua interface `LocalizationProvider` (bản POC dùng oracle pose từ sim).

**Architecture:** Phần toán điều khiển (PID, motor-mixing, waypoint planning) là hàm thuần Python, test bằng pytest không cần Isaac. Phần ghép sim (rigid body + áp lực + step world) là adapter mỏng, verify bằng script chạy thật trong Isaac Sim 6.0 binary. Localization đặt sau interface để sau gắn VIO không phải viết lại nav.

**Tech Stack:** Isaac Sim 6.0 binary (`~/isaacsim`, chạy qua `scripts/run_isaac.sh`), `isaacsim.core.api.World`, RigidPrim physics, numpy, pytest. Chạy test thuần Python qua `conda run -n isaac6 python -m pytest`.

---

## Bối cảnh (đã có từ M1)

- `sim/config.py`: HOME_POSE={"position":[-2,-2,1],"yaw_deg":0}, SCAN_STANDOFF=1.5, RACK_ORIGIN, COLUMN_SPACING=1.2, LEVEL_HEIGHT=0.8.
- `sim/bin_map.py`: `load_bin_map()` → 18 BIN; mỗi BIN có `scan_pose.position` (xyz) + `yaw_deg`.
- `sim/scene_builder.py`: dựng scene + `spawn_drone(stage, position)` tạo `/World/Drone/Body` (cube) + `/World/Drone/Camera`.
- Stage up-axis = Z. Đơn vị mét.
- Cách chạy Isaac: `scripts/run_isaac.sh <script.py>` (set EULA, libzbar, PYTHONPATH=repo root, active_gpu=0).

## File Structure

- `drone/__init__.py`
- `drone/localization.py` — `LocalizationProvider` (interface) + `OracleLocalization` (đọc world pose của prim từ sim). Thuần logic + adapter.
- `drone/controller.py` — **thuần Python**: cascaded PID (position→attitude), `motor_mixing()`, `Pid` class. Testable.
- `drone/waypoints.py` — **thuần Python**: `plan_waypoints(bin_id)` → list waypoint (home→approach→scan→home) từ bin_map + safety offset. Testable.
- `drone/quadrotor.py` — adapter Isaac: tạo rigid-body drone (4 rotor positions), `apply_motor_thrusts()`, đọc state. Verify bằng script.
- `drone/flight.py` — state machine `IDLE/FLYING/SCANNING/RETURNING`, vòng điều khiển: mỗi step đọc localization → controller → motor thrust → apply → step world.
- `scripts/fly_to_bin.py` — entrypoint: `fly_to_bin.py <BIN_ID>`; bay, in sai số hover.
- `tests/test_controller.py`, `tests/test_waypoints.py` — pytest thuần Python.

---

## Task 1: Waypoint planner (thuần Python, TDD)

**Files:**
- Create: `drone/__init__.py` (rỗng)
- Create: `drone/waypoints.py`
- Test: `tests/test_waypoints.py`

- [ ] **Step 1: Viết failing test**

```python
# tests/test_waypoints.py
from drone.waypoints import plan_waypoints, Waypoint

def test_plan_has_home_scan_home():
    wps = plan_waypoints("B3")
    assert len(wps) >= 4
    assert wps[0].label == "home"
    assert wps[-1].label == "home"
    assert any(w.label == "scan" for w in wps)

def test_scan_waypoint_matches_bin_map():
    from sim.bin_map import load_bin_map
    wps = plan_waypoints("A1")
    scan = next(w for w in wps if w.label == "scan")
    expected = load_bin_map()["A1"]["scan_pose"]["position"]
    assert scan.position == tuple(expected)

def test_approach_is_further_back_than_scan():
    # approach waypoint stands off further in -Y than the scan pose (safety)
    wps = plan_waypoints("A1")
    approach = next(w for w in wps if w.label == "approach")
    scan = next(w for w in wps if w.label == "scan")
    assert approach.position[1] < scan.position[1]

def test_unknown_bin_raises():
    import pytest
    with pytest.raises(KeyError):
        plan_waypoints("Z9")
```

- [ ] **Step 2: Run test → FAIL**

Run: `conda run -n isaac6 python -m pytest tests/test_waypoints.py -v`
Expected: FAIL `ModuleNotFoundError: No module named 'drone.waypoints'`.

- [ ] **Step 3: Implement `drone/waypoints.py`**

```python
from dataclasses import dataclass
from sim.bin_map import load_bin_map
from sim.config import HOME_POSE

APPROACH_EXTRA_STANDOFF = 0.8  # m further back than scan pose for a safe approach

@dataclass(frozen=True)
class Waypoint:
    position: tuple   # (x, y, z) world meters
    yaw_deg: float
    label: str        # "home" | "approach" | "scan"

def plan_waypoints(bin_id):
    bins = load_bin_map()
    if bin_id not in bins:
        raise KeyError(f"unknown bin {bin_id}")
    scan = bins[bin_id]["scan_pose"]
    sx, sy, sz = scan["position"]
    yaw = scan["yaw_deg"]
    home = Waypoint(tuple(HOME_POSE["position"]), HOME_POSE["yaw_deg"], "home")
    approach = Waypoint((sx, sy - APPROACH_EXTRA_STANDOFF, sz), yaw, "approach")
    scan_wp = Waypoint((sx, sy, sz), yaw, "scan")
    return [home, approach, scan_wp, approach, home]
```

- [ ] **Step 4: Run test → PASS**

Run: `conda run -n isaac6 python -m pytest tests/test_waypoints.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add drone/__init__.py drone/waypoints.py tests/test_waypoints.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M2): waypoint planner home->approach->scan->home

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 2: PID controller + motor mixing (thuần Python, TDD)

**Files:**
- Create: `drone/controller.py`
- Test: `tests/test_controller.py`

- [ ] **Step 1: Viết failing test**

```python
# tests/test_controller.py
import numpy as np
from drone.controller import Pid, motor_mixing, position_to_thrust

def test_pid_drives_error_to_zero_sign():
    pid = Pid(kp=1.0, ki=0.0, kd=0.0)
    # positive error -> positive output
    assert pid.step(error=2.0, dt=0.01) > 0
    pid.reset()
    assert pid.step(error=-2.0, dt=0.01) < 0

def test_pid_derivative_opposes_change():
    pid = Pid(kp=0.0, ki=0.0, kd=1.0)
    pid.step(error=0.0, dt=0.1)
    out = pid.step(error=1.0, dt=0.1)  # error rising -> derivative positive
    assert out > 0

def test_motor_mixing_pure_thrust_is_equal():
    # zero torque -> all 4 rotors equal, sum == total thrust
    m = motor_mixing(total_thrust=4.0, tx=0.0, ty=0.0, tz=0.0)
    assert len(m) == 4
    assert all(abs(v - 1.0) < 1e-6 for v in m)
    assert abs(sum(m) - 4.0) < 1e-6

def test_motor_mixing_roll_torque_differs_left_right():
    m = motor_mixing(total_thrust=4.0, tx=1.0, ty=0.0, tz=0.0)
    # roll torque should make opposite rotors differ
    assert not all(abs(v - m[0]) < 1e-6 for v in m)
    assert all(v >= 0 for v in m)  # thrusts clamped non-negative

def test_position_to_thrust_hover_points_up():
    # at rest under target directly above -> desired thrust vector has +Z dominant
    f = position_to_thrust(pos=np.array([0,0,0.0]), vel=np.zeros(3),
                           target=np.array([0,0,1.0]), mass=1.0, g=9.81)
    assert f[2] > f[0] and f[2] > f[1]
```

- [ ] **Step 2: Run test → FAIL**

Run: `conda run -n isaac6 python -m pytest tests/test_controller.py -v`
Expected: FAIL (module missing).

- [ ] **Step 3: Implement `drone/controller.py`**

```python
import numpy as np

class Pid:
    def __init__(self, kp, ki, kd, i_limit=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.reset()
    def reset(self):
        self._i = 0.0
        self._prev = None
    def step(self, error, dt):
        self._i = float(np.clip(self._i + error * dt, -self.i_limit, self.i_limit))
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        return self.kp * error + self.ki * self._i + self.kd * d

def position_to_thrust(pos, vel, target, mass, g, kp=6.0, kd=4.0):
    """PD on position -> desired world thrust vector (N). Adds gravity feedforward."""
    acc_des = kp * (target - pos) - kd * vel
    acc_des = acc_des + np.array([0.0, 0.0, g])  # counter gravity
    return mass * acc_des

# rotor layout (X config), unit arm; columns: [FL, FR, RL, RR]
# thrust = total/4 + roll/pitch/yaw mixing
def motor_mixing(total_thrust, tx, ty, tz):
    base = total_thrust / 4.0
    # signs per rotor for roll(tx), pitch(ty), yaw(tz)
    mix = [
        base + (-tx + ty + tz) / 4.0,  # FL
        base + (+tx + ty - tz) / 4.0,  # FR
        base + (-tx - ty - tz) / 4.0,  # RL
        base + (+tx - ty + tz) / 4.0,  # RR
    ]
    return [max(0.0, v) for v in mix]
```

- [ ] **Step 4: Run test → PASS**

Run: `conda run -n isaac6 python -m pytest tests/test_controller.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add drone/controller.py tests/test_controller.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M2): PID controller + X-config motor mixing (pure, tested)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 3: Localization interface + oracle (Isaac adapter)

**Files:**
- Create: `drone/localization.py`

- [ ] **Step 1: Implement `drone/localization.py`**

```python
"""Pose source for navigation. POC uses OracleLocalization (ground-truth from sim).
Interface kept minimal so a VIO implementation can replace it without touching nav."""
import numpy as np

class LocalizationProvider:
    def get_pose(self):
        """Return (position np.array[3], yaw_rad float)."""
        raise NotImplementedError
    def get_velocity(self):
        """Return linear velocity np.array[3] (world)."""
        raise NotImplementedError

class OracleLocalization(LocalizationProvider):
    """Reads true pose/velocity of a RigidPrim from the running sim."""
    def __init__(self, rigid_prim):
        self._rb = rigid_prim   # isaacsim RigidPrim wrapper (see drone/quadrotor.py)
    def get_pose(self):
        pos, quat = self._rb.get_world_pose()
        pos = np.asarray(pos, dtype=float).reshape(3)
        # yaw from quaternion (w,x,y,z)
        w, x, y, z = [float(v) for v in np.asarray(quat).reshape(4)]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pos, yaw
    def get_velocity(self):
        v = self._rb.get_linear_velocity()
        return np.asarray(v, dtype=float).reshape(3)
```

- [ ] **Step 2: Commit (verified together with Task 4/5 in sim)**

```bash
git add drone/localization.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M2): LocalizationProvider interface + OracleLocalization

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 4: Quadrotor rigid body adapter (Isaac, verify script)

**Files:**
- Create: `drone/quadrotor.py`
- Create: `scripts/verify_quadrotor.py`

- [ ] **Step 1: Implement `drone/quadrotor.py`**

```python
"""Create a physics rigid-body drone and apply per-rotor thrust forces.

NOTE: exact RigidPrim / force-application API must be confirmed against Isaac Sim
6.0 (see verify script). Target API used here:
  - isaacsim.core.api.World to own the physics scene
  - a RigidPrim wrapping /World/Drone/Body with mass set
  - apply_forces at rotor offset positions each physics step (genuine dynamics)
If apply_forces_at_pos is unavailable, fall back to applying a net force+torque at
COM computed from the same per-rotor thrusts; document the choice in INSTALL_NOTES.
"""
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf

ROTOR_ARM = 0.12  # m, half-spacing of rotors in body frame (X config)
# rotor offsets in body frame: FL, FR, RL, RR
ROTOR_OFFSETS = np.array([
    [-ROTOR_ARM,  ROTOR_ARM, 0.0],
    [ ROTOR_ARM,  ROTOR_ARM, 0.0],
    [-ROTOR_ARM, -ROTOR_ARM, 0.0],
    [ ROTOR_ARM, -ROTOR_ARM, 0.0],
])
DRONE_MASS = 1.0  # kg

def add_physics(stage, body_path="/World/Drone/Body"):
    """Make the drone body a dynamic rigid body with mass."""
    prim = stage.GetPrimAtPath(body_path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    massapi = UsdPhysics.MassAPI.Apply(prim)
    massapi.CreateMassAttr(DRONE_MASS)
    return body_path
```

- [ ] **Step 2: Implement `scripts/verify_quadrotor.py` (hover test)**

```python
"""Verify the drone is a physics body and can hover under PID for a few seconds."""
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})

import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from sim.warehouse import build_warehouse
from sim.drone_asset import spawn_drone
from drone.quadrotor import add_physics, ROTOR_OFFSETS, DRONE_MASS
from drone.controller import Pid, position_to_thrust, motor_mixing
from drone.localization import OracleLocalization

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
build_warehouse(stage)
start = (0.0, 0.0, 1.0)
spawn_drone(stage, start)
add_physics(stage)
rb = RigidPrim("/World/Drone/Body")
world.reset()

loc = OracleLocalization(rb)
target = np.array([0.0, 0.0, 1.5])   # hover 0.5 m above start
zpid = Pid(kp=8.0, ki=1.0, kd=5.0)
dt = 1.0 / 60.0
for i in range(600):  # 10 s
    pos, yaw = loc.get_pose()
    vel = loc.get_velocity()
    f = position_to_thrust(pos, vel, target, DRONE_MASS, g=9.81)
    total = max(0.0, float(f[2]))
    motors = motor_mixing(total, tx=0.0, ty=0.0, tz=0.0)
    # apply equal upward forces at each rotor (genuine multi-point thrust)
    for off, thr in zip(ROTOR_OFFSETS, motors):
        rb.apply_forces(np.array([[0, 0, thr]]), positions=np.array([pos + off]))
    world.step(render=False)

pos, _ = loc.get_pose()
err = float(np.linalg.norm(pos - target))
print(f"HOVER_OK final_pos={pos.round(3)} err={err:.3f}")
assert err < 0.3, f"hover error too large: {err}"
app.close()
```

- [ ] **Step 3: Run & verify (adapt API to real errors)**

Run: `scripts/run_isaac.sh scripts/verify_quadrotor.py 2>&1 | tail -20`
Expected: `HOVER_OK final_pos=[...] err=<0.3`.
IF the RigidPrim / apply_forces signature differs in Isaac Sim 6.0: read the error, fix the call (try `apply_forces_and_torques_at_pos`, or `rb.apply_forces(forces, positions, is_global=True)`, or set effort via articulation), tune PID gains for stability, and append findings under "## M2 physics API" in docs/superpowers/INSTALL_NOTES.md. Re-run until HOVER_OK with err<0.3.

- [ ] **Step 4: Commit**

```bash
git add drone/quadrotor.py scripts/verify_quadrotor.py docs/superpowers/INSTALL_NOTES.md && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M2): physics quadrotor body + verified PID hover (err<0.3m)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 5: Flight state machine + fly-to-bin entrypoint (Isaac, verify script)

**Files:**
- Create: `drone/flight.py`
- Create: `scripts/fly_to_bin.py`

- [ ] **Step 1: Implement `drone/flight.py`**

```python
"""State machine that flies the drone through the planned waypoints for a BIN."""
import numpy as np
from drone.controller import Pid, position_to_thrust, motor_mixing
from drone.quadrotor import ROTOR_OFFSETS, DRONE_MASS
from drone.waypoints import plan_waypoints

STATES = ("IDLE", "FLYING", "SCANNING", "RETURNING")
ARRIVE_TOL = 0.15   # m, considered "arrived" at a waypoint
SCAN_HOLD_STEPS = 120  # ~2 s hover at scan pose

class FlightController:
    def __init__(self, localization, apply_motor_fn, dt=1.0/60.0):
        self.loc = localization
        self.apply_motor = apply_motor_fn   # fn(motors:list[4], pos:np.array)
        self.dt = dt
        self.state = "IDLE"
        self._zpid = Pid(8.0, 1.0, 5.0)

    def _step_to(self, target):
        pos, yaw = self.loc.get_pose()
        vel = self.loc.get_velocity()
        f = position_to_thrust(pos, vel, np.asarray(target), DRONE_MASS, g=9.81)
        total = max(0.0, float(f[2]))
        motors = motor_mixing(total, 0.0, 0.0, 0.0)
        self.apply_motor(motors, pos)
        return float(np.linalg.norm(pos - np.asarray(target)))

    def run(self, bin_id, world_step, max_steps=4000):
        wps = plan_waypoints(bin_id)
        idx = 0
        scan_hold = 0
        self.state = "FLYING"
        for _ in range(max_steps):
            wp = wps[idx]
            err = self._step_to(wp.position)
            world_step()
            if wp.label == "scan":
                self.state = "SCANNING"
                if err < ARRIVE_TOL:
                    scan_hold += 1
                    if scan_hold >= SCAN_HOLD_STEPS:
                        idx += 1; scan_hold = 0
            elif err < ARRIVE_TOL:
                idx += 1
                if idx >= len(wps):
                    self.state = "IDLE"
                    return True
                self.state = "RETURNING" if wps[idx].label == "home" else "FLYING"
        return False
```

- [ ] **Step 2: Implement `scripts/fly_to_bin.py`**

```python
import sys
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})

import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from sim.warehouse import build_warehouse
from sim.rack import build_rack
from sim.drone_asset import spawn_drone
from sim.config import HOME_POSE
from drone.quadrotor import add_physics, ROTOR_OFFSETS
from drone.localization import OracleLocalization
from drone.flight import FlightController
from drone.waypoints import plan_waypoints

bin_id = sys.argv[1] if len(sys.argv) > 1 else "B3"
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
build_warehouse(stage); build_rack(stage)
spawn_drone(stage, tuple(HOME_POSE["position"]))
add_physics(stage)
rb = RigidPrim("/World/Drone/Body")
world.reset()

def apply_motor(motors, pos):
    for off, thr in zip(ROTOR_OFFSETS, motors):
        rb.apply_forces(np.array([[0, 0, thr]]), positions=np.array([pos + off]))

fc = FlightController(OracleLocalization(rb), apply_motor)
ok = fc.run(bin_id, lambda: world.step(render=False))
scan = next(w for w in plan_waypoints(bin_id) if w.label == "scan")
pos, _ = OracleLocalization(rb).get_pose()
err = float(np.linalg.norm(pos - np.array(scan.position)))
print(f"FLY_DONE bin={bin_id} reached_home={ok} final_state={fc.state}")
app.close()
```

- [ ] **Step 3: Run & verify**

Run: `scripts/run_isaac.sh scripts/fly_to_bin.py B3 2>&1 | tail -15`
Expected: `FLY_DONE bin=B3 reached_home=True final_state=IDLE` (drone visited scan pose then returned home). Tune ARRIVE_TOL / PID / max_steps if it stalls. If hover at scan pose is unstable, reduce gains.

- [ ] **Step 4: Commit**

```bash
git add drone/flight.py scripts/fly_to_bin.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M2): flight state machine + fly_to_bin entrypoint

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 6: M2 README + full test pass

**Files:**
- Create: `drone/README.md`

- [ ] **Step 1: Write `drone/README.md`** documenting: states, how to run `scripts/run_isaac.sh scripts/fly_to_bin.py <BIN>`, the LocalizationProvider/VIO extension point, controller tuning, and that pose is oracle (POC).
- [ ] **Step 2: Run all tests** `conda run -n isaac6 python -m pytest tests/ -v` → all pass.
- [ ] **Step 3: Commit** `git add drone/README.md && git commit -m "docs(M2): drone navigation README"`.

---

## Self-Review

**Spec coverage (M2 trong spec mục 5):**
- Quadrotor có physics (4 rotor + controller) → Task 2 (mixing) + Task 4 (rigid body, apply 4 forces) ✓
- LocalizationProvider interface + Oracle → Task 3 ✓
- Waypoint planner home→approach→scan→home + safety offset → Task 1 ✓
- State machine IDLE/FLYING/SCANNING/RETURNING → Task 5 ✓
- Kiểm thử: bay tới B3, hover ổn định, sai số < ngưỡng → Task 4 (hover err<0.3) + Task 5 (fly_to_bin) ✓
- Publish state qua WebSocket: **không thuộc M2** — sẽ do M4 backend đọc `FlightController.state` khi tích hợp (ghi rõ, không chặn M2).

**Placeholder scan:** không có TBD; các điểm API Isaac chưa chắc (RigidPrim.apply_forces) đều có hành động cụ thể + nơi ghi chú. ✓

**Type consistency:** `plan_waypoints`/`Waypoint(.position,.yaw_deg,.label)`, `Pid(.step,.reset)`, `position_to_thrust`, `motor_mixing` (4 phần tử), `OracleLocalization(.get_pose,.get_velocity)`, `ROTOR_OFFSETS`, `DRONE_MASS`, `add_physics`, `FlightController(.run,.state)` — nhất quán giữa các task. ✓

**Rủi ro:** API áp lực rigid body của Isaac 6.0 là điểm bất định lớn nhất — Task 4 verify sớm và có fallback (net force+torque tại COM) + ghi INSTALL_NOTES. Ổn định bay phụ thuộc tuning PID — đã có ngưỡng test rõ.
