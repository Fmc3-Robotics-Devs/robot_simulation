"""Load the abstract model, settle it on the floor, exercise the arms.

Run:  python3 smoke_test.py    (needs the mujoco pip package)
"""

from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
model = mujoco.MjModel.from_xml_path(str(HERE / "scene.xml"))
data = mujoco.MjData(model)
print(f"bodies={model.nbody} joints={model.njnt} geoms={model.ngeom} "
      f"actuators={model.nu}")

mujoco.mj_resetDataKeyframe(model, data, 0)
start = data.qpos[2]

# Phase 1: settle under gravity with the servos holding the home pose.
for _ in range(1500):
    mujoco.mj_step(model, data)
settled = data.qpos[2]
assert abs(settled - start) < 0.05, f"base sank: {start:.3f} -> {settled:.3f}"

# Phase 2: command both shoulders forward and close the grippers.
for name, target in (
    ("left_shoulder_pitch_joint", -0.8),
    ("right_shoulder_pitch_joint", -0.8),
    ("left_elbow_pitch_joint", -1.2),
    ("right_elbow_pitch_joint", -1.2),
    ("leftfinger1_joint", 0.03),
    ("leftfinger2_joint", -0.03),
    ("rightfinger1_joint", 0.03),
    ("rightfinger2_joint", -0.03),
):
    data.ctrl[model.actuator(name).id] = target
for _ in range(2000):
    mujoco.mj_step(model, data)

for name, target in (
    ("left_shoulder_pitch_joint", -0.8),
    ("left_elbow_pitch_joint", -1.2),
    ("leftfinger1_joint", 0.03),
):
    joint = model.joint(name)
    actual = data.qpos[model.jnt_qposadr[joint.id]]
    assert abs(actual - target) < 0.08, f"{name}: {actual:.3f} != {target:.3f}"
    print(f"{name}: commanded {target:+.3f}, reached {actual:+.3f}")

# Phase 3: drive forward on the wheels for two seconds.
x0 = data.qpos[0]
for wheel in ("left_front_wheel_joint", "right_front_wheel_joint",
              "rear_wheel_joint"):
    data.ctrl[model.actuator(wheel).id] = 8.0
for _ in range(1000):
    mujoco.mj_step(model, data)
travelled = data.qpos[0] - x0
assert travelled > 0.3, f"base only moved {travelled:.3f} m"
assert data.qpos[2] > 0.03, f"base fell over: z={data.qpos[2]:.3f}"
print(f"drove {travelled:.2f} m in 2 s, base height {data.qpos[2]:.3f} m, "
      f"quat {np.round(data.qpos[3:7], 3)}")
print("smoke test passed")
