"""In-process control SDK for the MuJoCo abstraction of the Franzi robot.

The method surface copies the vendor SDK (franzi_sdk ``ArmClient``) so code
written against the real robot drives this simulation unchanged:

    CommandJointPosition / MoveJ / MoveStop / Home / GetMotionStatus
    GetCurJointPos / GetCurJointVel          (per mech unit)
    GripperOpen / GripperClose / GripperSetPosition / GripperReadPosition
    IsReady / SoftStop / AxisCount

Same conventions as the vendor: arm APIs take ``side`` (None = dual,
"left"/1, "right"/2) with 7- or 14-dim joint vectors in radians; feedback
takes a mech unit (LEFTARM/RIGHTARM/WAIST/HEAD/LEFTHAND/RIGHTHAND, vendor
enum values). MoveJ is non-blocking - poll GetMotionStatus, like the RPC.

Simulation-only extensions, clearly outside the vendor surface (the real
chassis speaks a different protocol): CommandBaseVelocity (body-frame
vx/vy/wz through the three-steer-wheel inverse kinematics), GetBasePose,
CommandUnitPosition (waist/head), and step()/spin() - the caller owns time,
nothing runs in a background thread.

Quick start:

    from franzi_sim_sdk import FranziSimClient
    robot = FranziSimClient()
    robot.MoveJ([[0, 0.4, 0, -1.0, 0, 0, 0]], side="left")
    while robot.GetMotionStatus("left"):
        robot.step()
    robot.GripperClose("left")
    robot.CommandBaseVelocity(0.3, 0.0, 0.0)
    robot.spin(2.0)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Sequence

import mujoco

HERE = Path(__file__).resolve().parent

# Vendor mech-unit enum values (franzi_sdk core.types.MechUnitType).
LEFTARM, RIGHTARM, WAIST, HEAD, LEFTHAND, RIGHTHAND = 0, 10, 20, 30, 40, 50

# Joint layout per unit, in vendor order (= the MoveIt controller groups).
UNIT_JOINTS = {
    LEFTARM: [f"left_{j}_joint" for j in
              ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
               "elbow_pitch", "wrist_yaw", "wrist_pitch", "wrist_roll")],
    RIGHTARM: [f"right_{j}_joint" for j in
               ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                "elbow_pitch", "wrist_yaw", "wrist_pitch", "wrist_roll")],
    WAIST: ["calf_pitch_joint", "thigh_pitch_joint",
            "waist_pitch_joint", "waist_yaw_joint"],
    HEAD: ["head_yaw_joint", "head_pitch_joint"],
    LEFTHAND: ["leftfinger1_joint", "leftfinger2_joint"],
    RIGHTHAND: ["rightfinger1_joint", "rightfinger2_joint"],
}
UNIT_NAMES = {
    "left": LEFTARM, "leftarm": LEFTARM, "right": RIGHTARM,
    "rightarm": RIGHTARM, "waist": WAIST, "head": HEAD,
    "left_hand": LEFTHAND, "lefthand": LEFTHAND,
    "right_hand": RIGHTHAND, "righthand": RIGHTHAND,
}
# Vendor ArmSide for the motion APIs: None/0 = dual, 1 = left, 2 = right.
ARM_SIDES = {None: (LEFTARM, RIGHTARM), 0: (LEFTARM, RIGHTARM),
             "dual": (LEFTARM, RIGHTARM), 1: (LEFTARM,), "left": (LEFTARM,),
             2: (RIGHTARM,), "right": (RIGHTARM,)}

FINGER_TRAVEL = 0.0475  # prismatic range of each finger; grip 1.0 = closed
DEFAULT_JOINT_SPEED = 1.0  # rad/s used to time MoveJ segments

OK, ERR_NOT_READY, ERR_BAD_ARG = 0, 1, 2  # SdkStatus-style returns


def _unit(unit):
    if isinstance(unit, str):
        return UNIT_NAMES[unit.lower()]
    if unit in UNIT_JOINTS:
        return unit
    raise KeyError(f"unknown mech unit {unit!r}")


class FranziSimClient:
    """ArmClient-compatible subset, backed by Mujoco/franzi/scene.xml."""

    def __init__(self, xml: Optional[Path] = None, settle: float = 1.0):
        self.model = mujoco.MjModel.from_xml_path(str(xml or HERE / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self._softstop = False
        self._paths = {}  # unit -> list of (target_vector, duration_s)
        self._qadr = {
            unit: [self.model.jnt_qposadr[self.model.joint(j).id] for j in joints]
            for unit, joints in UNIT_JOINTS.items()
        }
        self._vadr = {
            unit: [self.model.jnt_dofadr[self.model.joint(j).id] for j in joints]
            for unit, joints in UNIT_JOINTS.items()
        }
        self._ctrl = {
            unit: [self.model.actuator(j).id for j in joints]
            for unit, joints in UNIT_JOINTS.items()
        }
        # Steer-wheel modules: (steer actuator, wheel actuator, x, y).
        self._modules = []
        for prefix in ("left_front", "right_front", "rear"):
            steer = self.model.actuator(f"{prefix}_steering_joint").id
            wheel = self.model.actuator(f"{prefix}_wheel_joint").id
            x, y, _ = self.model.body(f"{prefix}_steering_Link").pos
            self._modules.append((steer, wheel, float(x), float(y)))
        self._wheel_radius = float(
            self.model.geom("left_front_wheel_Link").size[0]
        )
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.spin(settle)

    # ------------------------------------------------------------ time
    def step(self, n: int = 1):
        """Advance physics; trajectory targets are refreshed each step."""
        for _ in range(n):
            self._advance_paths(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

    def spin(self, seconds: float):
        self.step(max(1, round(seconds / self.model.opt.timestep)))

    def _advance_paths(self, dt):
        done = []
        for unit, path in self._paths.items():
            target, remaining = path[0]
            current = [self.data.ctrl[a] for a in self._ctrl[unit]]
            if remaining <= dt:
                for a, v in zip(self._ctrl[unit], target):
                    self.data.ctrl[a] = v
                path.pop(0)
                if not path:
                    done.append(unit)
            else:
                frac = dt / remaining
                for a, c, v in zip(self._ctrl[unit], current, target):
                    self.data.ctrl[a] = c + (v - c) * frac
                path[0] = (target, remaining - dt)
        for unit in done:
            del self._paths[unit]

    # ------------------------------------------------------------ system
    def IsReady(self) -> bool:
        return not self._softstop

    def AxisCount(self) -> int:
        return len(UNIT_JOINTS[LEFTARM])

    def SoftStop(self, active: bool = True) -> bool:
        self._softstop = active
        if active:
            self._paths.clear()
            for unit in (LEFTARM, RIGHTARM, WAIST, HEAD):
                for a, q in zip(self._ctrl[unit], self._qadr[unit]):
                    self.data.ctrl[a] = self.data.qpos[q]  # hold in place
            self.CommandBaseVelocity(0.0, 0.0, 0.0)
        return True

    # ------------------------------------------------------------ feedback
    def GetCurJointPos(self, unit) -> list:
        return [float(self.data.qpos[a]) for a in self._qadr[_unit(unit)]]

    def GetCurJointVel(self, unit) -> list:
        return [float(self.data.qvel[a]) for a in self._vadr[_unit(unit)]]

    def GetMotionStatus(self, side) -> int:
        """1 while a MoveJ/Home path is still executing, else 0."""
        return int(any(u in self._paths for u in ARM_SIDES[side]))

    # ------------------------------------------------------------ motion
    def _split(self, joints, side):
        units = ARM_SIDES[side]
        n = len(UNIT_JOINTS[LEFTARM])
        if len(joints) != n * len(units):
            raise ValueError(
                f"expected {n * len(units)} joint values, got {len(joints)}"
            )
        return [(u, list(joints[i * n:(i + 1) * n])) for i, u in enumerate(units)]

    def CommandJointPosition(self, joints: Sequence[float], side=None) -> int:
        """Instant position target: 7-dim single arm, 14-dim dual."""
        if self._softstop:
            return ERR_NOT_READY
        for u, vals in self._split(joints, side):
            self._paths.pop(u, None)
            for a, v in zip(self._ctrl[u], vals):
                self.data.ctrl[a] = v
        return OK

    def MoveJ(
        self,
        joint_path: Sequence[Sequence[float]],
        side=None,
        speed: float = DEFAULT_JOINT_SPEED,
    ) -> int:
        """Queue a joint-space path (non-blocking; poll GetMotionStatus)."""
        if self._softstop:
            return ERR_NOT_READY
        per_unit = {}
        for point in joint_path:
            for u, vals in self._split(point, side):
                per_unit.setdefault(u, []).append(vals)
        for u, points in per_unit.items():
            previous = self.GetCurJointPos(u)
            path = []
            for point in points:
                span = max(abs(a - b) for a, b in zip(point, previous))
                path.append((point, max(span / speed, 0.05)))
                previous = point
            self._paths[u] = path
        return OK

    def MoveStop(self, side=None) -> int:
        for u in ARM_SIDES[side]:
            self._paths.pop(u, None)
            for a, q in zip(self._ctrl[u], self._qadr[u]):
                self.data.ctrl[a] = self.data.qpos[q]
        return OK

    def Home(self, side=None) -> int:
        """Blocking return to all-zero, vendor semantics."""
        n = len(ARM_SIDES[side]) * self.AxisCount()
        status = self.MoveJ([[0.0] * n], side)
        if status != OK:
            return status
        while self.GetMotionStatus(side):
            self.step()
        return OK

    # ------------------------------------------------------------ gripper
    def GripperSetPosition(self, side, position: float) -> int:
        """0.0 = fully open ... 1.0 = fully closed."""
        if self._softstop:
            return ERR_NOT_READY
        if not 0.0 <= position <= 1.0:
            return ERR_BAD_ARG
        unit = LEFTHAND if ARM_SIDES[side] == (LEFTARM,) else RIGHTHAND
        f1, f2 = self._ctrl[unit]
        self.data.ctrl[f1] = FINGER_TRAVEL * position
        self.data.ctrl[f2] = -FINGER_TRAVEL * position
        return OK

    def GripperOpen(self, side) -> int:
        return self.GripperSetPosition(side, 0.0)

    def GripperClose(self, side) -> int:
        return self.GripperSetPosition(side, 1.0)

    def GripperReadPosition(self, side) -> float:
        unit = LEFTHAND if ARM_SIDES[side] == (LEFTARM,) else RIGHTHAND
        q1, q2 = (self.data.qpos[a] for a in self._qadr[unit])
        return float((q1 - q2) / (2 * FINGER_TRAVEL))

    # ------------------------------------- simulation-only: base and units
    def CommandUnitPosition(self, unit, positions: Sequence[float]) -> int:
        """Position targets for WAIST or HEAD (not part of the vendor arm
        SDK - the real ones ride other controllers)."""
        if self._softstop:
            return ERR_NOT_READY
        unit = _unit(unit)
        if len(positions) != len(self._ctrl[unit]):
            return ERR_BAD_ARG
        self._paths.pop(unit, None)
        for a, v in zip(self._ctrl[unit], positions):
            self.data.ctrl[a] = v
        return OK

    def CommandBaseVelocity(self, vx: float, vy: float, wz: float) -> int:
        """Body-frame chassis velocity through the steer-wheel inverse
        kinematics. The real chassis has its own interface; in the ROS
        stack this is the /cmd_vel seam."""
        if self._softstop and (vx or vy or wz):
            return ERR_NOT_READY
        for steer, wheel, x, y in self._modules:
            mvx = vx - wz * y
            mvy = vy + wz * x
            speed = math.hypot(mvx, mvy)
            if speed < 1e-6:
                self.data.ctrl[wheel] = 0.0
                continue  # keep the last steering angle
            angle = math.atan2(mvy, mvx)
            if abs(angle) > math.pi / 2:  # fold: roll backwards instead
                angle -= math.copysign(math.pi, angle)
                speed = -speed
            self.data.ctrl[steer] = angle
            self.data.ctrl[wheel] = speed / self._wheel_radius
        return OK

    def GetBasePose(self):
        """(x, y, yaw) of base_link in the world."""
        x, y = self.data.qpos[0], self.data.qpos[1]
        w, _, _, qz = self.data.qpos[3:7]
        return float(x), float(y), 2.0 * math.atan2(qz, w)
