"""Python interface to the Franzi MuJoCo model.

One object wraps the model and its data and speaks the robot's vocabulary:
SRDF named postures, gripper gap in metres, a body-frame base twist, TCP poses
and IK, camera images and a 2D scan. Everything here drives the simulation
through its actuators - nothing writes joint positions behind the physics'
back, so what a script sees is what the servos, contacts and friction did.

    from franzi import Franzi
    robot = Franzi()                    # model/scene.xml, "home" keyframe
    robot.set_posture("head", "look_down")
    robot.drive(0.3, 0.0, 0.0)          # m/s, m/s, rad/s in base_link
    robot.step(2.0)                     # seconds of simulated time
    robot.set_gripper("left", 0.0)      # closed

Frames: the MuJoCo world frame is the ROS ``odom`` frame (the floor sits at
``task.yaml``'s ``ground_z``), so poses read here compare directly with the
ROS stack's.
"""

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SCENE = HERE / "model" / "scene.xml"
SRDF = (
    HERE.parent / "Rviz" / "src" / "franzi_moveit_config" / "config"
    / "wheel_robot_26.8.16_3.srdf"
)

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch",
              "wrist_yaw", "wrist_pitch", "wrist_roll")
SWERVE = ("left_front", "right_front", "rear")
# The finger joint origins are 95 mm apart and both travel inwards, so joint
# zero is fully open. Which sign closes differs per side (see the SRDF).
GRIPPER_GAP = 0.095
CLOSING_SIGN = {"left": -1.0, "right": 1.0}  # of finger01


def normalise(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def min_jerk(alpha):
    alpha = min(max(alpha, 0.0), 1.0)
    return alpha**3 * (10 - 15 * alpha + 6 * alpha**2)


def top_down(yaw=0.0):
    """TCP orientation for a vertical grasp: approach (TCP +x) straight down,
    fingers closing (TCP +z) along the horizontal direction ``yaw``."""
    closing = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    approach = np.array([0.0, 0.0, -1.0])
    return np.column_stack([approach, np.cross(closing, approach), closing])


def srdf_states(path=SRDF):
    """{(group, state): {joint: value}} from the MoveIt SRDF."""
    states = {}
    for state in ET.parse(path).getroot().findall("group_state"):
        states[(state.get("group"), state.get("name"))] = {
            j.get("name"): float(j.get("value")) for j in state.findall("joint")
        }
    return states


class Franzi:
    def __init__(self, model_path=SCENE, keyframe="home", srdf=SRDF):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self._states = srdf_states(srdf) if Path(srdf).exists() else {}
        self._renderers = {}
        self._twist = np.zeros(3)
        m = self.model

        self._actuator = {}  # joint name -> actuator id
        for a in range(m.nu):
            joint = m.joint(m.actuator_trnid[a, 0]).name
            self._actuator[joint] = a
        self._base_act = [m.actuator(n).id for n in ("base_x", "base_y", "base_yaw")]
        self._base_qpos = [m.jnt_qposadr[m.joint(n).id] for n in ("base_x", "base_y", "base_yaw")]

        # Swerve modules: steering axis position in base_link, wheel radius
        # from the mesh. Read off the model at zero so nothing is duplicated.
        zero = mujoco.MjData(m)
        mujoco.mj_forward(m, zero)
        self._modules = {}
        for name in SWERVE:
            steer = m.joint(f"{name}_steering_joint").id
            wheel_body = m.body(f"{name}_wheel_Link").id
            geom = next(g for g in range(m.ngeom)
                        if m.geom_bodyid[g] == wheel_body and m.geom_group[g] == 2)
            mesh = m.geom_dataid[geom]
            verts = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            world = verts @ zero.geom_xmat[geom].reshape(3, 3).T + zero.geom_xpos[geom]
            centre = zero.xanchor[m.joint(f"{name}_wheel_joint").id]
            radius = centre[2] - world[:, 2].min()
            self._modules[name] = (zero.xanchor[steer][:2].copy(), radius)
        self.wheel_radius = float(np.mean([r for _, r in self._modules.values()]))

        self.reset(keyframe)

    # --- state -----------------------------------------------------------
    def reset(self, keyframe="home"):
        m, d = self.model, self.data
        if keyframe is None:
            mujoco.mj_resetData(m, d)
        else:
            mujoco.mj_resetDataKeyframe(m, d, m.key(keyframe).id)
        self._twist[:] = 0
        mujoco.mj_forward(m, d)

    @property
    def time(self):
        return self.data.time

    def joint(self, name):
        return float(self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]])

    def joints(self, names):
        return np.array([self.joint(n) for n in names])

    def arm_joints(self, side):
        return tuple(f"{side}_{j}_joint" for j in ARM_JOINTS)

    # --- commands --------------------------------------------------------
    def set_targets(self, targets):
        """Position targets {joint: value} for the joints' servos."""
        for joint, value in targets.items():
            if joint.endswith("finger02_joint"):
                continue  # mirrored by the equality constraint
            self.data.ctrl[self._actuator[joint]] = value

    def targets(self, joints):
        return np.array([self.data.ctrl[self._actuator[j]] for j in joints])

    def set_posture(self, group, state):
        """Command an SRDF named state, e.g. ("head", "look_down")."""
        self.set_targets(self._states[(group, state)])

    def set_gripper(self, side, gap):
        """Command the jaw gap in metres, 0 (closed) to 0.095 (open)."""
        stroke = (GRIPPER_GAP - min(max(gap, 0.0), GRIPPER_GAP)) / 2
        self.data.ctrl[self._actuator[f"{side}_finger01_joint"]] = CLOSING_SIGN[side] * stroke

    def gripper_gap(self, side):
        return GRIPPER_GAP - abs(self.joint(f"{side}_finger01_joint")) - abs(
            self.joint(f"{side}_finger02_joint"))

    def drive(self, vx, vy, wz):
        """Body-frame base twist (m/s, m/s, rad/s), held until changed."""
        self._twist[:] = (vx, vy, wz)

    def base_pose(self):
        return tuple(float(self.data.qpos[i]) for i in self._base_qpos)

    def place_base(self, x, y, yaw):
        """Teleport the base (setup only, not a motion). The base servos
        integrate velocity into a position target held in ``act``, so the
        target moves with it - otherwise they would haul the robot back."""
        m, d = self.model, self.data
        for qadr, act, value in zip(self._base_qpos, self._base_act, (x, y, yaw)):
            d.qpos[qadr] = value
            d.act[m.actuator_actadr[act]] = value
            d.qvel[m.jnt_dofadr[m.actuator_trnid[act, 0]]] = 0
        mujoco.mj_forward(m, d)

    def _apply_twist(self):
        vx, vy, wz = self._twist
        yaw = self.data.qpos[self._base_qpos[2]]
        c, s = math.cos(yaw), math.sin(yaw)
        ctrl = self.data.ctrl
        ctrl[self._base_act[0]] = c * vx - s * vy
        ctrl[self._base_act[1]] = s * vx + c * vy
        ctrl[self._base_act[2]] = wz
        # Anti-windup: the servos integrate the command into a position
        # target (``act``). Blocked by a bench, that target would run away
        # and the base would lunge the moment it came free, so it is kept
        # within a leash of where the base actually is.
        m, d = self.model, self.data
        for qadr, act, leash in zip(self._base_qpos, self._base_act, (0.05, 0.05, 0.1)):
            i = m.actuator_actadr[act]
            d.act[i] = np.clip(d.act[i], d.qpos[qadr] - leash, d.qpos[qadr] + leash)
        # Swerve modules follow the motion: steer along each module's
        # velocity, the short way round (driving backwards is the same
        # motion), and spin the wheel to roll without slip.
        for name, ((px, py), radius) in self._modules.items():
            mvx, mvy = vx - wz * py, vy + wz * px
            speed = math.hypot(mvx, mvy)
            steer_joint = f"{name}_steering_joint"
            steer = ctrl[self._actuator[steer_joint]]
            if speed > 1e-4:
                angle = math.atan2(mvy, mvx)
                if abs(normalise(angle - steer)) > math.pi / 2:
                    angle, speed = normalise(angle + math.pi), -speed
                steer = angle
            ctrl[self._actuator[steer_joint]] = steer
            ctrl[self._actuator[f"{name}_wheel_joint"]] = speed / radius

    # --- time ------------------------------------------------------------
    def step(self, duration=None, callback=None):
        """Advance by ``duration`` seconds (one physics step if None)."""
        steps = 1 if duration is None else max(1, round(duration / self.model.opt.timestep))
        for _ in range(steps):
            self._apply_twist()
            mujoco.mj_step(self.model, self.data)
            if callback is not None:
                callback(self)

    def move_joints(self, targets, duration, callback=None):
        """Min-jerk interpolation of servo targets over ``duration`` seconds."""
        joints = list(targets)
        start = np.array([self.data.ctrl[self._actuator[j]] for j in joints])
        goal = np.array([targets[j] for j in joints])
        t0 = self.time
        while self.time - t0 < duration:
            alpha = min_jerk((self.time - t0) / duration)
            self.set_targets(dict(zip(joints, start + alpha * (goal - start))))
            self.step(callback=callback)
        self.set_targets(targets)

    def drive_to(self, x, y, yaw, speed=0.5, yaw_rate=0.8, tolerance=0.001, callback=None,
                 timeout=30.0):
        """Drive to an odom pose with a proportional twist, then stop."""
        t0 = self.time
        while self.time - t0 < timeout:
            px, py, pyaw = self.base_pose()
            ex, ey, eyaw = x - px, y - py, normalise(yaw - pyaw)
            if math.hypot(ex, ey) < tolerance and abs(eyaw) < tolerance:
                break
            c, s = math.cos(pyaw), math.sin(pyaw)
            bx, by = c * ex + s * ey, -s * ex + c * ey  # into base_link
            gain = 2.0
            v = np.array([bx, by]) * gain
            if np.linalg.norm(v) > speed:
                v *= speed / np.linalg.norm(v)
            w = float(np.clip(eyaw * gain, -yaw_rate, yaw_rate))
            self.drive(v[0], v[1], w)
            self.step(callback=callback)
        self.drive(0, 0, 0)
        return self.base_pose()

    # --- kinematics ------------------------------------------------------
    def site_pose(self, site):
        s = self.data.site(site)
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def tcp_pose(self, side):
        return self.site_pose(f"{side}_tcp")

    def solve_ik(self, side, position, rotation=None, seed=None, iterations=200,
                 restarts=20, tolerance=(5e-4, 5e-3)):
        """Arm joints putting ``{side}_tcp`` at a world pose.

        Damped least squares on a scratch copy of the current state (base,
        body and the other arm stay where they are), clamped to the joint
        limits. The step length is capped, so a near-singular start (the
        hanging arm is one) cannot throw the solution into the limits, and a
        null-space push away from the limits keeps it off them (the wrists'
        +-90 deg roll range is easy to run into). Runs from
        ``seed`` (default: the current joints) and ``restarts`` random
        configurations, and of the solutions found returns the one with the
        most margin to its joint limits. Returns ({joint: value}, success).
        """
        m = self.model
        scratch = mujoco.MjData(m)
        scratch.qpos[:] = self.data.qpos
        joints = self.arm_joints(side)
        jid = [m.joint(j).id for j in joints]
        qadr = np.array([m.jnt_qposadr[j] for j in jid])
        dadr = np.array([m.jnt_dofadr[j] for j in jid])
        lower, upper = m.jnt_range[jid].T
        span = upper - lower
        site = m.site(f"{side}_tcp").id
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        target_quat = np.zeros(4)
        if rotation is not None:
            mujoco.mju_mat2Quat(target_quat, np.asarray(rotation, float).ravel())
        random = np.random.default_rng(0)
        starts = [scratch.qpos[qadr].copy() if seed is None else np.asarray(seed, float)]
        starts += [random.uniform(lower, upper) for _ in range(restarts)]

        def errors():
            error_p = np.asarray(position) - scratch.site_xpos[site]
            error_r = np.zeros(3)
            if rotation is not None:
                current = np.zeros(4)
                mujoco.mju_mat2Quat(current, scratch.site_xmat[site])
                mujoco.mju_subQuat(error_r, target_quat, current)
                # mju_subQuat answers in the current frame; the Jacobian is world.
                error_r = scratch.site_xmat[site].reshape(3, 3) @ error_r
            return error_p, error_r

        solutions = []
        for start in starts:
            scratch.qpos[qadr] = start
            for _ in range(iterations):
                mujoco.mj_kinematics(m, scratch)
                mujoco.mj_comPos(m, scratch)
                error_p, error_r = errors()
                if np.linalg.norm(error_p) < tolerance[0] and np.linalg.norm(error_r) < tolerance[1]:
                    solutions.append(scratch.qpos[qadr].copy())
                    break
                mujoco.mj_jacSite(m, scratch, jacp, jacr, site)
                jac, error = jacp[:, dadr], error_p
                if rotation is not None:
                    jac, error = np.vstack([jac, jacr[:, dadr]]), np.concatenate([error_p, error_r])
                # Damped inverse for the task step; the exact pseudo-inverse
                # for the null-space projector, which the damped one is not -
                # it would leak the mid-range pull into the task error.
                damped = jac.T @ np.linalg.inv(jac @ jac.T + 1e-4 * np.eye(len(error)))
                null = np.eye(len(dadr)) - np.linalg.pinv(jac, rcond=1e-3) @ jac
                q = scratch.qpos[qadr]
                # Joint-limit avoidance: descend sum 1/(x(1-x)) over the
                # normalised positions x - flat mid-range, steep near a limit.
                x = np.clip((q - lower) / span, 1e-3, 1 - 1e-3)
                away = (1 - 2 * x) / (x * (1 - x)) ** 2 / span
                dq = damped @ error + null @ (1e-4 * away)
                longest = np.abs(dq).max()
                if longest > 0.2:
                    dq *= 0.2 / longest
                scratch.qpos[qadr] = np.clip(q + dq, lower, upper)
        if not solutions:
            return dict(zip(joints, starts[0])), False
        return dict(zip(joints, max(solutions, key=self._margin_fn(jid)))), True

    def _margin_fn(self, jid):
        lower, upper = self.model.jnt_range[jid].T
        return lambda q: float(np.min(np.minimum(q - lower, upper - q) / (upper - lower)))

    def limit_margin(self, solution):
        """Smallest distance of a {joint: value} solution to its joint
        limits, as a fraction of each joint's range (0 = on a limit)."""
        jid = [self.model.joint(j).id for j in solution]
        return self._margin_fn(jid)(np.array(list(solution.values())))

    def move_tcp(self, side, position, rotation, duration, waypoints=20, callback=None):
        """Straight-line TCP motion through IK waypoints.

        Each waypoint is solved from the previous one with no random
        restarts, and a waypoint that would jump the arm onto another IK
        branch is refused: swinging through a branch change mid-motion is
        what flings a held part across the room.
        """
        start, _ = self.tcp_pose(side)
        joints = self.arm_joints(side)
        seed = self.joints(joints)
        path = []
        for k in range(1, waypoints + 1):
            point = start + (np.asarray(position) - start) * min_jerk(k / waypoints)
            solution, ok = self.solve_ik(side, point, rotation, seed=seed, restarts=0)
            q = np.array([solution[j] for j in joints])
            if not ok or np.abs(q - seed).max() > 0.3:
                raise RuntimeError(f"{side} arm: no continuous IK path through {np.round(point, 3)}")
            seed = q
            path.append(solution)
        for solution in path:
            self.move_joints(solution, duration / waypoints, callback=callback)

    # --- sensors ---------------------------------------------------------
    def render(self, camera, depth=False, size=None):
        """RGB (H, W, 3) uint8 or depth (H, W) metres from a named camera."""
        m = self.model
        cam = m.camera(camera).id
        width, height = size or m.cam_resolution[cam]
        if not width:
            width, height = 640, 480
        key = (int(width), int(height))
        renderer = self._renderers.get(key)
        if renderer is None:
            renderer = self._renderers[key] = mujoco.Renderer(m, int(height), int(width))
        if depth:
            renderer.enable_depth_rendering()
        else:
            renderer.disable_depth_rendering()
        renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    def scan(self, site="2Dlidar", fov=math.radians(270), resolution=math.radians(0.33),
             max_range=25.0):
        """2D scan in the lidar's xy plane, angles from -fov/2 about site +x.

        The SICK TIM781's numbers. Only the world is visible (geom groups 0
        and 1): the robot's own geoms are filtered, as a real driver's
        self-mask would. Returns (angles, ranges), ranges inf beyond max.
        """
        m, d = self.model, self.data
        s = m.site(site).id
        origin = d.site_xpos[s]
        rot = d.site_xmat[s].reshape(3, 3)
        angles = np.arange(-fov / 2, fov / 2 + 1e-9, resolution)
        directions = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], 1) @ rot.T
        ranges = np.zeros(len(angles))
        geomid = np.zeros(len(angles), dtype=np.int32)
        mujoco.mj_multiRay(m, d, origin, directions.ravel(), np.array([1, 1, 0, 0, 0, 0], np.uint8),
                           1, -1, geomid, ranges, None, len(angles), max_range)
        ranges[(geomid < 0) | (ranges > max_range)] = np.inf
        return angles, ranges

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
