#!/usr/bin/env python3
"""Check the SRDF still matches the URDF after a re-export.

`franzi_description` owns the URDF; this package owns the SRDF on top of it
(planning groups, named states, the collision matrix). A re-export can rename a
link or a joint and leave the SRDF pointing at a name that no longer exists, or
shift geometry so a named state now self-collides. This builds the actual MoveIt
`RobotModel` from the installed xacro + SRDF and checks both.

Run from the Rviz/ workspace root, with the workspace sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/franzi_moveit_config/scripts/check_moveit_config.py
"""

import subprocess
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

from moveit.core.collision_detection import CollisionRequest, CollisionResult
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

CONFIG = Path(__file__).resolve().parent.parent / "config"
XACRO = CONFIG / "wheel_robot_26.8.16_3.urdf.xacro"
SRDF = CONFIG / "wheel_robot_26.8.16_3.srdf"


def main():
    failures = []

    def check(condition, message):
        print(f"  {'ok  ' if condition else 'FAIL'}  {message}")
        if not condition:
            failures.append(message)

    urdf = subprocess.check_output(["xacro", str(XACRO)], text=True)
    srdf = SRDF.read_text()

    tmp = Path(tempfile.mkdtemp())
    (tmp / "robot.urdf").write_text(urdf)
    (tmp / "robot.srdf").write_text(srdf)
    model = RobotModel(str(tmp / "robot.urdf"), str(tmp / "robot.srdf"))
    scene = PlanningScene(model)
    state = RobotState(model)

    print(f"{SRDF.name}: {len(model.joint_model_group_names)} planning groups")

    # Every joint and link the SRDF names must exist in the URDF.
    srdf_root = ElementTree.fromstring(srdf)
    urdf_joints = {j.get("name") for j in ElementTree.fromstring(urdf).iter("joint")}
    urdf_links = {l.get("name") for l in ElementTree.fromstring(urdf).iter("link")}

    print("\nSRDF names resolve")
    named_joints = {j.get("name") for j in srdf_root.iter("joint")}
    check(
        named_joints <= urdf_joints,
        f"every SRDF joint exists in the URDF (missing: {sorted(named_joints - urdf_joints)})",
    )
    named_links = {
        e.get(attr)
        for e in srdf_root.iter()
        for attr in ("link", "link1", "link2", "base_link", "tip_link", "parent_link")
        if e.get(attr)
    }
    check(
        named_links <= urdf_links,
        f"every SRDF link exists in the URDF (missing: {sorted(named_links - urdf_links)})",
    )

    def self_collides(rs):
        result = CollisionResult()
        scene.check_self_collision(CollisionRequest(), result, rs)
        return result.collision

    print("\nnamed states are self-collision-free")
    state.set_to_default_values()
    state.update()
    check(not self_collides(state), "the all-zero pose")

    for group_state in srdf_root.findall("group_state"):
        positions = {
            j.get("name"): float(j.get("value")) for j in group_state.findall("joint")
        }
        state.set_to_default_values()
        state.update()
        state.joint_positions = positions
        state.update()
        name = f"{group_state.get('group')}/{group_state.get('name')}"
        check(not self_collides(state), name)

    print()
    if failures:
        print(f"{len(failures)} check(s) failed")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
