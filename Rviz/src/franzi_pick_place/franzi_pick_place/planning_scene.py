"""Planning-scene plumbing.

Updates go out as diffs on ``/planning_scene``, not through the
``/apply_planning_scene`` service. That matters: there are two planning scene
monitors in this system - move_group's, which feeds RViz, and the one inside
the task node's MoveItCpp, which is what actually plans the arm - and both
subscribe to ``/planning_scene``. Using the service only reaches move_group, so
the planner ends up working against an empty world and happily sweeps the arm
through a bench.

Publishing is fire-and-forget, so every change is followed by a wait until the
local monitor - the one that plans - reports it.
"""

import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from .geometry import make_pose


def box(size):
    """Return a SolidPrimitive box from an (x, y, z) size triple."""
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    return primitive


class CollisionObjectBuilder:
    """Accumulate box primitives into a single named CollisionObject."""

    def __init__(self, object_id, frame_id):
        self._object = CollisionObject()
        self._object.id = object_id
        self._object.header.frame_id = frame_id
        self._object.pose = make_pose((0.0, 0.0, 0.0))
        self._object.operation = CollisionObject.ADD

    def set_pose(self, pose):
        """Anchor the object; primitive poses are then relative to it."""
        self._object.pose = pose
        return self

    def add_box(self, size, position, orientation=None):
        self._object.primitives.append(box(size))
        self._object.primitive_poses.append(make_pose(position, orientation))
        return self

    def build(self):
        return self._object


class PlanningSceneClient:
    """Thin synchronous wrapper around the move_group planning-scene services."""

    def __init__(self, node, monitor=None, settle_timeout=5.0):
        self._node = node
        self._logger = node.get_logger()
        self._monitor = monitor
        self._settle_timeout = settle_timeout
        self._diff = node.create_publisher(PlanningScene, "planning_scene", 10)
        self._get = node.create_client(GetPlanningScene, "get_planning_scene")

    def wait_for_services(self, timeout=30.0):
        if not self._get.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{self._get.srv_name} did not come up within {timeout}s")
        # Both monitors have to be listening before a diff is worth sending;
        # publishing into the void is silent and the planner pays for it later.
        deadline = time.monotonic() + timeout
        while self._diff.get_subscription_count() < 2:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "only %d planning scene monitor(s) subscribed to /planning_scene; "
                    "expected move_group and the task node"
                    % self._diff.get_subscription_count()
                )
            time.sleep(0.1)

    def _local_scene(self):
        with self._monitor.read_only() as scene:
            return scene.planning_scene_message

    def _settle(self, predicate, what):
        """Wait until the planning monitor reflects the diff just published."""
        if self._monitor is None:
            time.sleep(0.2)
            return
        deadline = time.monotonic() + self._settle_timeout
        while time.monotonic() < deadline:
            if predicate(self._local_scene()):
                return
            time.sleep(0.05)
        raise RuntimeError(f"planning scene never settled: {what}")

    def _apply_diff(self, scene):
        scene.is_diff = True
        self._diff.publish(scene)

    def add_objects(self, objects, colors=None):
        """Add or replace collision objects, optionally with RGBA display colors."""
        scene = PlanningScene()
        scene.world.collision_objects = list(objects)
        for object_id, rgba in (colors or {}).items():
            scene.object_colors.append(
                ObjectColor(
                    id=object_id,
                    color=ColorRGBA(
                        r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3])
                    ),
                )
            )
        self._apply_diff(scene)

        wanted = {obj.id for obj in objects}
        self._settle(
            lambda msg: wanted <= {o.id for o in msg.world.collision_objects},
            f"objects {sorted(wanted)} never appeared",
        )

    def object_ids(self):
        """Names of the collision objects currently in the world."""
        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(
            components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        response = self._get.call(request)
        if response is None:
            raise RuntimeError("get_planning_scene did not answer")
        return {obj.id for obj in response.scene.world.collision_objects}

    def remove_objects(self, object_ids):
        """Remove objects, ignoring the ones that are not there.

        move_group rejects the whole diff if it is asked to remove an unknown
        object, so the caller cannot just fire and forget.
        """
        present = [name for name in object_ids if name in self.object_ids()]
        if not present:
            return

        scene = PlanningScene()
        for name in present:
            removal = CollisionObject()
            removal.id = name
            removal.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(removal)
        self._apply_diff(scene)

        gone = set(present)
        self._settle(
            lambda msg: not (gone & {o.id for o in msg.world.collision_objects}),
            f"objects {sorted(gone)} never went away",
        )

    def remove_object(self, object_id):
        self.remove_objects([object_id])

    def attach(self, object_id, link_name, touch_links):
        """Move a world object onto a robot link.

        move_group takes care of removing it from the world and of keeping its
        pose relative to the link, so no explicit pose bookkeeping is needed.
        """
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object.id = object_id
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = list(touch_links)

        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        self._apply_diff(scene)
        self._settle(
            lambda msg: object_id
            in {a.object.id for a in msg.robot_state.attached_collision_objects},
            f"{object_id} never attached to {link_name}",
        )

    def detach(self, object_id, link_name):
        """Drop an attached object back into the world at its current pose."""
        detached = AttachedCollisionObject()
        detached.link_name = link_name
        detached.object.id = object_id
        detached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [detached]
        scene.robot_state.is_diff = True
        self._apply_diff(scene)
        self._settle(
            lambda msg: object_id
            not in {a.object.id for a in msg.robot_state.attached_collision_objects},
            f"{object_id} never detached from {link_name}",
        )

    def move_object(self, object_id, frame_id, pose: Pose):
        """Teleport an existing world object to a new pose."""
        moved = CollisionObject()
        moved.id = object_id
        moved.header.frame_id = frame_id
        moved.pose = pose
        moved.operation = CollisionObject.MOVE

        scene = PlanningScene()
        scene.world.collision_objects = [moved]
        self._apply_diff(scene)

    def _allowed_collision_matrix(self):
        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = self._get.call(request)
        if response is None:
            raise RuntimeError("get_planning_scene did not answer")
        return response.scene.allowed_collision_matrix

    def set_collisions(self, pairs, allowed):
        """Whitelist or re-arm collisions between collision-object / link names.

        A planning-scene diff replaces the whole ACM, so the current one is read
        back, edited, and written as a whole.
        """
        acm = self._allowed_collision_matrix()
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]

        def ensure(name):
            if name in names:
                return names.index(name)
            names.append(name)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))
            return len(names) - 1

        for first, second in pairs:
            i, j = ensure(first), ensure(second)
            rows[i][j] = allowed
            rows[j][i] = allowed

        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]

        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        self._apply_diff(scene)

        def applied(msg):
            local = msg.allowed_collision_matrix
            index = {name: i for i, name in enumerate(local.entry_names)}

            def entry(first, second):
                # A missing entry means "not allowed": MoveIt drops an object's
                # row once it becomes an attached body, and that is exactly the
                # state a re-arm is asking for.
                if first not in index or second not in index:
                    return False
                return local.entry_values[index[first]].enabled[index[second]]

            return all(entry(first, second) == allowed for first, second in pairs)

        self._settle(applied, f"collision rules for {pairs} were not taken up")
