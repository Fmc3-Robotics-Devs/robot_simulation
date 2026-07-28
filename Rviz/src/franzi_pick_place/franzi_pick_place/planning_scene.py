"""Planning-scene plumbing on top of move_group's standard services.

Collision objects are pushed through ``/apply_planning_scene`` (rather than into
the local MoveItPy scene) so that move_group owns the scene: RViz then shows it
and every planner in the pipeline sees the same world.
"""

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
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

    def add_box(self, size, position, orientation=None):
        self._object.primitives.append(box(size))
        self._object.primitive_poses.append(make_pose(position, orientation))
        return self

    def build(self):
        return self._object


class PlanningSceneClient:
    """Thin synchronous wrapper around the move_group planning-scene services."""

    def __init__(self, node, timeout=10.0):
        self._node = node
        self._logger = node.get_logger()
        self._timeout = timeout
        self._apply = node.create_client(ApplyPlanningScene, "apply_planning_scene")
        self._get = node.create_client(GetPlanningScene, "get_planning_scene")

    def wait_for_services(self, timeout=30.0):
        for client in (self._apply, self._get):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"{client.srv_name} did not come up within {timeout}s")

    def _apply_diff(self, scene):
        scene.is_diff = True
        response = self._apply.call(ApplyPlanningScene.Request(scene=scene))
        if response is None or not response.success:
            raise RuntimeError("apply_planning_scene rejected the update")

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

    def remove_object(self, object_id):
        removal = CollisionObject()
        removal.id = object_id
        removal.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [removal]
        self._apply_diff(scene)

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

    def allow_collisions(self, pairs):
        """Whitelist collisions between pairs of collision-object / link names.

        A planning-scene diff replaces the whole ACM, so the current one is read
        back, extended, and written as a whole.
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
            rows[i][j] = True
            rows[j][i] = True

        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]

        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        self._apply_diff(scene)
