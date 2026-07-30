"""Open a USD scene in headless Isaac Sim and validate its required prims."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_USD = PROJECT_ROOT / "usd/scenes/warehouse_box_transfer.usda"
DEFAULT_REQUIRED_PRIMS = (
    "/WheelBotBoxTransfer",
    "/WheelBotBoxTransfer/WheelBot",
    "/WheelBotBoxTransfer/BlueTransportBox",
    "/WheelBotBoxTransfer/BlueTransportBox/AprilTag_0",
    "/WheelBotBoxTransfer/Workcell/PickTable/AprilTag_1",
    "/WheelBotBoxTransfer/Workcell/PickTable/AprilTag_2",
    "/WheelBotBoxTransfer/DropZone",
    "/WheelBotBoxTransfer/PhysicsScene",
    "/WheelBotBoxTransfer/BlueTransportBox/PhysicsCollision",
)
PHYSICS_SCENE_PATH = "/WheelBotBoxTransfer/PhysicsScene"
ROBOT_PATH = "/WheelBotBoxTransfer/WheelBot"
BOX_PATH = "/WheelBotBoxTransfer/BlueTransportBox"
BOX_COLLISION_PATH = f"{BOX_PATH}/PhysicsCollision"
BOX_MATERIAL_PATH = "/WheelBotBoxTransfer/PhysicsMaterials/BlueTransportBoxMaterial"
BOX_APRILTAG_PATH = f"{BOX_PATH}/AprilTag_0"
PICK_TABLE_APRILTAG_1_PATH = (
    "/WheelBotBoxTransfer/Workcell/PickTable/AprilTag_1"
)
PICK_TABLE_APRILTAG_2_PATH = (
    "/WheelBotBoxTransfer/Workcell/PickTable/AprilTag_2"
)
APRILTAG_EXPECTATIONS = {
    "box_tag_0": {
        "path": BOX_APRILTAG_PATH,
        "id": 0,
        "family": "tag36h11",
        "size_m": 0.1,
        "role": "box_pose_landmark",
        "visible_normal": (-1.0, 0.0, 0.0),
    },
    "pick_table_tag_1": {
        "path": PICK_TABLE_APRILTAG_1_PATH,
        "id": 1,
        "family": "tag36h11",
        "size_m": 0.08,
        "role": "pick_table_right_static_landmark",
        # PickTable rotates only around world Z, so both unrotated boards remain
        # upward-facing. Their table-local ±1.16 m X coordinates map to the
        # symmetric formal workcell coordinates below.
        "visible_normal": (0.0, 0.0, 1.0),
        "origin_m": (0.95, -1.16, 0.9946),
    },
    "pick_table_tag_2": {
        "path": PICK_TABLE_APRILTAG_2_PATH,
        "id": 2,
        "family": "tag36h11",
        "size_m": 0.08,
        "role": "pick_table_left_static_landmark",
        "visible_normal": (0.0, 0.0, 1.0),
        "origin_m": (0.95, 1.16, 0.9946),
    },
}
CAMERA_PATHS = {
    "head_d435": (
        "/WheelBotBoxTransfer/WheelBot/head_d435_Link/"
        "head_d435_camera_mount/camera"
    ),
    "chest_d435": (
        "/WheelBotBoxTransfer/WheelBot/body_d435_Link/"
        "chest_d435_camera_mount/camera"
    ),
    "left_wrist_d405": (
        "/WheelBotBoxTransfer/WheelBot/left_wrist_d405_Link/"
        "left_wrist_d405_camera_mount/camera"
    ),
    "right_wrist_d405": (
        "/WheelBotBoxTransfer/WheelBot/right_wrist_d405_Link/"
        "right_wrist_d405_camera_mount/camera"
    ),
}
CAMERA_OPTICAL_FRAME_PATHS = {
    "head_d435": f"{ROBOT_PATH}/head_d435_Link/head_d435_optical_frame",
    "chest_d435": f"{ROBOT_PATH}/body_d435_Link/chest_d435_optical_frame",
    "left_wrist_d405": (
        f"{ROBOT_PATH}/left_wrist_d405_Link/left_wrist_d405_optical_frame"
    ),
    "right_wrist_d405": (
        f"{ROBOT_PATH}/right_wrist_d405_Link/right_wrist_d405_optical_frame"
    ),
}
D405_WRIST_ROLL_PATHS = {
    "left_wrist_d405": f"{ROBOT_PATH}/left_wrist_roll_Link",
    "right_wrist_d405": f"{ROBOT_PATH}/right_wrist_roll_Link",
}
ROBOT_EXPECTED_COLLIDER_COUNT = 37
GROUND_Z_M = 0.0
GROUND_CONTACT_TOLERANCE_M = 0.0005
SCALE_CHECKS = {
    "/WheelBotBoxTransfer/WheelBot": {
        "minimum_size_m": (0.45, 0.45, 1.4),
        "maximum_size_m": (1.2, 1.2, 2.2),
    },
    "/WheelBotBoxTransfer/BlueTransportBox": {
        "minimum_size_m": (0.55, 0.35, 0.15),
        "maximum_size_m": (0.70, 0.50, 0.25),
    },
    "/WheelBotBoxTransfer/Workcell/PickTable": {
        "minimum_size_m": (0.70, 2.3, 0.90),
        "maximum_size_m": (0.90, 2.6, 1.10),
    },
    "/WheelBotBoxTransfer/Workcell/DeliveryTable": {
        "minimum_size_m": (2.3, 0.70, 0.90),
        "maximum_size_m": (2.6, 0.90, 1.10),
    },
}


def portable_path(path: Path) -> str:
    """Prefer a repository-relative path in evidence produced inside the project."""

    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(resolved)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the scene path and any additional prim paths to require."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "usd",
        nargs="?",
        type=Path,
        default=DEFAULT_USD,
        help=f"USD stage to open (default: {DEFAULT_USD.relative_to(PROJECT_ROOT)})",
    )
    parser.add_argument(
        "--require-prim",
        action="append",
        default=[],
        metavar="PATH",
        help=(
            "require an additional prim. Relative names are resolved below the "
            "stage default prim; may be repeated."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="optional JSON evidence file",
    )
    return parser.parse_args(argv)


def resolve_required_prim_paths(default_prim_path: str, extra_paths: Sequence[str]) -> list[str]:
    """Return built-in requirements plus caller-supplied absolute prim paths."""

    resolved = list(DEFAULT_REQUIRED_PRIMS)
    base = default_prim_path.rstrip("/")
    for path in extra_paths:
        if not path:
            raise ValueError("--require-prim cannot be empty")
        resolved.append(path if path.startswith("/") else f"{base}/{path}")
    return list(dict.fromkeys(resolved))


def validate_box_physics(stage) -> tuple[dict[str, object], list[str]]:
    """Validate the project-owned dynamic-box physics and contact contract."""

    from pxr import Usd, UsdPhysics

    physics_scene = stage.GetPrimAtPath(PHYSICS_SCENE_PATH)
    box = stage.GetPrimAtPath(BOX_PATH)
    collision = stage.GetPrimAtPath(BOX_COLLISION_PATH)
    material = stage.GetPrimAtPath(BOX_MATERIAL_PATH)
    enabled_box_colliders = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(box, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
    ]
    material_binding = collision.GetRelationship("physics:material:binding") if collision.IsValid() else None
    binding_targets = (
        [str(target) for target in material_binding.GetTargets()]
        if material_binding and material_binding.IsValid()
        else []
    )

    mass = UsdPhysics.MassAPI(box).GetMassAttr().Get() if box.IsValid() else None
    static_friction = material.GetAttribute("physics:staticFriction").Get() if material.IsValid() else None
    dynamic_friction = material.GetAttribute("physics:dynamicFriction").Get() if material.IsValid() else None
    restitution = material.GetAttribute("physics:restitution").Get() if material.IsValid() else None
    contract = {
        "physics_scene": physics_scene.IsValid() and physics_scene.GetTypeName() == "PhysicsScene",
        "rigid_body_api": box.IsValid() and box.HasAPI(UsdPhysics.RigidBodyAPI),
        "mass_api": box.IsValid() and box.HasAPI(UsdPhysics.MassAPI),
        "mass_kg": float(mass) if mass is not None else None,
        "collision_api": collision.IsValid() and collision.HasAPI(UsdPhysics.CollisionAPI),
        "enabled_box_colliders": enabled_box_colliders,
        "single_box_collider": enabled_box_colliders == [BOX_COLLISION_PATH],
        "material_path": str(material.GetPath()) if material.IsValid() else None,
        "material_binding_targets": binding_targets,
        "static_friction": float(static_friction) if static_friction is not None else None,
        "dynamic_friction": float(dynamic_friction) if dynamic_friction is not None else None,
        "restitution": float(restitution) if restitution is not None else None,
    }
    failures = [
        name
        for name, valid in {
            "PhysicsScene": contract["physics_scene"],
            "RigidBodyAPI": contract["rigid_body_api"],
            "MassAPI": contract["mass_api"],
            "CollisionAPI": contract["collision_api"],
            "single enabled box collider": contract["single_box_collider"],
            "collision material binding": BOX_MATERIAL_PATH in binding_targets,
            "positive mass": isinstance(mass, (float, int)) and mass > 0,
            "positive static friction": isinstance(static_friction, (float, int)) and static_friction > 0,
            "positive dynamic friction": isinstance(dynamic_friction, (float, int)) and dynamic_friction > 0,
        }.items()
        if not valid
    ]
    return contract, failures


def validate_robot_collisions(stage) -> tuple[dict[str, object], list[str]]:
    """Audit instance-proxy colliders and the fixed-base wheel contact plane."""

    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics

    robot = stage.GetPrimAtPath(ROBOT_PATH)
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def geometry_world_bounds(collider_prim) -> tuple[np.ndarray, np.ndarray]:
        """Return exact mesh-vertex AABB bounds, including invisible colliders."""

        minimum = np.full(3, float("inf"), dtype=float)
        maximum = np.full(3, float("-inf"), dtype=float)
        for geometry_prim in Usd.PrimRange(
            collider_prim,
            Usd.TraverseInstanceProxies(),
        ):
            points = geometry_prim.GetAttribute("points").Get()
            if not points:
                continue
            local_points = np.asarray(points, dtype=float)
            homogeneous = np.column_stack(
                (local_points, np.ones(len(local_points), dtype=float))
            )
            # Gf matrices use row-vector convention; transforming every source
            # vertex gives the exact support point of its convex hull.
            world_matrix = np.asarray(
                xform_cache.GetLocalToWorldTransform(geometry_prim),
                dtype=float,
            )
            world_points = homogeneous @ world_matrix
            euclidean_points = world_points[:, :3] / world_points[:, 3, None]
            minimum = np.minimum(minimum, np.min(euclidean_points, axis=0))
            maximum = np.maximum(maximum, np.max(euclidean_points, axis=0))
        return minimum * meters_per_unit, maximum * meters_per_unit

    colliders: list[dict[str, object]] = []
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        minimum_m, maximum_m = geometry_world_bounds(prim)
        minimum_z_m = float(minimum_m[2])
        approximation = prim.GetAttribute("physics:approximation").Get()
        colliders.append(
            {
                "path": str(prim.GetPath()),
                "approximation": str(approximation),
                "minimum_z_m": minimum_z_m,
                "minimum_m": minimum_m.tolist(),
                "maximum_m": maximum_m.tolist(),
            }
        )

    approximation_counts: dict[str, int] = {}
    for collider in colliders:
        approximation = str(collider["approximation"])
        approximation_counts[approximation] = (
            approximation_counts.get(approximation, 0) + 1
        )
    wheel_colliders = [
        collider
        for collider in colliders
        if any(
            wheel_link in str(collider["path"])
            for wheel_link in (
                "left_front_wheel_Link",
                "right_front_wheel_Link",
                "rear_wheel_Link",
            )
        )
    ]
    minimum_z_m = min(
        (float(collider["minimum_z_m"]) for collider in colliders),
        default=float("inf"),
    )
    wheel_minimums_m = [
        float(collider["minimum_z_m"]) for collider in wheel_colliders
    ]
    wheel_height_spread_m = (
        max(wheel_minimums_m) - min(wheel_minimums_m)
        if wheel_minimums_m
        else float("inf")
    )
    base_minimums_m = [
        float(collider["minimum_z_m"])
        for collider in colliders
        if "/base_link/" in str(collider["path"])
    ]
    base_clearance_m = (
        min(base_minimums_m) - GROUND_Z_M
        if base_minimums_m
        else None
    )
    ground_clearance_m = minimum_z_m - GROUND_Z_M
    box_collision = stage.GetPrimAtPath(BOX_COLLISION_PATH)
    box_size = float(box_collision.GetAttribute("size").Get())
    box_local_corners = np.asarray(
        [
            (x, y, z, 1.0)
            for x in (-box_size / 2.0, box_size / 2.0)
            for y in (-box_size / 2.0, box_size / 2.0)
            for z in (-box_size / 2.0, box_size / 2.0)
        ],
        dtype=float,
    )
    box_world_homogeneous = box_local_corners @ np.asarray(
        xform_cache.GetLocalToWorldTransform(box_collision),
        dtype=float,
    )
    box_world_points_m = (
        box_world_homogeneous[:, :3]
        / box_world_homogeneous[:, 3, None]
        * meters_per_unit
    )
    box_minimum_m = np.min(box_world_points_m, axis=0)
    box_maximum_m = np.max(box_world_points_m, axis=0)
    robot_box_aabb_candidates = []
    for collider in colliders:
        overlap_m = np.minimum(
            np.asarray(collider["maximum_m"], dtype=float),
            box_maximum_m,
        ) - np.maximum(
            np.asarray(collider["minimum_m"], dtype=float),
            box_minimum_m,
        )
        if np.all(overlap_m > 0.0):
            robot_box_aabb_candidates.append(
                {
                    "path": collider["path"],
                    "overlap_m": overlap_m.tolist(),
                }
            )
    contract = {
        "enabled_collider_count": len(colliders),
        "expected_collider_count": ROBOT_EXPECTED_COLLIDER_COUNT,
        "approximation_counts": approximation_counts,
        "collider_paths": [str(collider["path"]) for collider in colliders],
        "minimum_collider_z_m": minimum_z_m,
        "ground_z_m": GROUND_Z_M,
        "ground_clearance_m": ground_clearance_m,
        "wheel_collider_count": len(wheel_colliders),
        "wheel_minimum_z_m": wheel_minimums_m,
        "wheel_height_spread_m": wheel_height_spread_m,
        "base_clearance_m": base_clearance_m,
        "box_collision_minimum_m": box_minimum_m.tolist(),
        "box_collision_maximum_m": box_maximum_m.tolist(),
        "robot_box_aabb_candidates": robot_box_aabb_candidates,
    }
    failures = []
    if len(colliders) != ROBOT_EXPECTED_COLLIDER_COUNT:
        failures.append(
            f"expected {ROBOT_EXPECTED_COLLIDER_COUNT} robot colliders, "
            f"found {len(colliders)}"
        )
    if approximation_counts != {"convexHull": ROBOT_EXPECTED_COLLIDER_COUNT}:
        failures.append(
            f"robot collision approximation must be convexHull: "
            f"{approximation_counts}"
        )
    if len(wheel_colliders) != 3:
        failures.append(f"expected three wheel colliders, found {len(wheel_colliders)}")
    for wheel in wheel_colliders:
        wheel_clearance_m = float(wheel["minimum_z_m"]) - GROUND_Z_M
        if abs(wheel_clearance_m) > GROUND_CONTACT_TOLERANCE_M:
            failures.append(
                f"wheel collider {wheel['path']} is "
                f"{wheel_clearance_m:.6f} m from the floor"
            )
    if wheel_height_spread_m > 0.0001:
        failures.append(
            f"wheel contact planes differ by {wheel_height_spread_m:.6f} m"
        )
    if abs(ground_clearance_m) > GROUND_CONTACT_TOLERANCE_M:
        failures.append(
            f"lowest robot collider is {ground_clearance_m:.6f} m from the floor"
        )
    if base_clearance_m is None or base_clearance_m <= 0.0:
        failures.append(f"base_link clearance is not positive: {base_clearance_m}")
    return contract, failures


def inspect_apriltag(stage, path: str, meters_per_unit: float) -> dict[str, object]:
    """Return identity, pose, and official-material details for one composed tag."""

    from pxr import Gf, Usd, UsdGeom, UsdShade

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return {"prim_path": path, "valid": False}

    def attribute_value(target, name: str):
        attribute = target.GetAttribute(name)
        return attribute.Get() if attribute.IsValid() else None

    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    origin = matrix.Transform(Gf.Vec3d(0, 0, 0))
    normal = matrix.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
    face = stage.GetPrimAtPath(f"{path}/Face")
    shader = stage.GetPrimAtPath(f"{path}/AprilTagMaterial/Shader")
    face_vertex_counts: list[int] = []
    face_vertex_indices: list[int] = []
    edge_lengths_m: list[float] = []
    geometric_normal: list[float] | None = None
    authored_normal: list[float] | None = None
    if face.IsValid() and face.GetTypeName() == "Mesh":
        mesh = UsdGeom.Mesh(face)
        points = list(mesh.GetPointsAttr().Get() or [])
        face_vertex_counts = [
            int(value) for value in (mesh.GetFaceVertexCountsAttr().Get() or [])
        ]
        face_vertex_indices = [
            int(value) for value in (mesh.GetFaceVertexIndicesAttr().Get() or [])
        ]
        face_matrix = UsdGeom.XformCache(
            Usd.TimeCode.Default()
        ).GetLocalToWorldTransform(face)
        world_points = [
            face_matrix.Transform(
                Gf.Vec3d(float(point[0]), float(point[1]), float(point[2]))
            )
            for point in points
        ]
        if face_vertex_counts == [4] and len(face_vertex_indices) == 4:
            ordered_points = [world_points[index] for index in face_vertex_indices]
            edges = [
                ordered_points[(index + 1) % 4] - ordered_points[index]
                for index in range(4)
            ]
            edge_lengths_m = [
                float(edge.GetLength()) * meters_per_unit for edge in edges
            ]
            first, second = edges[:2]
            normal_vector = Gf.Vec3d(
                first[1] * second[2] - first[2] * second[1],
                first[2] * second[0] - first[0] * second[2],
                first[0] * second[1] - first[1] * second[0],
            ).GetNormalized()
            geometric_normal = [float(value) for value in normal_vector]
        normals = list(mesh.GetNormalsAttr().Get() or [])
        if len(normals) == 1:
            normal_value = normals[0]
            world_normal = face_matrix.TransformDir(
                Gf.Vec3d(
                    float(normal_value[0]),
                    float(normal_value[1]),
                    float(normal_value[2]),
                )
            ).GetNormalized()
            authored_normal = [float(value) for value in world_normal]

    material_path = None
    if face.IsValid():
        material, _relationship = UsdShade.MaterialBindingAPI(
            face
        ).ComputeBoundMaterial()
        if material and material.GetPrim().IsValid():
            material_path = str(material.GetPath())

    st_values = attribute_value(face, "primvars:st") if face.IsValid() else None
    mdl_source = (
        attribute_value(shader, "info:mdl:sourceAsset") if shader.IsValid() else None
    )
    mdl_sub_identifier = (
        attribute_value(shader, "info:mdl:sourceAsset:subIdentifier")
        if shader.IsValid()
        else None
    )
    mosaic = attribute_value(shader, "inputs:tag_mosaic") if shader.IsValid() else None
    return {
        "prim_path": path,
        "valid": True,
        "tag_id": attribute_value(prim, "apriltag:id"),
        "shader_tag_id": (
            attribute_value(shader, "inputs:tag_id") if shader.IsValid() else None
        ),
        "family": attribute_value(prim, "apriltag:family"),
        "size_m": attribute_value(prim, "apriltag:sizeMeters"),
        "role": attribute_value(prim, "scenario:role"),
        "origin_m": [float(value) * meters_per_unit for value in origin],
        "visible_normal": [float(value) for value in normal],
        "face_prim": str(face.GetPath()) if face.IsValid() else None,
        "bound_material": material_path,
        "shader_prim": str(shader.GetPath()) if shader.IsValid() else None,
        "mdl_source_asset": str(mdl_source) if mdl_source is not None else None,
        "mdl_sub_identifier": (
            str(mdl_sub_identifier) if mdl_sub_identifier is not None else None
        ),
        "mosaic_asset": str(mosaic) if mosaic is not None else None,
        "shader_tag_size": (
            attribute_value(shader, "inputs:tag_size") if shader.IsValid() else None
        ),
        "shader_tags_per_row": (
            attribute_value(shader, "inputs:tags_per_row")
            if shader.IsValid()
            else None
        ),
        "shader_spacing": (
            attribute_value(shader, "inputs:spacing") if shader.IsValid() else None
        ),
        "face_vertex_counts": face_vertex_counts,
        "face_vertex_indices": face_vertex_indices,
        "face_edge_lengths_m": edge_lengths_m,
        "face_geometric_normal": geometric_normal,
        "face_authored_normal": authored_normal,
        "texture_coordinate_count": len(st_values or []),
        "double_sided": (
            bool(UsdGeom.Mesh(face).GetDoubleSidedAttr().Get())
            if face.IsValid() and face.GetTypeName() == "Mesh"
            else False
        ),
    }


def validate_stage(stage, extra_paths: Sequence[str]) -> dict[str, object]:
    """Check required prims, stage units, up-axis, and task-asset scale."""

    if stage is None:
        return {"ok": False, "error": "Isaac Sim did not open a USD stage"}

    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        return {"ok": False, "error": "USD stage has no valid defaultPrim"}

    default_path = str(default_prim.GetPath())
    required_paths = resolve_required_prim_paths(default_path, extra_paths)
    missing = [path for path in required_paths if not stage.GetPrimAtPath(path).IsValid()]
    from pxr import Gf, Usd, UsdGeom

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    up_axis = str(UsdGeom.GetStageUpAxis(stage))
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    bounds: dict[str, dict[str, list[float]]] = {}
    scale_failures: list[dict[str, object]] = []
    for path, limits in SCALE_CHECKS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        minimum = aligned.GetMin()
        maximum = aligned.GetMax()
        size = [
            float(maximum[index] - minimum[index]) * meters_per_unit
            for index in range(3)
        ]
        bounds[path] = {
            "minimum_m": [float(value) * meters_per_unit for value in minimum],
            "maximum_m": [float(value) * meters_per_unit for value in maximum],
            "size_m": size,
        }
        minimum_size = limits["minimum_size_m"]
        maximum_size = limits["maximum_size_m"]
        if any(
            value < minimum_size[index] or value > maximum_size[index]
            for index, value in enumerate(size)
        ):
            scale_failures.append(
                {
                    "prim": path,
                    "size_m": size,
                    "minimum_size_m": list(minimum_size),
                    "maximum_size_m": list(maximum_size),
                }
            )

    apriltags = {
        name: inspect_apriltag(stage, expectation["path"], meters_per_unit)
        for name, expectation in APRILTAG_EXPECTATIONS.items()
    }
    apriltag_failures: list[str] = []
    for name, expectation in APRILTAG_EXPECTATIONS.items():
        contract = apriltags[name]
        if not contract["valid"]:
            apriltag_failures.append(f"{name}: missing prim {expectation['path']}")
            continue
        for key in ("id", "family", "role"):
            actual_key = "tag_id" if key == "id" else key
            if contract[actual_key] != expectation[key]:
                apriltag_failures.append(
                    f"{name}: {actual_key}={contract[actual_key]!r}, "
                    f"expected {expectation[key]!r}"
                )
        if contract["shader_tag_id"] != expectation["id"]:
            apriltag_failures.append(
                f"{name}: shader_tag_id={contract['shader_tag_id']!r}, "
                f"expected {expectation['id']}"
            )
        if (
            contract["size_m"] is None
            or abs(float(contract["size_m"]) - float(expectation["size_m"])) > 1e-9
        ):
            apriltag_failures.append(
                f"{name}: size_m={contract['size_m']!r}, "
                f"expected {expectation['size_m']}"
            )
        actual_normal = Gf.Vec3d(*contract["visible_normal"]).GetNormalized()
        expected_normal = Gf.Vec3d(*expectation["visible_normal"]).GetNormalized()
        if float(actual_normal * expected_normal) < 0.999:
            apriltag_failures.append(
                f"{name}: visible_normal={contract['visible_normal']!r}, "
                f"expected {expectation['visible_normal']}"
            )
        for normal_key in ("face_geometric_normal", "face_authored_normal"):
            face_normal = contract[normal_key]
            if (
                face_normal is None
                or float(Gf.Vec3d(*face_normal).GetNormalized() * expected_normal)
                < 0.999
            ):
                apriltag_failures.append(
                    f"{name}: {normal_key}={face_normal!r}, "
                    f"expected {expectation['visible_normal']}"
                )
        edge_lengths = contract["face_edge_lengths_m"]
        if len(edge_lengths) != 4 or any(
            abs(float(length) - float(expectation["size_m"])) > 1e-6
            for length in edge_lengths
        ):
            apriltag_failures.append(
                f"{name}: face_edge_lengths_m={edge_lengths!r}, "
                f"expected four {expectation['size_m']} m edges"
            )
        expected_origin = expectation.get("origin_m")
        if expected_origin is not None and any(
            abs(float(contract["origin_m"][index]) - float(expected_origin[index]))
            > 1e-4
            for index in range(3)
        ):
            apriltag_failures.append(
                f"{name}: origin_m={contract['origin_m']!r}, "
                f"expected {expected_origin}"
            )
        if (
            not contract["face_prim"]
            or contract["bound_material"]
            != f"{expectation['path']}/AprilTagMaterial"
            or contract["shader_prim"]
            != f"{expectation['path']}/AprilTagMaterial/Shader"
            or "Isaac/Materials/AprilTag/AprilTag.mdl"
            not in (contract["mdl_source_asset"] or "")
            or contract["mdl_sub_identifier"] != "AprilTag"
            or "Isaac/Materials/AprilTag/Textures/tag36h11.png"
            not in (contract["mosaic_asset"] or "")
            or contract["shader_tag_size"] != 10
            or contract["shader_tags_per_row"] != 24
            or contract["shader_spacing"] != 1
            or contract["face_vertex_counts"] != [4]
            or contract["face_vertex_indices"] != [0, 1, 2, 3]
            or contract["texture_coordinate_count"] != 4
            or not contract["double_sided"]
        ):
            apriltag_failures.append(f"{name}: incomplete render contract")

    tag_ids = [
        contract["tag_id"]
        for contract in apriltags.values()
        if contract["valid"]
    ]
    if len(tag_ids) != len(set(tag_ids)):
        apriltag_failures.append(f"AprilTag IDs must be unique, found {tag_ids}")

    # Preserve the original Tag 0 evidence fields while adding the structured
    # three-landmark contract below.
    box_tag = apriltags["box_tag_0"]
    tag_pose = (
        {
            "origin_m": box_tag["origin_m"],
            "visible_normal": box_tag["visible_normal"],
        }
        if box_tag["valid"]
        else None
    )
    tag_render_contract = (
        {
            key: box_tag[key]
            for key in (
                "face_prim",
                "bound_material",
                "shader_prim",
                "mdl_source_asset",
                "mdl_sub_identifier",
                "mosaic_asset",
                "texture_coordinate_count",
                "double_sided",
            )
        }
        if box_tag["valid"]
        else None
    )

    camera_contract: dict[str, dict[str, object]] = {}
    camera_failures: list[str] = []
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for name, path in CAMERA_PATHS.items():
        prim = stage.GetPrimAtPath(path)
        optical_prim = stage.GetPrimAtPath(CAMERA_OPTICAL_FRAME_PATHS[name])
        clipping = (
            UsdGeom.Camera(prim).GetClippingRangeAttr().Get()
            if prim.IsValid() and prim.GetTypeName() == "Camera"
            else None
        )
        clipping_values = (
            [float(clipping[0]), float(clipping[1])]
            if clipping is not None
            else None
        )
        valid_clipping = (
            clipping_values is not None
            and abs(clipping_values[0] - 0.05) <= 1e-6
            and abs(clipping_values[1] - 100.0) <= 1e-6
        )
        axis_dots: dict[str, float] | None = None
        d405_lens_axis_dot: float | None = None
        d405_gripper_axis_dot: float | None = None
        if prim.IsValid() and optical_prim.IsValid():
            camera_matrix = xform_cache.GetLocalToWorldTransform(prim)
            optical_matrix = xform_cache.GetLocalToWorldTransform(optical_prim)
            camera_axes = {
                "right": camera_matrix.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized(),
                "down": camera_matrix.TransformDir(Gf.Vec3d(0, -1, 0)).GetNormalized(),
                "forward": camera_matrix.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized(),
            }
            optical_axes = {
                "right": optical_matrix.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized(),
                "down": optical_matrix.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized(),
                "forward": optical_matrix.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized(),
            }
            axis_dots = {
                axis: float(camera_axes[axis] * optical_axes[axis])
                for axis in ("right", "down", "forward")
            }
            if name.endswith("wrist_d405"):
                housing_path = str(optical_prim.GetParent().GetPath())
                housing_matrix = xform_cache.GetLocalToWorldTransform(
                    stage.GetPrimAtPath(housing_path)
                )
                # The physical dual-lens plane is the housing -Z face.  The
                # corrected URDF fixed joint aligns it with wrist-roll -Z.
                physical_lens_axis = housing_matrix.TransformDir(
                    Gf.Vec3d(0, 0, -1)
                ).GetNormalized()
                d405_lens_axis_dot = float(
                    camera_axes["forward"] * physical_lens_axis
                )
                wrist_matrix = xform_cache.GetLocalToWorldTransform(
                    stage.GetPrimAtPath(D405_WRIST_ROLL_PATHS[name])
                )
                gripper_direction = wrist_matrix.TransformDir(
                    Gf.Vec3d(0, 0, -1)
                ).GetNormalized()
                d405_gripper_axis_dot = float(
                    camera_axes["forward"] * gripper_direction
                )
        axes_valid = (
            axis_dots is not None
            and min(axis_dots.values()) >= 0.999
            and (d405_lens_axis_dot is None or d405_lens_axis_dot >= 0.999)
            and (
                d405_gripper_axis_dot is None
                or d405_gripper_axis_dot >= 0.999
            )
        )
        camera_contract[name] = {
            "prim_path": path,
            "optical_frame_path": CAMERA_OPTICAL_FRAME_PATHS[name],
            "camera_prim": prim.IsValid() and prim.GetTypeName() == "Camera",
            "optical_frame_prim": optical_prim.IsValid(),
            "clipping_range_m": clipping_values,
            "clipping_range_valid": valid_clipping,
            "usd_to_ros_axis_dots": axis_dots,
            "d405_camera_forward_to_housing_lens_axis_dot": d405_lens_axis_dot,
            "d405_camera_forward_to_gripper_axis_dot": d405_gripper_axis_dot,
            "axes_valid": axes_valid,
        }
        if (
            not camera_contract[name]["camera_prim"]
            or not camera_contract[name]["optical_frame_prim"]
            or not valid_clipping
            or not axes_valid
        ):
            camera_failures.append(name)

    metadata_failures = []
    if abs(meters_per_unit - 1.0) > 1e-9:
        metadata_failures.append(
            f"metersPerUnit must be 1.0, found {meters_per_unit}"
        )
    if up_axis.upper() != "Z":
        metadata_failures.append(f"upAxis must be Z, found {up_axis}")
    physics, physics_failures = validate_box_physics(stage)
    robot_collisions, robot_collision_failures = validate_robot_collisions(stage)
    return {
        "ok": (
            not missing
            and not metadata_failures
            and not scale_failures
            and not physics_failures
            and not robot_collision_failures
            and not apriltag_failures
            and not camera_failures
        ),
        "default_prim": default_path,
        "required_prims": required_paths,
        "missing_prims": missing,
        "meters_per_unit": meters_per_unit,
        "up_axis": up_axis,
        "metadata_failures": metadata_failures,
        "bounds": bounds,
        "scale_failures": scale_failures,
        "physics": physics,
        "physics_failures": physics_failures,
        "robot_collisions": robot_collisions,
        "robot_collision_failures": robot_collision_failures,
        "apriltags": apriltags,
        "apriltag_failures": apriltag_failures,
        "cameras": camera_contract,
        "camera_failures": camera_failures,
        "apriltag_pose": tag_pose,
        "apriltag_render_contract": tag_render_contract,
    }


def emit_result(result: dict[str, object], output_path: Path | None) -> None:
    """Print a result and optionally persist it before Kit shuts the process down."""

    serialized = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(serialized, encoding="utf-8")
    # Kit may replace Python's normal stdout during shutdown.
    sys.__stdout__.write(serialized)
    sys.__stdout__.flush()


def open_and_validate(
    usd_path: Path,
    extra_paths: Sequence[str],
    output_path: Path | None = None,
) -> dict[str, object]:
    """Launch headless Isaac Sim, open ``usd_path``, and return validation data."""

    if not usd_path.is_file():
        result = {"ok": False, "error": f"USD file does not exist: {usd_path}"}
        emit_result(result, output_path)
        return result

    # Omni modules must be imported only after SimulationApp has started.
    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
    finally:
        # Do not pass this script's flags to Kit itself.
        sys.argv = script_argv
    try:
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path.resolve())):
            return {"ok": False, "error": f"Isaac Sim could not open USD: {usd_path}"}
        # Let Isaac finish composing the stage before inspecting its prims.
        app.update()
        result = validate_stage(context.get_stage(), extra_paths)
        result["usd"] = portable_path(usd_path)
        # Emit before app.close(); Kit may terminate the process as part of
        # shutdown and skip Python statements that follow it.
        emit_result(result, output_path)
        return result
    finally:
        app.close()


def main(argv: Sequence[str] | None = None) -> int:
    """Run validation, print one JSON result, and return a shell exit status."""

    args = parse_args(argv)
    try:
        result = open_and_validate(args.usd, args.require_prim, args.output)
    except Exception as error:  # Isaac startup and USD composition errors are failures.
        result = {"ok": False, "error": str(error), "usd": portable_path(args.usd)}
        emit_result(result, args.output)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
