"""Portable composition checks for the P2 AprilTag assets and scene overlay."""

from pathlib import Path


ASSET = Path("usd/assets/tags/apriltag_36h11.usda")
OVERLAY = Path("usd/scenes/warehouse_box_transfer_apriltags.usda")
TABLE_OVERLAY = Path("usd/scenes/warehouse_box_transfer_table_apriltags.usda")
PHYSICS_OVERLAY = Path("usd/scenes/warehouse_box_transfer_physics.usda")
BOX_ASSET = Path("usd/assets/props/blue_transport_box.usda")
SCENE = Path("usd/scenes/warehouse_box_transfer.usda")
PHYSICS_SMOKE = Path("scripts/verify_box_physics_smoke.py")
WORKCELL_CAPTURE = Path("scripts/capture_workcell_evidence.py")


def test_apriltag_asset_uses_the_versioned_official_isaac_material_source() -> None:
    """The local board delegates pixels and shader behavior to Isaac 5.1 assets."""

    asset = ASSET.read_text(encoding="utf-8")
    assert 'apriltag:assetVersion = "1.0.0"' in asset
    assert 'apriltag:family = "tag36h11"' in asset
    assert "AprilTag.mdl" in asset
    assert "Textures/tag36h11.png" in asset
    assert "int inputs:tag_size = 10" in asset
    assert "int inputs:tags_per_row = 24" in asset
    assert "outputs:mdl:surface.connect" in asset
    assert 'renderType = "material"' in asset
    assert "bool doubleSided = 1" in asset
    assert "/home/" not in asset
    assert "@../../../vendor/isaac_assets/" in asset


def test_apriltag_scene_overlay_attaches_tag_zero_to_the_box() -> None:
    """The tag is box-local, metre-scaled, front-facing, and portable."""

    overlay = OVERLAY.read_text(encoding="utf-8")
    assert 'apriltag:layerVersion = "1.0.0"' in overlay
    assert "@../assets/tags/apriltag_36h11.usda@" in overlay
    assert 'over "BlueTransportBox"' in overlay
    assert 'def Xform "AprilTag_0"' in overlay
    assert 'double3 xformOp:translate = (-0.3040666, 0, 0.0851469)' in overlay
    assert 'double3 xformOp:rotateXYZ = (0, -90, 0)' in overlay
    assert "DropZoneTag" not in overlay
    assert "/home/" not in overlay


def test_pick_table_overlay_places_symmetric_unique_tags_on_top_corners() -> None:
    """Both fixed table landmarks are upward-facing, scaled, and portable."""

    overlay = TABLE_OVERLAY.read_text(encoding="utf-8")
    assert 'apriltag:layerVersion = "1.0.0"' in overlay
    assert "@../assets/tags/apriltag_36h11.usda@" in overlay
    assert 'over "Workcell"' in overlay
    assert 'over "PickTable"' in overlay
    assert 'def Xform "AprilTag_1"' in overlay
    assert 'def Xform "AprilTag_2"' in overlay
    assert 'double3 xformOp:translate = (-1.16, 0.30, 0.9946)' in overlay
    assert 'double3 xformOp:translate = (1.16, 0.30, 0.9946)' in overlay
    assert 'double3 xformOp:scale = (0.8, 0.8, 0.8)' in overlay
    assert 'custom int apriltag:id = 1' in overlay
    assert 'custom int apriltag:id = 2' in overlay
    assert 'custom double apriltag:sizeMeters = 0.08' in overlay
    assert 'int inputs:tag_id = 1' in overlay
    assert 'int inputs:tag_id = 2' in overlay
    assert 'scenario:role = "pick_table_right_static_landmark"' in overlay
    assert 'scenario:role = "pick_table_left_static_landmark"' in overlay
    assert "xformOp:rotate" not in overlay
    assert "/home/" not in overlay

    table_half_length_m = 2.4736465 / 2
    table_half_width_m = 0.762 / 2
    backing_half_size_m = 0.052 * 0.8
    local_y_m = 0.30
    for local_x_m in (-1.16, 1.16):
        edge_margins_m = (
            table_half_length_m - abs(local_x_m) - backing_half_size_m,
            table_half_width_m - abs(local_y_m) - backing_half_size_m,
        )
        assert min(edge_margins_m) >= 0.03
    # PickTable has world translate (1.25, 0, 0) and a +90 degree Z rotation.
    world_xy_m = tuple(
        (1.25 - local_y_m, local_x_m) for local_x_m in (-1.16, 1.16)
    )
    assert world_xy_m == ((0.95, -1.16), (0.95, 1.16))


def test_box_and_pick_table_tags_have_distinct_ids_and_motion_roles() -> None:
    """Tag 0 follows the box, while Tags 1 and 2 remain fixed on PickTable."""

    box_overlay = OVERLAY.read_text(encoding="utf-8")
    table_overlay = TABLE_OVERLAY.read_text(encoding="utf-8")

    assert 'custom int apriltag:id = 0' in box_overlay
    assert 'scenario:role = "box_pose_landmark"' in box_overlay
    assert 'custom int apriltag:id = 1' in table_overlay
    assert 'custom int apriltag:id = 2' in table_overlay
    assert 'scenario:role = "pick_table_right_static_landmark"' in table_overlay
    assert 'scenario:role = "pick_table_left_static_landmark"' in table_overlay
    assert 'over "BlueTransportBox"' not in table_overlay


def test_foundation_scene_composes_the_apriltag_overlay() -> None:
    """The public scene composes independent moving-box and fixed-table layers."""

    scene = SCENE.read_text(encoding="utf-8")
    assert "@warehouse_box_transfer_workcell.usda@" in scene
    assert "@warehouse_box_transfer_apriltags.usda@" in scene
    assert "@warehouse_box_transfer_table_apriltags.usda@" in scene
    assert "AprilTag live detection" in scene


def test_table_tag_layer_is_hashed_without_polluting_table_contact_bounds() -> None:
    """Evidence includes the tag layer while physics bounds only the table model."""

    physics_smoke = PHYSICS_SMOKE.read_text(encoding="utf-8")
    workcell_capture = WORKCELL_CAPTURE.read_text(encoding="utf-8")
    layer = 'Path("usd/scenes/warehouse_box_transfer_table_apriltags.usda")'

    assert layer in physics_smoke
    assert layer in workcell_capture
    assert 'PICK_TABLE_MODEL_PATH = f"{PICK_TABLE_PATH}/Model"' in physics_smoke
    assert "stage.GetPrimAtPath(PICK_TABLE_MODEL_PATH)" in physics_smoke


def test_physics_overlay_keeps_the_box_dynamic_with_project_owned_contact_data() -> None:
    """Physics must live in an overlay, never in the vendored NVIDIA asset."""

    overlay = PHYSICS_OVERLAY.read_text(encoding="utf-8")
    box_asset = BOX_ASSET.read_text(encoding="utf-8")
    scene = SCENE.read_text(encoding="utf-8")

    assert "def PhysicsScene \"PhysicsScene\"" in overlay
    assert '"PhysicsRigidBodyAPI", "PhysicsMassAPI"' in overlay
    assert "float physics:mass = 3.5" in overlay
    assert "def Cube \"PhysicsCollision\"" in overlay
    assert '"PhysicsCollisionAPI"' in overlay
    assert "SM_Crate_A08_Blue_01.usd@" in box_asset
    assert "SM_Crate_A08_Blue_01_physics.usd@" not in box_asset
    assert '"PhysicsMaterialAPI"' in overlay
    assert "physics:staticFriction = 0.65" in overlay
    assert "physics:dynamicFriction = 0.50" in overlay
    assert "BlueTransportBox/AprilTag_0" not in overlay
    assert "/home/" not in overlay
    assert "@warehouse_box_transfer_physics.usda@" in scene
