"""Portable composition checks for the P2 AprilTag assets and scene overlay."""

from pathlib import Path


ASSET = Path("usd/assets/tags/apriltag_36h11.usda")
OVERLAY = Path("usd/scenes/warehouse_box_transfer_apriltags.usda")
PHYSICS_OVERLAY = Path("usd/scenes/warehouse_box_transfer_physics.usda")
BOX_ASSET = Path("usd/assets/props/blue_transport_box.usda")
SCENE = Path("usd/scenes/warehouse_box_transfer.usda")


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


def test_foundation_scene_composes_the_apriltag_overlay() -> None:
    """The public scene entry must include the independently editable tag layer."""

    scene = SCENE.read_text(encoding="utf-8")
    assert "@warehouse_box_transfer_workcell.usda@" in scene
    assert "@warehouse_box_transfer_apriltags.usda@" in scene
    assert "AprilTag live detection" in scene


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
