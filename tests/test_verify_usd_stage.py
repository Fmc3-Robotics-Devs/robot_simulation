"""Unit tests for pure USD-stage verifier helpers."""

from scripts.verify_usd_stage import (
    APRILTAG_EXPECTATIONS,
    BOX_APRILTAG_PATH,
    BOX_COLLISION_PATH,
    DEFAULT_USD,
    PHYSICS_SCENE_PATH,
    PICK_TABLE_APRILTAG_1_PATH,
    PICK_TABLE_APRILTAG_2_PATH,
    parse_args,
    resolve_required_prim_paths,
)


def test_default_arguments_target_delivered_scene() -> None:
    """The command works without explicitly supplying a scene path."""

    assert parse_args([]).usd == DEFAULT_USD


def test_extra_relative_prim_is_resolved_below_default_prim() -> None:
    """Short prim names are convenient for checks within the scene assembly."""

    required = resolve_required_prim_paths("/Scene", ["Robot", "/World/Camera"])

    assert "/Scene/Robot" in required
    assert "/World/Camera" in required


def test_default_requirements_include_dynamic_box_physics_prims() -> None:
    """The delivered-stage verifier cannot silently skip the physics overlay."""

    required = resolve_required_prim_paths("/WheelBotBoxTransfer", [])

    assert PHYSICS_SCENE_PATH in required
    assert BOX_COLLISION_PATH in required


def test_default_requirements_distinguish_box_and_pick_table_tags() -> None:
    """The stage gate requires moving Tag 0 and both static table-tag prims."""

    required = resolve_required_prim_paths("/WheelBotBoxTransfer", [])

    assert BOX_APRILTAG_PATH in required
    assert PICK_TABLE_APRILTAG_1_PATH in required
    assert PICK_TABLE_APRILTAG_2_PATH in required
    assert APRILTAG_EXPECTATIONS["box_tag_0"]["id"] == 0
    assert APRILTAG_EXPECTATIONS["box_tag_0"]["role"] == "box_pose_landmark"
    assert APRILTAG_EXPECTATIONS["pick_table_tag_1"]["id"] == 1
    assert (
        APRILTAG_EXPECTATIONS["pick_table_tag_1"]["role"]
        == "pick_table_right_static_landmark"
    )
    assert APRILTAG_EXPECTATIONS["pick_table_tag_1"]["visible_normal"] == (
        0.0,
        0.0,
        1.0,
    )
    assert APRILTAG_EXPECTATIONS["pick_table_tag_1"]["origin_m"] == (
        0.95,
        -1.16,
        0.9946,
    )
    assert APRILTAG_EXPECTATIONS["pick_table_tag_2"]["id"] == 2
    assert (
        APRILTAG_EXPECTATIONS["pick_table_tag_2"]["role"]
        == "pick_table_left_static_landmark"
    )
    assert APRILTAG_EXPECTATIONS["pick_table_tag_2"]["visible_normal"] == (
        0.0,
        0.0,
        1.0,
    )
    assert APRILTAG_EXPECTATIONS["pick_table_tag_2"]["origin_m"] == (
        0.95,
        1.16,
        0.9946,
    )
