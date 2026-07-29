"""Unit tests for pure USD-stage verifier helpers."""

from scripts.verify_usd_stage import (
    BOX_COLLISION_PATH,
    PHYSICS_SCENE_PATH,
    DEFAULT_USD,
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
