"""The depth-based block finder, on synthetic scenes."""

import math

import numpy as np
import pytest

from franzi_skills.block_detector import back_project, block_pose_from_points

FX = FY = 600.0
CX, CY = 320.0, 240.0
K = ((FX, 0.0, CX), (0.0, FY, CY), (0.0, 0.0, 1.0))

BENCH_TOP = 0.80


def overhead_camera_scene(block_x, block_y, block_yaw=0.0, size=(0.05, 0.05, 0.09)):
    """Depth image of a bench plane with one block, camera looking straight
    down from 1.6 m with +z forward (optical convention)."""
    camera_height = 1.6
    depth = np.full((480, 640), camera_height - BENCH_TOP, dtype=np.float32)

    cos_yaw, sin_yaw = math.cos(block_yaw), math.sin(block_yaw)
    vs, us = np.mgrid[0:480, 0:640]
    plane_z = camera_height - BENCH_TOP
    world_x = (us - CX) * plane_z / FX
    world_y = (vs - CY) * plane_z / FY
    local_x = cos_yaw * (world_x - block_x) + sin_yaw * (world_y - block_y)
    local_y = -sin_yaw * (world_x - block_x) + cos_yaw * (world_y - block_y)
    on_block = (np.abs(local_x) < size[0] / 2) & (np.abs(local_y) < size[1] / 2)
    depth[on_block] = camera_height - (BENCH_TOP + size[2])
    return depth


def world_points(depth):
    """Camera at (0, 0, 1.6) looking straight down: x->x, y->y, z flips."""
    points = back_project(depth, K)
    return np.column_stack((points[:, 0], points[:, 1], 1.6 - points[:, 2]))


def test_finds_the_block():
    depth = overhead_camera_scene(0.05, -0.03)
    found = block_pose_from_points(world_points(depth), BENCH_TOP, (0.0, 0.0))
    assert found is not None
    x, y, _ = found
    assert x == pytest.approx(0.05, abs=0.01)
    assert y == pytest.approx(-0.03, abs=0.01)


def test_reports_yaw_folded_to_quarter_turn():
    depth = overhead_camera_scene(0.0, 0.0, block_yaw=math.radians(30), size=(0.09, 0.05, 0.09))
    found = block_pose_from_points(world_points(depth), BENCH_TOP, (0.0, 0.0))
    assert found is not None
    assert found[2] == pytest.approx(math.radians(30), abs=math.radians(6))


def test_empty_bench_finds_nothing():
    depth = overhead_camera_scene(10.0, 10.0)  # block far outside the frame
    assert block_pose_from_points(world_points(depth), BENCH_TOP, (0.0, 0.0)) is None


def test_ignores_blocks_outside_the_search_window():
    depth = overhead_camera_scene(0.6, 0.0)  # someone else's part
    found = block_pose_from_points(
        world_points(depth), BENCH_TOP, (0.0, 0.0), search_radius=0.3
    )
    assert found is None


def test_depth_noise_does_not_move_the_centre():
    rng = np.random.default_rng(0)
    depth = overhead_camera_scene(0.02, 0.01)
    depth = depth + rng.normal(0.0, 0.004, depth.shape).astype(np.float32)
    found = block_pose_from_points(world_points(depth), BENCH_TOP, (0.0, 0.0))
    assert found is not None
    assert found[0] == pytest.approx(0.02, abs=0.012)
    assert found[1] == pytest.approx(0.01, abs=0.012)


def test_rgb_on_plane_finds_the_block():
    import numpy as np

    from franzi_skills.block_detector import block_pose_from_rgb

    # Overhead camera 1.6 m up, looking straight down (optical z down):
    # world x -> image u, world y -> image v, plane at the block's top.
    top = BENCH_TOP + 0.04
    camera_to_world = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 1.6],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    image = np.full((480, 640, 3), 90, dtype=np.uint8)  # dark grey bench
    depth_to_plane = 1.6 - top
    block_x, block_y = 0.06, -0.02
    u0 = int(CX + block_x * FX / depth_to_plane)
    v0 = int(CY + block_y * FY / depth_to_plane)
    half = int(0.035 * FX / depth_to_plane)
    image[v0 - half : v0 + half, u0 - half : u0 + half] = 240  # white part
    # A tag-like patch: white with black cells - must be rejected.
    image[100:180, 100:180] = 240
    image[120:160, 120:160] = 10

    found = block_pose_from_rgb(
        image, K, camera_to_world, top, (0.0, 0.0)
    )
    assert found is not None
    x, y, _ = found
    assert abs(x - block_x) < 0.012
    assert abs(y - block_y) < 0.012


def test_tag_white_cells_are_not_the_part():
    import numpy as np

    from franzi_skills.block_detector import block_pose_from_rgb

    top = BENCH_TOP + 0.04
    camera_to_world = np.array(
        [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, -1.0, 1.6], [0, 0, 0, 1.0]]
    )
    image = np.full((480, 640, 3), 120, dtype=np.uint8)
    # A tag: white sheet with black cells and clean white cells inside them.
    image[200:280, 260:340] = 250
    image[210:270, 270:330] = 10
    image[228:252, 288:312] = 250  # the bright central cell, ~5 cm on-plane
    found = block_pose_from_rgb(image, K, camera_to_world, top, (0.0, 0.0))
    assert found is None


def test_dimmer_block_is_still_found():
    import numpy as np

    from franzi_skills.block_detector import block_pose_from_rgb

    top = BENCH_TOP + 0.04
    camera_to_world = np.array(
        [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, -1.0, 1.6], [0, 0, 0, 1.0]]
    )
    image = np.full((480, 640, 3), 120, dtype=np.uint8)
    depth_to_plane = 1.6 - top
    half = int(0.035 * FX / depth_to_plane)
    image[240 - half : 240 + half, 320 - half : 320 + half] = 175  # not-so-bright part
    found = block_pose_from_rgb(image, K, camera_to_world, top, (0.0, 0.0))
    assert found is not None
