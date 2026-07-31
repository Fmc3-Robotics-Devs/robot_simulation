"""Find the workpiece in the head camera's depth image.

This is the vision path of `detect_material` (plan sections 5.4 and 16): the
block's pose is *measured from pixels*, not inferred from a tag offset. The
method is the plan's own recommendation for parts resting on a known plane:

    depth image -> back-project to points in the world frame
                -> keep the slab just above the bench top (only the part
                   can be there; the bench itself and the floor fall out)
                -> largest cluster inside the station's search window
                -> centroid = position, principal axis = yaw,
                   z pinned to the plane (depth noise never reaches z)

Colour never enters: a grey block on a grey bench segments poorly by hue, but
in *height above the bench* it is the only thing there. Depth is used as an
area statistic (median-filtered slab), never a single pixel - plan section 5.4.

The maths lives in pure functions over numpy arrays so the whole detector runs
in pytest with synthetic depth images; the ROS class underneath only feeds it
real frames. On hardware the same code consumes the RealSense's aligned depth.
"""

import math
import threading

import numpy as np


def back_project(depth, intrinsics, stride=2, max_range=3.0, edge_jump=0.03):
    """Camera-frame points from a depth image, sampled on a pixel grid.

    Pixels on a depth discontinuity are dropped: at an object's silhouette the
    sensor (and the renderer) interpolates between foreground and background,
    minting phantom points that hang in mid-air along the edge - at a bench's
    rim those land exactly in the height band the block is found in, and drag
    the cluster's centroid off the part. A pixel whose 3x3 neighbourhood
    spans more than ``edge_jump`` of depth is not a surface, so it does not
    get to vote.
    """
    fx, fy = intrinsics[0][0], intrinsics[1][1]
    cx, cy = intrinsics[0][2], intrinsics[1][2]

    finite = np.isfinite(depth)
    filled = np.where(finite, depth, 0.0)
    local_min = filled.copy()
    local_max = filled.copy()
    for shift_v in (-1, 0, 1):
        for shift_u in (-1, 0, 1):
            rolled = np.roll(np.roll(filled, shift_v, axis=0), shift_u, axis=1)
            local_min = np.minimum(local_min, rolled)
            local_max = np.maximum(local_max, rolled)
    on_surface = (local_max - local_min) < edge_jump

    height, width = depth.shape
    vs, us = np.mgrid[0:height:stride, 0:width:stride]
    ds = depth[::stride, ::stride]
    valid = (
        np.isfinite(ds)
        & (ds > 0.05)
        & (ds < max_range)
        & on_surface[::stride, ::stride]
    )
    zs, us, vs = ds[valid], us[valid], vs[valid]
    return np.column_stack(
        ((us - cx) * zs / fx, (vs - cy) * zs / fy, zs)
    )


def block_pose_from_points(
    points_world,
    bench_top,
    search_centre,
    search_radius=0.35,
    band=(0.012, 0.10),
    min_points=30,
    cell=0.01,
):
    """The block's (x, y, yaw) from world-frame points, or None.

    ``band`` is the height slab above the bench top that only the part can
    occupy. Clustering is a coarse occupancy grid: the connected component
    around the densest cell, which needs no tuning and cannot chain across
    the empty bench like nearest-neighbour clustering can.
    """
    points = np.asarray(points_world)
    if points.size == 0:
        return None

    dx = points[:, 0] - search_centre[0]
    dy = points[:, 1] - search_centre[1]
    keep = (
        (points[:, 2] > bench_top + band[0])
        & (points[:, 2] < bench_top + band[1])
        & (dx * dx + dy * dy < search_radius * search_radius)
    )
    slab = points[keep]
    if len(slab) < min_points:
        return None

    # Occupancy grid over xy; flood-fill from the densest cell.
    xy = slab[:, :2]
    origin = xy.min(axis=0)
    indices = np.floor((xy - origin) / cell).astype(int)
    shape = indices.max(axis=0) + 1
    grid = np.zeros(shape, dtype=int)
    np.add.at(grid, (indices[:, 0], indices[:, 1]), 1)

    seed = np.unravel_index(np.argmax(grid), grid.shape)
    member = np.zeros(shape, dtype=bool)
    stack = [seed]
    while stack:
        i, j = stack.pop()
        if not (0 <= i < shape[0] and 0 <= j < shape[1]):
            continue
        if member[i, j] or grid[i, j] == 0:
            continue
        member[i, j] = True
        stack += [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]

    chosen = member[indices[:, 0], indices[:, 1]]
    blob = xy[chosen]
    if len(blob) < min_points:
        return None

    centre = blob.mean(axis=0)
    spread = blob - centre
    # Principal axis -> yaw, folded to (-45, 45] deg: the block is square, so
    # every quarter turn is the same grasp and the nearest one wins.
    covariance = spread.T @ spread / len(spread)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    yaw = math.atan2(axis[1], axis[0])
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return float(centre[0]), float(centre[1]), float(yaw)


class DepthBlockDetector:
    """Feeds the head camera's depth stream into the pure detector.

    Subscribes once; `detect` uses the latest frame, transformed through TF at
    that frame's camera pose. The camera link is optically oriented (+z out of
    the lens), so the back-projected axes map as x->x, y->y, z->z directly.
    """

    def __init__(self, node, tf_buffer, camera="head_d435", camera_frame="head_d435_Link",
                 fy_override=0.0):
        from sensor_msgs.msg import CameraInfo, Image

        self._node = node
        self._buffer = tf_buffer
        self._camera_frame = camera_frame
        # Depth-channel vertical focal length, when it differs from the RGB
        # camera_info. Calibrated by fitting the (known-height) bench plane in
        # the depth image - the standard flat-target depth calibration any
        # RGBD camera goes through on hardware. Zero trusts camera_info.
        self._fy_override = fy_override
        self._lock = threading.Lock()
        self._depth = None
        self._rgb = None
        self._intrinsics = None
        node.create_subscription(
            Image, f"{camera}/depth/image_rect_raw", self._on_depth, 5
        )
        node.create_subscription(
            Image, f"{camera}/color/image_raw", self._on_rgb, 5
        )
        node.create_subscription(
            CameraInfo, f"{camera}/color/camera_info", self._on_info, 5
        )

    def _on_depth(self, message):
        depth = np.frombuffer(message.data, np.float32).reshape(
            message.height, message.width
        )
        with self._lock:
            self._depth = depth

    def _on_rgb(self, message):
        image = np.frombuffer(message.data, np.uint8).reshape(
            message.height, message.width, -1
        )[..., :3]
        with self._lock:
            self._rgb = image

    def _on_info(self, message):
        intrinsics = np.array(message.k, dtype=float).reshape(3, 3)
        if self._fy_override > 0.0:
            intrinsics[1][1] = self._fy_override
        with self._lock:
            self._intrinsics = intrinsics

    def _save_debug(self, rgb, annotations, debug_dir, label, found):
        """The annotated detection frame, one per attempt, for the run log."""
        import os
        import time as _time

        import cv2

        os.makedirs(debug_dir, exist_ok=True)
        verdict = "found" if found is not None else "nothing"
        header = f"{label or 'detect'}: {verdict}"
        if found is not None:
            header += f" ({found[0]:.3f}, {found[1]:.3f}) yaw {math.degrees(found[2]):.1f}deg"
        canvas = draw_annotations(rgb[..., ::-1], annotations, header)
        stamp = _time.strftime("%H%M%S")
        cv2.imwrite(os.path.join(debug_dir, f"{label or 'detect'}_{stamp}.png"), canvas)

    def detect(self, bench_top, search_centre, part_height=0.04, source="rgb",
               timeout=3.0, debug_dir="", label=""):
        """(x, y, yaw) of the block in the planning frame, or None.

        ``source`` picks the sensing path: "rgb" intersects pixel rays with
        the part's known top plane (plan section 5.4's monocular recipe;
        immune to depth-channel intrinsics), "depth" clusters the height band
        above the bench (preferable on hardware, where the depth camera's
        factory calibration is trustworthy).
        """
        import time

        from rclpy.time import Time
        from tf2_ros import TransformException

        from franzi_pick_place.transforms import from_transform_msg

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                depth = self._depth
                rgb = self._rgb
                intrinsics = self._intrinsics
            if intrinsics is None or (depth is None and rgb is None):
                time.sleep(0.1)
                continue
            try:
                message = self._buffer.lookup_transform(
                    "odom", self._camera_frame, Time()
                )
            except TransformException:
                time.sleep(0.1)
                continue
            camera_to_world = from_transform_msg(message.transform)

            found = None
            if source == "rgb" and rgb is not None:
                annotations = [] if debug_dir else None
                found = block_pose_from_rgb(
                    rgb,
                    intrinsics,
                    camera_to_world,
                    bench_top + part_height,
                    search_centre,
                    annotations=annotations,
                )
                if debug_dir:
                    self._save_debug(rgb, annotations, debug_dir, label, found)
            elif depth is not None:
                points = back_project(depth, intrinsics)
                world = points @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]
                found = block_pose_from_points(world, bench_top, search_centre)
            if found is not None:
                return found
            time.sleep(0.2)  # maybe mid-motion blur or an empty frame
        return None


def block_pose_from_rgb(
    image,
    intrinsics,
    camera_to_world,
    plane_z,
    search_centre,
    search_radius=0.35,
    size_range=(0.045, 0.11),
    contrast_margin=20,
    dark_level=80,
    max_dark_ratio=0.06,
    min_pixels=250,
    annotations=None,
):
    """The block's (x, y, yaw) from a colour image and the known rest plane.

    Plan section 5.4's monocular recipe, verbatim: the part lies on a fixed
    plane, so a pixel ray intersected with that plane recovers its position,
    and z is not measured at all. Segmentation is deliberately dumb - the
    aluminium blank is the only large bright region that contains no black -
    which is exactly what separates it from the tag (black cells) and the
    bench (dark grey). The same intrinsics chain that puts apriltag within
    millimetres carries this measurement, so no separate depth calibration
    exists to drift.
    """
    import cv2

    gray = image.max(axis=2) if image.ndim == 3 else image
    # The bench dominates a look-down frame, so its brightness is the image
    # median; the part only has to be brighter than the bench it lies on.
    # Any fixed threshold fails eventually - too high loses a dim part, too
    # low welds the bench into one giant blob.
    level = float(np.median(gray)) + contrast_margin
    white = (gray > level).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=4)
    # Black pixels, grown by a few pixels: anything white that borders black
    # is part of the tag's cell pattern, however clean the white patch itself
    # is. Without this, the tag's central white cells - the brightest, purest
    # blobs in the image - impersonate the part.
    dark_halo = cv2.dilate(
        (gray < dark_level).astype(np.uint8), np.ones((7, 7), np.uint8)
    ).astype(bool)

    fx, fy = intrinsics[0][0], intrinsics[1][1]
    cx, cy = intrinsics[0][2], intrinsics[1][2]
    rotation = camera_to_world[:3, :3]
    camera_at = camera_to_world[:3, 3]

    def to_plane(u, v):
        ray = rotation @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        if abs(ray[2]) < 1e-6:
            return None
        t = (plane_z - camera_at[2]) / ray[2]
        if t <= 0:
            return None
        return camera_at + t * ray

    def note(rect, verdict, detail=""):
        if annotations is not None:
            annotations.append((cv2.boxPoints(rect), verdict, detail))

    best = None
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] < min_pixels:
            continue
        mask = labels == index
        points = np.column_stack(np.nonzero(mask))  # (v, u)
        rect = cv2.minAreaRect(points[:, ::-1].astype(np.float32))
        if (gray[mask] < dark_level).mean() > max_dark_ratio or dark_halo[mask].mean() > 0.15:
            note(rect, "tag")  # black cells inside or all around: tag, not part
            continue
        (u0, v0), (w_px, h_px), angle = rect
        centre = to_plane(u0, v0)
        if centre is None:
            continue
        # World-frame edge lengths, from the rect's corner projections.
        corners = cv2.boxPoints(rect)
        world = [to_plane(u, v) for u, v in corners]
        if any(p is None for p in world):
            continue
        edge_a = float(np.linalg.norm(world[1][:2] - world[0][:2]))
        edge_b = float(np.linalg.norm(world[2][:2] - world[1][:2]))
        if not (size_range[0] < edge_a < size_range[1] and size_range[0] < edge_b < size_range[1]):
            note(rect, "size", f"{edge_a * 100:.0f}x{edge_b * 100:.0f}cm")
            continue
        offset = centre[:2] - np.asarray(search_centre)
        distance = float(np.hypot(*offset))
        if distance > search_radius:
            note(rect, "far", f"{distance:.2f}m")
            continue
        edge = world[1][:2] - world[0][:2]
        yaw = math.atan2(edge[1], edge[0])
        yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
        candidate = (distance, float(centre[0]), float(centre[1]), float(yaw), rect)
        note(rect, "candidate", f"({centre[0]:.3f}, {centre[1]:.3f})")
        if best is None or distance < best[0]:
            best = candidate

    if best is None:
        return None
    if annotations is not None:
        _, x, y, yaw, rect = best
        annotations.append(
            (cv2.boxPoints(rect), "chosen", f"({x:.3f}, {y:.3f}) yaw {math.degrees(yaw):.1f}deg")
        )
    return best[1], best[2], best[3]


ANNOTATION_COLOURS = {
    "tag": (60, 60, 230),        # red: rejected, contains black cells
    "size": (0, 160, 255),       # orange: wrong world-frame size
    "far": (60, 200, 230),       # yellow: outside the search window
    "candidate": (230, 160, 60), # blue: passed every gate
    "chosen": (80, 220, 80),     # green: the grasp target
}


def draw_annotations(image, annotations, header=""):
    """The detection, explained on the image itself: every bright region with
    its verdict, the chosen part in green with its world pose."""
    import cv2

    canvas = image.copy()
    if canvas.ndim == 2:
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    for corners, verdict, detail in annotations:
        colour = ANNOTATION_COLOURS.get(verdict, (200, 200, 200))
        box = corners.astype(int)
        thickness = 3 if verdict == "chosen" else 1
        cv2.polylines(canvas, [box], True, colour, thickness)
        label = verdict if not detail else f"{verdict} {detail}"
        anchor = box.min(axis=0)
        cv2.putText(canvas, label, (int(anchor[0]), max(12, int(anchor[1]) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
    if header:
        cv2.putText(canvas, header, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return canvas
