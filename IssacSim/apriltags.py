"""Real AprilTag markers: generated as textures, decoded from rendered images.

The RViz side simulates detection geometrically - it computes where the tag
would be and publishes that, without an image ever existing. Here the marker is
an actual 36h11 texture on the bench and the detector is the same OpenCV code
that would run on hardware, so a detection failure here means a detection
failure there.

Generation and detection live together because they have to agree on two
things: the dictionary, and what "tag size" means. It is the outer edge of the
black border, not the printed sheet, and getting that wrong scales every pose
estimate.
"""

from pathlib import Path

import cv2
import numpy as np

DICTIONARY_ID = cv2.aruco.DICT_APRILTAG_36h11
# White margin around the marker, as a fraction of the marker's own width. A
# tag with no quiet zone against a dark bench is much harder to find.
QUIET_FRACTION = 0.125


def dictionary():
    return cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)


def sheet_scale():
    """How much wider the printed sheet is than the marker itself."""
    return 1.0 + 2.0 * QUIET_FRACTION


def write_marker(tag_id: int, path: Path, marker_pixels: int = 900) -> Path:
    """Write a 36h11 marker with a quiet zone, ready to use as a texture."""
    marker = cv2.aruco.generateImageMarker(dictionary(), tag_id, marker_pixels)
    margin = int(round(marker_pixels * QUIET_FRACTION))
    sheet = np.full(
        (marker_pixels + 2 * margin, marker_pixels + 2 * margin), 255, np.uint8
    )
    sheet[margin : margin + marker_pixels, margin : margin + marker_pixels] = marker

    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), sheet)
    return path


def _detector():
    parameters = cv2.aruco.DetectorParameters()
    # Sub-pixel corners: the difference between a usable pose and a wobbly one.
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary(), parameters)


def detect(image, intrinsics, marker_size):
    """Find markers and solve their pose in the camera's optical frame.

    ``intrinsics`` is the 3x3 camera matrix. Returns ``{id: (position, corners)}``
    with the position in metres, x right, y down, z forward - OpenCV's frame,
    which is also the ROS optical frame.
    """
    grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
    corners, ids, _ = _detector().detectMarkers(grey)
    if ids is None:
        return {}

    half = marker_size / 2.0
    # Marker frame: x right, y up, z out of the face; corner order matches
    # what detectMarkers returns (top-left, top-right, bottom-right, bottom-left).
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )

    found = {}
    for corner, tag_id in zip(corners, ids.flatten()):
        ok, rotation, translation = cv2.solvePnP(
            object_points,
            corner.reshape(4, 2).astype(np.float32),
            np.asarray(intrinsics, dtype=np.float64),
            np.zeros(5),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            continue
        found[int(tag_id)] = (translation.reshape(3), corner.reshape(4, 2))
    return found
