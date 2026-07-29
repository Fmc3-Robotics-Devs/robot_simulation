"""Offline contract check for Wheel Bot's four static camera optical-frame TF edges."""

from __future__ import annotations

from franzi_sim.optical_frame_tf import optical_frame_transforms, validate_optical_frame_transforms


def main() -> None:
    """Validate and print the static parent-to-child TF graph."""

    validate_optical_frame_transforms()
    for transform in optical_frame_transforms():
        quaternion = ", ".join(f"{component:.6f}" for component in transform.rotation_xyzw)
        print(f"{transform.parent_frame} -> {transform.child_frame}; xyzw=({quaternion})")


if __name__ == "__main__":
    main()
