# Franzi description

URDF, meshes, and joint-name configuration for the Franzi mobile manipulator.
The model is `wheel_robot_26.8.16_3`, exported from SolidWorks.

## The export is not used verbatim

`urdf/wheel_robot_26.8.16_3.urdf` is the exporter's output with one block of
corrections appended at the very end of the file. Everything above that block
is the export byte-for-byte, so a new export can be dropped in and only the
block re-applied.

**1. A REP-103 root.** The export is built against a reference frame in which
the robot stands along **+X**. Expressing a point in the exported chassis frame
gives `p_export = P · p_rep103 + t`, with `P` mapping `(x, y, z) → (z, x, y)`
and `t = (-0.24785, 0, 0)`. Every link matches the previous `wheel_robot_4.0`
model under that permutation to within 0.05 mm, so this is purely a frame
convention difference and not a geometry change — but REP-103 wants x forward /
y left / z up, and the planar `world_joint`, the base driver and the lidars all
depend on it.

The exported root is therefore renamed `base_body_Link`, and a massless
`base_link` is put in front of it carrying `P⁻¹` and `-P⁻¹t`. `base_link` keeps
both the orientation *and* the origin the previous model used, which is what
the heights in `franzi_pick_place/config/task.yaml` are measured against.

Consequence for consumers: **the chassis mesh and inertia are on
`base_body_Link`**, not on `base_link`. The MoveIt collision matrix names
`base_body_Link` for that reason.

**2. SDK flange and TCP frames.** The flange origin is the J7 wrist-roll axis
centre. In this export that is exactly the `wrist_roll_Link` frame itself
(identity on both arms), so `left_flange` / `right_flange` are plain aliases
and each TCP hangs 273.5 mm along flange +X, with opposite X-axis rolls per
side. In the previous model the flange sat 30.3 mm off `wrist_roll_Link`; the
export moved that link frame onto the flange, so the numbers are unchanged but
the parent is not.

## What changed against `wheel_robot_4.0`

- `leftfinger1/2_Link` → `left_finger01/02_Link` (and the same on the right)
- `left_wrist_d405_Link` → `left_D405_Link`
- new `left/right_gripper_link`: the fingers and the wrist D405 now hang off a
  gripper mount instead of directly off `wrist_roll_Link`
- new fixed `left/right_j0_Link` between the torso and each shoulder
- `shoulder_pitch` and `elbow_pitch` limits changed sign, so stored joint
  values for those joints from the old model must be negated
- the torso mount moved 15.7 mm; the head joint compensates, so the head, the
  cameras, the lidars and the wheels all land within 0.15 mm of the old model

Note the export's own inconsistency: the left gripper mount is
`left_gripper_link` (lowercase) and the right is `right_gripper_Link`.

## Validate

From the `Rviz/` workspace root, with no ROS environment needed:

```bash
python3 src/franzi_description/scripts/check_description.py
```

This confirms the tree has a single `base_link` root, that every referenced
mesh exists, that `base_link` really is REP-103, that the flange/TCP frames
match the SDK convention, and that each gripper spans 95 mm open and 0 mm
closed. Run it after any re-export.
