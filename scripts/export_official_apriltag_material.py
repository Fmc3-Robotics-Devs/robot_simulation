#!/usr/bin/env python3
"""Export the AprilTag material authored by Isaac Sim's official GUI command.

This maintenance utility is intentionally small: it lets reviewers compare the
project asset with the exact USD schema emitted by ``Create -> April Tags`` in
the installed Isaac Sim version.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET_ROOT = PROJECT_ROOT / "vendor/isaac_assets"


def parse_args() -> argparse.Namespace:
    """Parse local Isaac asset root and USD output path."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSET_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    """Run Isaac's material command and export the resulting text layer."""

    args = parse_args()
    asset_root = args.asset_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    mdl = asset_root / "Isaac/Materials/AprilTag/AprilTag.mdl"
    mosaic = asset_root / "Isaac/Materials/AprilTag/Textures/tag36h11.png"
    if not mdl.is_file() or not mosaic.is_file():
        raise FileNotFoundError(f"AprilTag source assets missing below {asset_root}")

    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
    finally:
        sys.argv = script_argv

    try:
        import omni.kit.commands
        import omni.usd
        from pxr import Sdf

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        omni.kit.commands.execute(
            "CreateMdlMaterialPrim",
            mtl_url=str(mdl),
            mtl_name="AprilTag",
            mtl_path="/Looks/AprilTag",
            select_new_prim=False,
        )
        shader = stage.GetPrimAtPath("/Looks/AprilTag/Shader")
        shader.CreateAttribute("inputs:tag_mosaic", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(str(mosaic))
        )
        for _ in range(8):
            app.update()
        output.parent.mkdir(parents=True, exist_ok=True)
        if not stage.GetRootLayer().Export(str(output)):
            raise RuntimeError(f"failed to export official material: {output}")
        sys.__stdout__.write(f"{output}\n")
        sys.__stdout__.flush()
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
