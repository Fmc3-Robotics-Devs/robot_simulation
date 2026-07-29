#!/usr/bin/env python3
"""Inspect a USD asset's prim tree and world-space bounds in headless Isaac Sim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def parse_args() -> argparse.Namespace:
    """Parse the asset path and optional JSON destination."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd", type=Path, nargs="+")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-prims", type=int, default=80)
    return parser.parse_args()


def main() -> int:
    """Open the asset, calculate its default-prim bounds, and emit JSON."""

    args = parse_args()
    usd_paths = [path.expanduser().resolve() for path in args.usd]
    missing = [path for path in usd_paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"USD assets do not exist: {missing}")
    if args.max_prims <= 0:
        raise ValueError("--max-prims must be positive")

    from isaacsim import SimulationApp

    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = script_argv

    try:
        from pxr import Usd, UsdGeom

        assets = []
        for usd_path in usd_paths:
            stage = Usd.Stage.Open(str(usd_path))
            if stage is None:
                raise RuntimeError(f"Isaac Sim could not open USD: {usd_path}")
            default_prim = stage.GetDefaultPrim()
            if not default_prim or not default_prim.IsValid():
                raise RuntimeError(f"asset has no valid defaultPrim: {usd_path}")

            purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
            box = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                purposes,
            ).ComputeWorldBound(default_prim).ComputeAlignedBox()
            minimum, maximum = box.GetMin(), box.GetMax()
            meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
            size_stage_units = [
                float(maximum[index] - minimum[index]) for index in range(3)
            ]
            prims = [
                {"path": str(prim.GetPath()), "type": prim.GetTypeName()}
                for prim in stage.Traverse()
            ]
            assets.append(
                {
                    "usd": str(usd_path),
                    "default_prim": str(default_prim.GetPath()),
                    "prim_count": len(prims),
                    "meters_per_unit": meters_per_unit,
                    "bounds_stage_units": {
                        "min": [float(value) for value in minimum],
                        "max": [float(value) for value in maximum],
                        "size": size_stage_units,
                    },
                    "bounds_m": {
                        "min": [float(value) * meters_per_unit for value in minimum],
                        "max": [float(value) * meters_per_unit for value in maximum],
                        "size": [value * meters_per_unit for value in size_stage_units],
                    },
                    "prims": prims[: args.max_prims],
                    "prims_truncated": len(prims) > args.max_prims,
                }
            )
        result = {"ok": True, "assets": assets}
        serialized = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(serialized, encoding="utf-8")
        sys.__stdout__.write(serialized)
        sys.__stdout__.flush()
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
