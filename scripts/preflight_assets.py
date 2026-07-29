#!/usr/bin/env python3
"""Validate the local NVIDIA asset root and create its relative project mount."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MOUNT_PATH = PROJECT_ROOT / "vendor" / "isaac_assets"


@dataclass(frozen=True)
class AssetPreflight:
    """Validated paths required by project USD composition."""

    root: Path
    warehouse_usd: Path


def find_warehouse_usd(asset_root: Path) -> Path | None:
    """Return the first official Warehouse USD under the configured root."""

    candidates = (
        asset_root / "Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
        asset_root / "Isaac/Environments/Simple_Warehouse/warehouse.usd",
        asset_root / "Isaac/Environments/Modular_Warehouse/warehouse.usd",
    )
    # Fixed 5.1 locations avoid a costly recursive scan of the full asset tree at startup.
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def validate_asset_root(asset_root: Path) -> AssetPreflight:
    """Validate an Isaac 5.1 asset root and locate an official Warehouse USD."""

    missing = [name for name in ("Isaac", "NVIDIA") if not (asset_root / name).is_dir()]
    if missing:
        raise ValueError(f"asset root is missing required directories: {', '.join(missing)}")
    warehouse_usd = find_warehouse_usd(asset_root)
    if warehouse_usd is None:
        raise ValueError("no official Warehouse USD found below asset root")
    return AssetPreflight(root=asset_root, warehouse_usd=warehouse_usd)


def mount_assets(asset_root: Path, mount_path: Path = MOUNT_PATH) -> Path:
    """Create or verify the ignored, relative project symlink to local assets."""

    mount_path.parent.mkdir(parents=True, exist_ok=True)
    if mount_path.is_symlink():
        if mount_path.resolve() == asset_root.resolve():
            return mount_path
        mount_path.unlink()
    elif mount_path.exists():
        raise FileExistsError(f"mount path exists and is not a symlink: {mount_path}")
    # A symlink keeps USD references reproducible without copying the NVIDIA asset bundle.
    mount_path.symlink_to(asset_root, target_is_directory=True)
    return mount_path


def main() -> int:
    """Run the asset-root validation command."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=os.environ.get("ISAAC_ASSET_ROOT"))
    parser.add_argument("--mount", action="store_true", help="create vendor/isaac_assets symlink")
    args = parser.parse_args()
    if args.asset_root is None:
        parser.error("set ISAAC_ASSET_ROOT or pass --asset-root")
    result = validate_asset_root(args.asset_root.expanduser().resolve())
    if args.mount:
        mount_assets(result.root)
    print(f"asset root: {result.root}")
    print(f"warehouse: {result.warehouse_usd.relative_to(result.root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
