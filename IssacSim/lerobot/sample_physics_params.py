#!/usr/bin/env python3
"""Deterministically sample per-episode PhysX parameters from the JSON config."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def sample_parameters(config: dict, episode_index: int) -> dict:
    if episode_index < 0:
        raise ValueError("episode_index must be non-negative")
    base_seed = int(config["base_seed"])
    episode_seed = (base_seed + episode_index) % (2**32)
    randomization = config["physics_randomization"]
    if len(randomization) != 8:
        raise ValueError(
            "physics_randomization must contain exactly eight parameters, "
            f"got {len(randomization)}"
        )

    rng = np.random.default_rng(episode_seed)
    values = {}
    for name, specification in randomization.items():
        if specification["distribution"] != "uniform":
            raise ValueError(
                f"Unsupported distribution for {name}: "
                f"{specification['distribution']}"
            )
        low = float(specification["low"])
        high = float(specification["high"])
        if not low <= high:
            raise ValueError(f"Invalid range for {name}: {low} > {high}")
        values[name] = {
            "value": float(rng.uniform(low, high)),
            "unit": specification.get("unit"),
            "range": [low, high],
        }
    return {
        "base_seed": base_seed,
        "episode_index": episode_index,
        "episode_seed": episode_seed,
        "physics_engine": config["runtime"]["physics_engine"],
        "parameters": values,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("physics_collection.json"),
    )
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")

    config = json.loads(args.config.read_text())
    samples = [
        sample_parameters(config, args.episode_index + offset)
        for offset in range(args.count)
    ]
    print(json.dumps(samples[0] if args.count == 1 else samples, indent=2))


if __name__ == "__main__":
    main()
