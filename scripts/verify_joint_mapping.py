#!/usr/bin/env python3
"""Run the strict Wheel Bot joint-map verifier from a checkout.

The binary robot USD requires Kit's USD plug-ins.  Source-only checks remain
fast, while the default path boots a headless Isaac Sim application before the
strict verifier imports ``pxr``.
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source/franzi_sim"))


def run() -> int:
    """Bootstrap Kit only when the caller requests binary USD validation."""

    app = None
    if "--skip-usd" not in sys.argv:
        from isaacsim import SimulationApp

        script_argv = sys.argv
        sys.argv = [sys.argv[0]]
        try:
            app = SimulationApp({"headless": True})
        finally:
            sys.argv = script_argv
    try:
        from franzi_sim.control.validate_joint_map import main

        return main()
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(run())
