# Repository Guidelines

## Project Structure & Module Organization

This repository groups robot-simulation work by runtime:

- `IssacSim/` contains NVIDIA Isaac Sim-specific scenes, scripts, and setup notes. Keep its existing spelling.
- `Mujoco/` contains MuJoCo models, environments, and runtime-specific tooling.
- `Rviz/` contains RViz configurations, launch files, and visualization assets.

Each directory currently contains a placeholder `README.md`; there is no shared source, test, or asset directory. Keep engine-specific files in their matching directory. Create a shared top-level location only for cross-runtime assets or utilities, and document its ownership in a README.

## Build, Test, and Development Commands

No build system, package manifest, or automated test command is tracked. Work from the relevant runtime directory and document each added workflow in its README, including prerequisites and an exact run command.

Useful repository checks:

```bash
git status --short       # review local changes
git ls-files             # list tracked repository files
```

Do not add a root-level command wrapper until at least two runtimes need the same workflow.

## Coding Style & Naming Conventions

No formatter or language-specific configuration has been established. Follow the formatter and conventions native to the runtime or language being added. For new Python code, use four-space indentation, `snake_case` modules and functions, and `PascalCase` classes. Prefer descriptive lowercase filenames such as `pick_and_place_scene.py` or `warehouse.rviz`.

Keep simulator paths, versions, and launch parameters configurable rather than embedding machine-specific paths in code.

## Testing Guidelines

There is no test framework or coverage target at present. New executable code must include a reproducible validation command in its README. Add automated tests alongside the owning runtime when practical, using its standard framework and clear names such as `test_<behavior>.py`. For visual changes, verify the scene or RViz configuration in the target application and record the simulator version used.

## Commit and Pull Request Guidelines

Existing history uses short, lowercase imperative subjects, for example `create Rviz folder`. Continue that style and keep each commit focused, such as `add mujoco arm model`.

Pull requests should describe the affected runtime, required simulator version or dependencies, validation performed, and any changed assets. Link relevant issues and include screenshots or a short recording for scene or visualization changes.

## Configuration and Asset Safety

Do not commit credentials, local absolute paths, generated caches, or restricted simulation assets. Provide sanitized example configuration and document required environment variables or external asset sources in the applicable README.
