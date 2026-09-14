#!/usr/bin/env bash
# Launch the Isaac cell with ROS 2 publishing.
#
# Isaac's bridge needs the ROS 2 C libraries on LD_LIBRARY_PATH, but must keep
# its own Python: sourcing setup.bash wholesale puts ROS's site-packages on
# PYTHONPATH and Isaac then imports the wrong rclpy. Only the library path and
# the middleware settings are carried across.
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROS_ROOT="/opt/ros/${ROS_DISTRO_NAME}"
ISAAC_ENV_NAME="${ISAAC_ENV_NAME:-env_isaaclab_6}"

# Allow an explicit interpreter override, but otherwise resolve the named
# Conda environment from the current user's Conda installation.  This avoids
# embedding the previous developer's home directory in the launcher.
if [[ -z "${ISAAC_PYTHON:-}" ]]; then
  if ! command -v conda >/dev/null 2>&1; then
    echo "Conda is required to find ${ISAAC_ENV_NAME}; set ISAAC_PYTHON explicitly" >&2
    exit 1
  fi
  ISAAC_PYTHON="$(conda info --base)/envs/${ISAAC_ENV_NAME}/bin/python"
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "${ROS_ROOT}" ]]; then
  echo "no ROS 2 at ${ROS_ROOT}" >&2
  exit 1
fi

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "no Isaac Python at ${ISAAC_PYTHON}" >&2
  echo "create or select the Conda environment with ISAAC_ENV_NAME or ISAAC_PYTHON" >&2
  exit 1
fi

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${ROS_ROOT}/lib:${LD_LIBRARY_PATH:-}"
# Do not allow packages installed under ~/.local to override the versions
# pinned in the dedicated Isaac Sim 6 environment.
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"

exec "${ISAAC_PYTHON}" -u "${HERE}/ros2_cell.py" "$@"
