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
ISAAC_PYTHON="${ISAAC_PYTHON:-/home/phl/miniconda3/envs/env_isaaclab/bin/python}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "${ROS_ROOT}" ]]; then
  echo "no ROS 2 at ${ROS_ROOT}" >&2
  exit 1
fi

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${ROS_ROOT}/lib:${LD_LIBRARY_PATH:-}"

exec "${ISAAC_PYTHON}" -u "${HERE}/ros2_cell.py" "$@"
