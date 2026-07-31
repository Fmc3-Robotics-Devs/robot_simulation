#!/usr/bin/env bash
# 批量采集 tending episode:每条 episode 一次 Isaac + ROS 栈的完整起落。
#
#   IssacSim/lerobot/collect_batch.sh <起始序号> <条数> [ROS_DOMAIN_ID]
#
# 每条的物理随机化由 --physics-episode(Isaac 侧)与 --episode-index
# (采集器)按同一 base_seed+index 确定性采样。并行采集 = 两个终端各跑
# 一份,序号区间错开、ROS_DOMAIN_ID 不同(如 42 和 43)。
set -uo pipefail

START="${1:?用法: collect_batch.sh <起始序号> <条数> [domain_id]}"
COUNT="${2:?用法: collect_batch.sh <起始序号> <条数> [domain_id]}"
DOMAIN="${3:-42}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
EPISODES_DIR="${EPISODES_DIR:-${REPO}/IssacSim/datasets/episodes}"
LOG_DIR="${EPISODES_DIR}/logs"
mkdir -p "${EPISODES_DIR}" "${LOG_DIR}"

export ROS_DOMAIN_ID="${DOMAIN}"

ros_env() {
  # 子 shell 内加载 ROS + 工作区;collector 用系统 python3(带 rclpy)。
  # setup.bash 与 set -u 不兼容(AMENT_TRACE_SETUP_FILES 未绑定),先关。
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${REPO}/Rviz/install/setup.bash"
  set -u
}

wait_topic() {
  local topic="$1" deadline=$(( $(date +%s) + ${2:-240} ))
  while (( $(date +%s) < deadline )); do
    if (ros_env && timeout 6 ros2 topic echo --once "${topic}" >/dev/null 2>&1); then
      return 0
    fi
    sleep 3
  done
  return 1
}

kill_tree() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 0
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

overall=0
for (( i = START; i < START + COUNT; i++ )); do
  ep=$(printf 'episode_%04d' "${i}")
  if [[ -f "${EPISODES_DIR}/${ep}/telemetry.npz" ]]; then
    echo "[batch] ${ep} 已有 telemetry,跳过"
    continue
  fi
  if [[ -d "${EPISODES_DIR}/${ep}" ]]; then
    echo "[batch] ${ep} 是残缺目录,清掉重采"
    rm -rf "${EPISODES_DIR:?}/${ep}"
  fi
  echo "[batch] === ${ep} (domain ${DOMAIN}) ==="

  # Isaac 偶发启动后渲染管线堵死(相机拓扑在但不出帧,episode_0004 实例):
  # 就绪后实测 head 帧率,太低就杀掉整个 Isaac 重来,最多三次。
  ISAAC_PID=""
  for isaac_try in 1 2 3; do
    setsid env ROS_DOMAIN_ID="${DOMAIN}" \
      "${REPO}/IssacSim/run_ros2_cell.sh" \
      --cameras head_d435 left_wrist_d405 right_wrist_d405 \
      --capture dataset \
      --physics-episode "${i}" \
      > "${LOG_DIR}/${ep}_isaac.log" 2>&1 &
    ISAAC_PID=$!
    if ! wait_topic "/head_d435/color/camera_info" 300; then
      echo "[batch] ${ep}: Isaac 相机 300s 未就绪(第 ${isaac_try} 次)"
      kill_tree "${ISAAC_PID}"; ISAAC_PID=""; continue
    fi
    frames=$( (ros_env && timeout 10 ros2 topic hz /head_d435/color/image_raw 2>&1 \
               | grep -oE "window: [0-9]+" | tail -1 | tr -dc '0-9') || echo 0 )
    if (( ${frames:-0} >= 20 )); then
      echo "[batch] ${ep}: Isaac 就绪(head 10s 内 ${frames} 帧),启动任务栈"
      break
    fi
    echo "[batch] ${ep}: Isaac 渲染坏死(10s 仅 ${frames:-0} 帧),重启(第 ${isaac_try} 次)"
    kill_tree "${ISAAC_PID}"; ISAAC_PID=""
    sleep 5
  done
  if [[ -z "${ISAAC_PID}" ]]; then
    echo "[batch] ${ep}: Isaac 三次都起不来,放弃本条"
    overall=1; continue
  fi

  setsid bash -c "
    source /opt/ros/jazzy/setup.bash
    source '${REPO}/Rviz/install/setup.bash'
    export ROS_DOMAIN_ID='${DOMAIN}'
    exec ros2 launch franzi_skills tending_cell.launch.py \
      backend:=isaac rviz:=false cycles:=1
  " > "${LOG_DIR}/${ep}_stack.log" 2>&1 &
  STACK_PID=$!

  (
    ros_env
    exec python3 "${HERE}/collect_episode.py" \
      --output "${EPISODES_DIR}" --episode-index "${i}" \
      --fps "${COLLECT_FPS:-15}"
  ) > "${LOG_DIR}/${ep}_collect.log" 2>&1
  status=$?
  tail -1 "${LOG_DIR}/${ep}_collect.log" || true

  kill_tree "${STACK_PID}"
  kill_tree "${ISAAC_PID}"
  sleep 5
  if [[ ${status} -ne 0 ]]; then
    echo "[batch] ${ep}: 采集退出码 ${status}(2=任务失败条,已存目录仅缺 success)"
    overall=1
  fi
done
exit "${overall}"
