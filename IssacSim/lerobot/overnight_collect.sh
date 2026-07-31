#!/usr/bin/env bash
# 通宵无人值守采集:等当前批次结束 -> 连续采到目标条数(每批 10 条,
# 批间查磁盘)-> 全部转换成 LeRobot v3.0 -> 校验 -> 软链到 dataset/Robot。
# 用 setsid nohup 启动,不依赖任何终端/会话。
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="${REPO}/IssacSim/lerobot"
EPISODES_DIR="${REPO}/IssacSim/datasets/episodes"
DATASET_OUT="${REPO}/IssacSim/datasets/franzi_machine_tending_physx_v3"
LINK_TARGET="/home/phl/workspace/dataset/Robot/franzi_machine_tending_sim_v3"
DOMAIN=45
START=5            # 0-4 由白天的批次负责
END=84             # 含;约 80 条 x ~8-10 分钟
MIN_FREE_GB=20
GROOT_PY="/home/phl/miniconda3/envs/lerobot-groot/bin/python"

log() { echo "[overnight $(date '+%m-%d %H:%M:%S')] $*"; }

log "等待白天批次(0-4)收尾……"
while pgrep -f "collect_batch.sh 0 5" > /dev/null; do sleep 60; done
log "开始通宵采集 episode ${START}..${END}(DOMAIN ${DOMAIN})"

for chunk_start in $(seq "${START}" 10 "${END}"); do
  remaining=$(( END - chunk_start + 1 ))
  count=$(( remaining < 10 ? remaining : 10 ))
  free_gb=$(df --output=avail -BG "${REPO}" | tail -1 | tr -dc '0-9')
  if (( free_gb < MIN_FREE_GB )); then
    log "磁盘只剩 ${free_gb}G(<${MIN_FREE_GB}G),提前停止采集"
    break
  fi
  log "批 ${chunk_start}..$(( chunk_start + count - 1 ))(磁盘余 ${free_gb}G)"
  COLLECT_FPS=8 "${HERE}/collect_batch.sh" "${chunk_start}" "${count}" "${DOMAIN}" \
    || log "该批有失败条,继续"
done

good=$(ls "${EPISODES_DIR}"/episode_*/telemetry.npz 2>/dev/null | wc -l)
log "采集阶段结束,共 ${good} 条有 telemetry 的 episode,开始转换"

if [[ -e "${DATASET_OUT}" ]]; then
  DATASET_OUT="${DATASET_OUT}_$(date +%H%M)"
  log "输出目录已存在,改用 ${DATASET_OUT}"
fi
"${GROOT_PY}" "${HERE}/build_lerobot_v3_dataset.py" \
  --episodes "${EPISODES_DIR}" \
  --output "${DATASET_OUT}" \
  --repo-id local/franzi_machine_tending_physx_v3 \
  && log "转换完成 -> ${DATASET_OUT}" \
  || { log "转换失败,原始 episodes 保留在 ${EPISODES_DIR}"; exit 1; }

"${GROOT_PY}" "${HERE}/validate_lerobot_v3.py" \
  --root "${DATASET_OUT}" \
  --repo-id local/franzi_machine_tending_physx_v3 \
  --require-actions --decode-sample \
  > "${DATASET_OUT}/meta/validation.json" \
  && log "校验通过,报告在 meta/validation.json" \
  || log "校验失败,看 meta/validation.json"

mkdir -p "$(dirname "${LINK_TARGET}")"
ln -sfn "${DATASET_OUT}" "${LINK_TARGET}"
log "软链 ${LINK_TARGET} -> ${DATASET_OUT}"
log "全部完成"
