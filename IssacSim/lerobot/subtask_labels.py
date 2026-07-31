"""状态机状态 -> 英文 subtask 标签的唯一映射表。

采集器逐帧记录 `task/state` 的状态名,转换器用这里的映射把状态序列压成
subtask 段(连续同标签合并)。粒度与 LW-BenchHub 的 subtask_fine 数据集
对齐:每个"机器人在干一件事"为一段;纯机床侧的等待并入相邻的机器人
动作段之前,单独成段(策略也要学会"等")。

映射覆盖 states.yaml 的全部状态;遇到未知状态转换器直接报错,不静默。
"""

from __future__ import annotations

STATE_TO_SUBTASK = {
    "IDLE": "hold position and wait for the cycle to start",
    "SYSTEM_CHECK": "hold position and wait for the cycle to start",
    "GO_RAW_STATION": "navigate from home to the feeder station",
    "DOCK_RAW_STATION": "precisely dock at the feeder station",
    "DETECT_MATERIAL": "look down and detect the raw workpiece on the feeder",
    "PICK_MATERIAL": "pick up the raw aluminum workpiece from the feeder",
    "VERIFY_PICK": "pick up the raw aluminum workpiece from the feeder",
    "MOVE_TRANSPORT_POSE": "tuck the arm into the transport pose",
    "GO_MACHINE": "navigate to the engraving machine",
    "DOCK_MACHINE": "precisely dock at the engraving machine",
    "CHECK_MACHINE_READY": "wait for the machine door and clamp to open",
    "OPEN_DOOR": "wait for the machine door and clamp to open",
    "OPEN_CLAMP": "wait for the machine door and clamp to open",
    "LOAD_MACHINE": "load the workpiece into the machine fixture",
    "CLOSE_CLAMP": "load the workpiece into the machine fixture",
    "EXIT_MACHINE": "retreat from the machine envelope",
    "CLOSE_DOOR": "retreat from the machine envelope",
    "START_MACHINING": "retreat from the machine envelope",
    "GO_HOME_WAIT": "navigate to the standby point while the machine runs",
    "WAIT_MACHINING": "wait at the standby point while the machine runs",
    "GO_MACHINE_UNLOAD": "navigate back to the engraving machine",
    "DOCK_MACHINE_UNLOAD": "precisely dock at the engraving machine for unloading",
    "OPEN_DOOR_UNLOAD": "wait for the machine to open for unloading",
    "OPEN_CLAMP_UNLOAD": "wait for the machine to open for unloading",
    "UNLOAD_MACHINE": "take the finished part out of the machine fixture",
    "VERIFY_UNLOAD": "take the finished part out of the machine fixture",
    "EXIT_MACHINE_UNLOAD": "retreat from the machine with the finished part",
    "GO_OUTPUT": "navigate to the outfeed station",
    "DOCK_OUTPUT": "precisely dock at the outfeed station",
    "PLACE_PRODUCT": "place the finished part into the next empty outfeed slot",
    "RETURN_HOME": "return to the home standby point",
    "GO_HOME_END": "return to the home standby point",
    "DONE": "return to the home standby point",
    # 异常路径:episode 会被标记失败,默认不进数据集,但标签仍要有定义。
    "RECOVERY": "recover to a safe posture after a failure",
    "FAULT": "recover to a safe posture after a failure",
}


def segments_from_states(states: list[str]) -> list[dict]:
    """把逐帧状态序列压成 [{subtask, start, end_inclusive}] 段列表。"""
    unknown = sorted(set(states) - set(STATE_TO_SUBTASK))
    if unknown:
        raise ValueError(f"states.yaml 之外的状态: {unknown}")
    segments: list[dict] = []
    for index, state in enumerate(states):
        label = STATE_TO_SUBTASK[state]
        if segments and segments[-1]["subtask"] == label:
            segments[-1]["end_inclusive"] = index
        else:
            segments.append(
                {"subtask": label, "start": index, "end_inclusive": index}
            )
    return segments
